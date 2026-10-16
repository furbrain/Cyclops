"""
Model viewer: Flask Blueprint.

Drop this module (plus templates/model/, static/, and models/) into a
larger Flask project and register it:

    from model import model_bp
    app.register_blueprint(model_bp)                       # mounted at /models
    app.register_blueprint(model_bp, url_prefix="/caves")   # ...or wherever you want

All routes below are relative to whatever prefix you register it under —
nothing in this module or its template/JS hardcodes "/models". See
get_models_dir() for pointing it at a real data directory instead of the
bundled demo folder.

Assumptions (change these to match your real pipeline — search for "TODO"):
  - Each model lives in  <models dir> / <model_id> /
  - Files expected in that folder:
      info.json           {"name": ..., "keyframes": int, "points": int, "submaps": int}
      pointcloud.json      raw SLAM point cloud, {"points": [[x,y,z],...], "colors": [[r,g,b],...]?}
      sparse.ply            \
      dense.ply              > the "three models" — rename to match your pipeline
      textured.ply           /
      track.json            camera trajectory, same {"points": [...], "colors": [...]} shape
      survey.svx            user-entered/uploaded survey text
      survey.json            leg endpoints generated from survey.svx, same shape as track.json
                              (points are taken in pairs: 0-1 is one leg, 2-3 the next, etc.)
  - Generating a missing asset, and merging submaps, both run as background
    jobs with their live output captured for the frontend to poll — see
    "Background job infrastructure" below. The actual pipeline calls are
    still stubbed with demo output; wire up the real thing there.
"""

from contextlib import contextmanager
import json
import mimetypes
import random
import subprocess
import sys
import threading
import time
from pathlib import Path

from flask import Blueprint, current_app, render_template, request, jsonify, send_from_directory, abort

from utils import CAM_FILENAME, IMAGE_FILENAME, POINTS_FILENAME


# Not registered by Python's mimetypes module by default — set it explicitly
# so the GLB response gets the right Content-Type header (GLTFLoader parses
# the ArrayBuffer regardless, but this keeps things correct for anyone else
# fetching the asset directly, e.g. curl/browser download).
mimetypes.add_type("model/gltf-binary", ".glb")

# static_url_path is absolute and namespaced under /model/static so it can't
# collide with the host app's own /static folder, regardless of what
# url_prefix the routes below end up registered under.
model_bp = Blueprint(
    "model",
    __name__,
    template_folder="templates",
    static_folder="static",
    static_url_path="/model/static",
    url_prefix="/models",  # default; override via register_blueprint(url_prefix=...)
)

# Bundled demo data, used unless the host app points this elsewhere — see
# get_models_dir().
DEFAULT_MODELS_DIR = Path(__file__).parent / "models"

# The three mesh variants the viewer can switch between. Rename the keys/labels
# to whatever your pipeline actually calls these.
MODEL_VARIANTS = ["pointcloud", "coarse", "coarse_texture", "hi-res", "hi-res_texture"]

# Every viewable asset, keyed by the name used in the URL / JS.
#   filename:    what we look for on disk
#   format:      "points" -> JSON {"points": [[x,y,z],...], "colors": [[r,g,b],...]?}
#                "ply"    -> untextured PLY mesh (vertex color at most), PLYLoader
#                "glb"    -> textured mesh, geometry+materials+images all in one
#                            binary glTF file, GLTFLoader
#   generatable: whether the "Generate" button applies
#
# PLY has no slot for a texture image, and non-embedded glTF (.gltf + .bin +
# .png) needs the loader to fetch those side files relative to the .gltf URL,
# which this single-file-per-asset API doesn't support — hence GLB for
# anything that needs an actual texture. If your mesher only outputs
# OBJ+MTL+JPG or split glTF, convert once with `gltf-pipeline -i in.gltf -o
# out.glb` (or export GLB directly if your tool supports it).
SCENE_FILE = "scene.mvs"
SCENE_DENSE_FILE = "scene_dense.mvs"
COARSE_MESH_FILE = "coarse_mesh.ply"
COARSE_TEXTURE_FILE = "coarse_full.glb"
HIRES_MESH_FILE = "hires_mesh.ply"
HIRES_TEXTURE_FILE = "hires_full.glb"

ASSET_DEFS = {
    "pointcloud": {"filename": "pointcloud.json", "label": "Point cloud", "generatable": True, "format": "points"},
    "coarse": {"filename": COARSE_MESH_FILE, "label": "Coarse shape", "generatable": True, "format": "ply"},
    "coarse_texture": {"filename": COARSE_TEXTURE_FILE, "label": "Coarse photo", "generatable": True, "format": "glb"},
    "hi-res": {"filename": HIRES_MESH_FILE, "label": "HiRes shape", "generatable": True, "format": "ply"},
    "hi-res_texture": {"filename": HIRES_TEXTURE_FILE, "label": "HiRes photo", "generatable": True, "format": "glb"},
    "track": {"filename": "track.json", "label": "Track", "generatable": False, "format": "points"},
    "survey": {"filename": "survey.json", "label": "Survey", "generatable": True, "format": "points"},
}

RASPBIAN_PATH = Path(__file__).resolve().parents[2] / "raspbian"
MAKE_INFO_SCRIPT = RASPBIAN_PATH / "make_info.py"
MERGE_SCRIPT = RASPBIAN_PATH / "merge_maps.py"

def get_models_dir() -> Path:
    """
    Where model folders live on disk. Defaults to the bundled demo folder;
    the host app can point this at real data by setting, before or after
    registering the blueprint:

        app.config["CYCLOPS_MODELS_DIR"] = "/data/cyclops-models"

    Read from current_app.config at request time (not import time), so one
    Flask app could even register this blueprint twice against different
    directories if that's ever useful.
    """
    configured = current_app.config.get("CYCLOPS_MODELS_DIR")
    return Path(configured) if configured else DEFAULT_MODELS_DIR


def model_dir(model_id: str) -> Path:
    d = get_models_dir() / model_id
    if not d.is_dir():
        abort(404, f"No such model: {model_id}")
    return d


def load_info(model_id: str) -> dict:
    d = model_dir(model_id)
    info_path = d / "info.json"
    if info_path.exists():
        info = json.loads(info_path.read_text())
    else:
        info = {"name": model_id, "keyframes": None, "points": None}
    info["model_id"] = model_id
    return info


def asset_status(model_id: str) -> dict:
    """Which of ASSET_DEFS actually exist on disk for this model."""
    d = model_dir(model_id)
    status = {}
    for key, meta in ASSET_DEFS.items():
        status[key] = (d / meta["filename"]).exists()
    return status


# --- Background job infrastructure ------------------------------------------
#
# Generation and merging can take a while, and your real pipeline tools
# print useful progress to stdout (TextureMesh, DensifyPointCloud, your
# ORB-SLAM3 export, etc). This runs that work on a background thread and
# captures its output line by line so the frontend can poll for it and show
# a live log, instead of the browser just hanging on one long POST.
#
# JOBS is keyed by (model_id, job_key): job_key is an asset key ("sparse",
# "track", ...) for generation, or the literal string "merge" for the submap
# merge. In-memory only — restart the server and job history is gone, and
# it's module-level state shared by every request regardless of how the
# blueprint is mounted. Fine for a single dev-server instance; behind
# multiple worker processes you'd want a shared store (DB row, Redis, etc.)
# instead.
JOBS = {}
JOBS_LOCK = threading.Lock()


def get_job(model_id, job_key):
    key = (model_id, job_key)
    with JOBS_LOCK:
        if key not in JOBS:
            JOBS[key] = {"state": "idle", "log": [], "lock": threading.Lock()}
        return JOBS[key]


def job_log(job, line):
    with job["lock"]:
        job["log"].append(line)


def job_snapshot(job):
    with job["lock"]:
        return {"state": job["state"], "log": "\n".join(job["log"])}


def run_subprocess_logged(job, cmd, cwd=None, check=True, outputs=()):
    """
    Real-pipeline helper: run `cmd`, streaming its combined stdout/stderr
    into the job log as each line is produced (not buffered until the
    process exits). Returns the exit code; if `check` is set, raises
    RuntimeError on a non-zero exit instead.

    `outputs` are files the command writes: they're deleted if it fails or is
    interrupted, so a partial file never looks up to date to is_up_to_date().
    """
    job_log(job, f"$ {' '.join(str(c) for c in cmd)}")
    try:
        process = subprocess.Popen(
            cmd, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1
        )
        for line in iter(process.stdout.readline, ""):
            job_log(job, line.rstrip())
        process.stdout.close()
        code = process.wait()
    except BaseException:
        remove_files(job, outputs)
        raise
    if code != 0:
        remove_files(job, outputs)
    if check and code != 0:
        # name the script rather than the interpreter for `python3 foo.py ...`
        name = next((Path(c).name for c in cmd if str(c).endswith(".py")), Path(cmd[0]).name)
        raise RuntimeError(f"{name} exited with code {code}")
    return code

def remove_files(job, paths):
    for path in paths:
        if path.exists():
            job_log(job, f"Removing partial output {path.name}")
            path.unlink()

def is_up_to_date(output: Path, *inputs: Path) -> bool:
    """True if output exists and is newer than all the inputs that exist"""
    if not output.exists():
        return False
    mtime = output.stat().st_mtime
    return all(x.stat().st_mtime <= mtime for x in inputs if x.exists())

def sparse_files(model_path: Path) -> list[Path]:
    return list((model_path / "sparse").glob("*.txt"))

def make_mvs_scene(model_path: Path, job) -> Path:
    """Create an OpenMVS scene from the sparse COLMAP model, ready for densification and meshing"""
    scene = model_path / SCENE_FILE
    if is_up_to_date(scene, *sparse_files(model_path)):
        job_log(job, "OpenMVS scene already exists")
        return scene
    run_subprocess_logged(
        job,
        ["InterfaceCOLMAP", "-i", str(model_path), "--image-folder", str(model_path / "images"),
         "-o", str(scene)],
        model_path,
        outputs=[scene]
    )
    return scene

def make_dense_scene(model_path: Path, job) -> Path:
    """Densify the OpenMVS scene"""
    scene = make_mvs_scene(model_path, job)
    dense = model_path / SCENE_DENSE_FILE
    if is_up_to_date(dense, scene):
        job_log(job, "Dense OpenMVS scene already exists")
        return dense
    run_subprocess_logged(
        job,
        ["DensifyPointCloud", "-i", str(scene), "-o", str(dense)],
        model_path,
        outputs=[dense]
    )
    return dense

def make_scene(model_path: Path, job, hires: bool) -> Path:
    """Make the scene a mesh is built from: dense for hires, sparse for coarse"""
    return make_dense_scene(model_path, job) if hires else make_mvs_scene(model_path, job)

def make_mesh(model_path: Path, job, hires: bool) -> Path:
    """ Create a coarse or high resolution mesh from the sparse COLMAP model
    """
    scene = make_scene(model_path, job, hires)
    mesh = model_path / (HIRES_MESH_FILE if hires else COARSE_MESH_FILE)
    if is_up_to_date(mesh, scene):
        job_log(job, f"{'High resolution' if hires else 'Coarse'} mesh already exists")
        return mesh
    run_subprocess_logged(
        job,
        ["ReconstructMesh", "-i", str(scene), "-o", str(mesh)],
        model_path,
        outputs=[mesh]
    )
    return mesh

def make_texture(model_path: Path, job, hires: bool) -> Path:
    """ Create a coarse or high resolution textured mesh from the sparse COLMAP model
    """
    mesh = make_mesh(model_path, job, hires) # first make the mesh (and its scene)
    scene = model_path / (SCENE_DENSE_FILE if hires else SCENE_FILE)
    textured = model_path / (HIRES_TEXTURE_FILE if hires else COARSE_TEXTURE_FILE)
    if is_up_to_date(textured, mesh):
        job_log(job, f"{'High resolution' if hires else 'Coarse'} textured mesh already exists")
        return textured
    temp = model_path / "temp.glb"
    try:
        run_subprocess_logged(
            job,
            ["TextureMesh", "-i", str(scene),
             "-m", str(mesh),
             "-o", str(temp),
             "--export-type", "glb"],
            model_path
        )
        run_subprocess_logged(
            job,
            ["gltf-pipeline", "-i", str(temp), "-o", str(textured)],
            model_path,
            outputs=[textured]
        )
    finally:
        # remove temporary files, whether or not texturing succeeded
        temp.unlink(missing_ok=True)
        for f in model_path.glob("temp*.png"):
            f.unlink()
    return textured

# asset key -> (builder, hires)
MESH_BUILDERS = {
    "coarse": (make_mesh, False),
    "coarse_texture": (make_texture, False),
    "hi-res": (make_mesh, True),
    "hi-res_texture": (make_texture, True),
}


def refresh_info(model_path: Path, job):
    """Re-run make_info.py on the atlas: rewrites info.json, pointcloud.json, track.json and survey.json"""
    run_subprocess_logged(job, ["python3", str(MAKE_INFO_SCRIPT), "-d", str(model_path)])


def run_refresh_job(model_id, job):
    refresh_info(model_dir(model_id), job)
    job_log(job, "Done.")


def run_generation_job(model_id, asset_key, job):
    """
    TODO: replace the demo simulation below with real calls into your
    pipeline. Use run_subprocess_logged() for anything that's a CLI tool
    (TextureMesh, obj2gltf, your ORB-SLAM3 export binary, ...), and
    job_log(job, "...") directly to report progress from pure-Python steps
    (e.g. writing track.json/survey.json yourself). Whatever you write to
    disk should end up at model_dir(model_id) / ASSET_DEFS[asset_key]["filename"].
    """
    d = model_dir(model_id)
    fmt = ASSET_DEFS[asset_key]["format"]
    filename = ASSET_DEFS[asset_key]["filename"]

    if fmt == "points":
        refresh_info(d, job)
    elif fmt in ("ply", "glb"):
        with mounted_images(d, job):
            ensure_sparse(d, job)
            if asset_key not in MESH_BUILDERS:
                raise NotImplementedError(f"Unknown mesh asset key: {asset_key}")
            builder, hires = MESH_BUILDERS[asset_key]
            builder(d, job, hires)
    else:  # glb
        raise NotImplementedError(
            f"Unknown asset format {fmt} for {asset_key} — add it to run_generation_job()"
        )

    job_log(job, "Done.")


def swap_in_merged_atlas(model_path: Path, job):
    """Keep the old atlas as atlas_pre_merge.osa and promote atlas_final.osa to atlas.osa"""
    merged = model_path / "atlas_merged.osa"
    if not merged.exists():
        raise RuntimeError("Merge finished but atlas_merged.osa was not created")
    atlas = model_path / "atlas.osa"
    pre_merge = model_path / "atlas_pre_merge.osa"
    if pre_merge.exists():
        # already merged before - keep the original atlas, discard the previous merge result
        job_log(job, "Keeping existing atlas_pre_merge.osa")
    elif atlas.exists():
        atlas.replace(pre_merge)
        job_log(job, "Renamed atlas.osa -> atlas_pre_merge.osa")
    merged.replace(atlas)
    job_log(job, "Renamed atlas_merged.osa -> atlas.osa")


def run_merge_job(model_id, job):
    model_path = model_dir(model_id)
    run_subprocess_logged(job, ["python3", str(MERGE_SCRIPT), "-d", str(model_path)])

    swap_in_merged_atlas(model_path, job)
    refresh_info(model_path, job)
    job_log(job, "Merge complete.")


def start_job(model_id, job_key, target):
    """
    Starts `target(job)` on a background thread if job_key isn't already
    running for this model. Returns (job, started) — started is False if a
    job was already in progress, so the caller can report "already running"
    instead of stomping on it.

    Captures the real Flask app object (current_app._get_current_object())
    while still inside the request that triggered this, and pushes an app
    context inside the thread — get_models_dir() reads current_app.config,
    which doesn't exist on a bare background thread otherwise, and fails
    with "Working outside of application context" the moment the job tries
    to touch the filesystem.
    """
    app = current_app._get_current_object()
    job = get_job(model_id, job_key)
    with job["lock"]:
        if job["state"] == "running":
            return job, False
        job["state"] = "running"
        job["log"] = []

    def runner():
        with app.app_context():
            try:
                target(job)
                with job["lock"]:
                    job["state"] = "done"
            except Exception as exc:
                job_log(job, f"ERROR: {exc}")
                with job["lock"]:
                    job["state"] = "error"

    threading.Thread(target=runner, daemon=True).start()
    return job, True


# --- Routes ------------------------------------------------------------------
# All relative to the blueprint's url_prefix (default "/models", see above).

@model_bp.route("/<model_id>")
def model_page(model_id):
    info = load_info(model_id)
    return render_template(
        "model/model.html",
        info=info,
        model_variants=[(key, ASSET_DEFS[key]['label']) for key in MODEL_VARIANTS],
        asset_status=asset_status(model_id),
        asset_formats={key: meta["format"] for key, meta in ASSET_DEFS.items()},
        has_survey_text=(model_dir(model_id) / "survey.svx").exists(),
    )


@model_bp.route("/api/<model_id>/info")
def api_info(model_id):
    return jsonify(load_info(model_id) | {"assets": asset_status(model_id)})


@model_bp.route("/api/<model_id>/asset/<asset_key>")
def api_asset(model_id, asset_key):
    """
    Serve an asset file straight from disk — a PLY or GLB for mesh assets,
    or a JSON {"points": [...], "colors": [...]} file for point-based
    assets. Flask/mimetypes picks the right Content-Type from the
    extension either way (see mimetypes.add_type for .glb above).
    """
    if asset_key not in ASSET_DEFS:
        abort(404, f"Unknown asset: {asset_key}")
    d = model_dir(model_id)
    filename = ASSET_DEFS[asset_key]["filename"]
    if not (d / filename).exists():
        abort(404, f"Asset not generated yet: {asset_key}")
    return send_from_directory(d, filename)


@model_bp.route("/api/<model_id>/survey", methods=["GET", "POST"])
def api_survey(model_id):
    d = model_dir(model_id)
    survey_path = d / "survey.svx"

    if request.method == "GET":
        text = survey_path.read_text() if survey_path.exists() else ""
        return jsonify({"text": text})

    # POST — either a pasted textbox value or an uploaded file, not both.
    if "file" in request.files and request.files["file"].filename:
        f = request.files["file"]
        text = f.read().decode("utf-8", errors="replace")
    else:
        text = request.form.get("text", "")

    if not text.strip():
        return jsonify({"error": "No survey text provided"}), 400

    survey_path.write_text(text)
    return jsonify({"ok": True, "length": len(text)})


@model_bp.route("/api/<model_id>/generate/<asset_key>", methods=["POST"])
def api_generate(model_id, asset_key):
    """Kick off generation of a missing asset as a background job."""
    if asset_key not in ASSET_DEFS:
        abort(404, f"Unknown asset: {asset_key}")
    if not ASSET_DEFS[asset_key]["generatable"]:
        abort(400, f"Asset '{asset_key}' is not generatable")

    d = model_dir(model_id)
    if asset_key == "survey" and not (d / "survey.svx").exists():
        return jsonify({"error": "Enter or upload a survey before generating it"}), 400

    job, started = start_job(model_id, asset_key, lambda job: run_generation_job(model_id, asset_key, job))
    if not started:
        return jsonify({"error": f"'{asset_key}' generation is already running"}), 409
    return jsonify({"ok": True}), 202


@model_bp.route("/api/<model_id>/generate/<asset_key>/status")
def api_generate_status(model_id, asset_key):
    if asset_key not in ASSET_DEFS:
        abort(404, f"Unknown asset: {asset_key}")
    return jsonify(job_snapshot(get_job(model_id, asset_key)))


@model_bp.route("/api/<model_id>/merge", methods=["POST"])
def api_merge(model_id):
    """Kick off a submap merge as a background job."""
    info = load_info(model_id)
    if (info.get("submaps") or 1) <= 1:
        return jsonify({"error": "Nothing to merge"}), 400

    job, started = start_job(model_id, "merge", lambda job: run_merge_job(model_id, job))
    if not started:
        return jsonify({"error": "Merge is already running"}), 409
    return jsonify({"ok": True}), 202


@model_bp.route("/api/<model_id>/merge/status")
def api_merge_status(model_id):
    return jsonify(job_snapshot(get_job(model_id, "merge")))


@model_bp.route("/api/<model_id>/refresh", methods=["POST"])
def api_refresh(model_id):
    """Re-run make_info.py as a background job."""
    job, started = start_job(model_id, "refresh", lambda job: run_refresh_job(model_id, job))
    if not started:
        return jsonify({"error": "Refresh is already running"}), 409
    return jsonify({"ok": True}), 202


@model_bp.route("/api/<model_id>/refresh/status")
def api_refresh_status(model_id):
    return jsonify(job_snapshot(get_job(model_id, "refresh")))


@contextmanager
def mounted_images(target_path: Path, job):
    """Mount images.tar at target_path/images with ratarmount for the duration of the block"""
    mount_point = target_path / "images"
    if mount_point.is_dir() and any(mount_point.iterdir()):
        yield mount_point  # already extracted or mounted
        return
    if mount_point.is_symlink():  # stale link left by an earlier colmap.py run
        mount_point.unlink()
    mount_point.mkdir(exist_ok=True)
    try:
        run_subprocess_logged(job, ["ratarmount", str(target_path / "images.tar"), str(mount_point)])
    except RuntimeError:
        mount_point.rmdir()
        raise
    try:
        yield mount_point
    finally:
        # don't let an unmount failure mask an exception from the body
        run_subprocess_logged(job, ["ratarmount", "-u", str(mount_point)], check=False)
        mount_point.rmdir()


COLMAP_SCRIPT = Path(__file__).resolve().parents[2] / "raspbian" / "colmap.py"
SPARSE_FILES = [CAM_FILENAME, IMAGE_FILENAME, POINTS_FILENAME]


def ensure_sparse(target_path: Path, job):
    """Run colmap.py to create the sparse COLMAP files if any are missing"""
    sparse_dir = target_path / "sparse"
    if all((sparse_dir / f).exists() for f in SPARSE_FILES):
        job_log(job, "Sparse model already exists")
        return
    with mounted_images(target_path, job):
        run_subprocess_logged(
            job,
            [sys.executable, "-u", str(COLMAP_SCRIPT), "-s",
             "-d", str(target_path), "-a", str(target_path / "atlas.osa")],
            outputs=[sparse_dir / f for f in SPARSE_FILES],
        )