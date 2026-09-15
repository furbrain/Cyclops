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

import json
import mimetypes
import random
import subprocess
import threading
import time
from pathlib import Path

from flask import Blueprint, current_app, render_template, request, jsonify, send_from_directory, abort

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
ASSET_DEFS = {
    "pointcloud": {"filename": "pointcloud.json", "label": "Point cloud", "generatable": False, "format": "points"},
    "coarse": {"filename": "coarse_mesh.ply", "label": "Coarse shape", "generatable": True, "format": "ply"},
    "coarse_texture": {"filename": "coarse_full.glb", "label": "Coarse photo", "generatable": True, "format": "glb"},
    "hi-res": {"filename": "hires_mesh.ply", "label": "HiRes shape", "generatable": True, "format": "ply"},
    "hi-res_texture": {"filename": "hires_full.glb", "label": "HiRes photo", "generatable": True, "format": "glb"},
    "track": {"filename": "track.json", "label": "Track", "generatable": False, "format": "points"},
    "survey": {"filename": "survey.json", "label": "Survey", "generatable": True, "format": "points"},
}



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


def run_subprocess_logged(job, cmd, cwd=None):
    """
    Real-pipeline helper: run `cmd`, streaming its combined stdout/stderr
    into the job log as each line is produced (not buffered until the
    process exits). Returns the exit code — raise if it's non-zero.

    Example, once you're ready to wire up a real step:

        code = run_subprocess_logged(
            job,
            ["TextureMesh", "-i", str(mvs_path), "--export-type", "obj"],
            cwd=str(model_dir(model_id)),
        )
        if code != 0:
            raise RuntimeError(f"TextureMesh exited with code {code}")
    """
    job_log(job, f"$ {' '.join(str(c) for c in cmd)}")
    process = subprocess.Popen(
        cmd, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1
    )
    for line in iter(process.stdout.readline, ""):
        job_log(job, line.rstrip())
    process.stdout.close()
    return process.wait()


def write_demo_points(path):
    """Placeholder generator for 'points'-format assets — swap for real output."""
    points = [[round(random.uniform(-1, 1), 4) for _ in range(3)] for _ in range(400)]
    path.write_text(json.dumps({"points": points}))


def write_demo_ply(path):
    """Placeholder generator for 'ply'-format assets — a tiny octahedron with real faces."""
    verts = [(0, 0, 1), (1, 0, 0), (0, 1, 0), (-1, 0, 0), (0, -1, 0), (0, 0, -1)]
    faces = [(0, 1, 2), (0, 2, 3), (0, 3, 4), (0, 4, 1), (5, 2, 1), (5, 3, 2), (5, 4, 3), (5, 1, 4)]
    lines = [
        "ply", "format ascii 1.0", f"element vertex {len(verts)}",
        "property float x", "property float y", "property float z",
        f"element face {len(faces)}", "property list uchar int vertex_indices", "end_header",
    ]
    lines += [" ".join(str(c) for c in v) for v in verts]
    lines += [f"3 {' '.join(str(i) for i in f)}" for f in faces]
    path.write_text("\n".join(lines) + "\n")


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

    job_log(job, f"Starting generation of '{asset_key}'…")
    for step, pct in [("Loading inputs", 20), ("Processing", 60), ("Writing output", 90)]:
        time.sleep(1)
        job_log(job, f"[{pct:3d}%] {step}…")

    if fmt == "points":
        write_demo_points(d / filename)
    elif fmt == "ply":
        write_demo_ply(d / filename)
    else:  # glb
        raise NotImplementedError(
            "GLB generation has no demo output — wire up your texturing step "
            "(TextureMesh -> obj2gltf, see README) in run_generation_job()."
        )

    job_log(job, "Done.")


def run_merge_job(model_id, job):
    """
    TODO: replace with your real submap merge — offline atlas map merging,
    ID remapping across ID spaces, RANSAC+Horn's method alignment, etc.
    """
    info = load_info(model_id)
    submaps = info.get("submaps") or 1
    job_log(job, f"Merging {submaps} submaps…")
    for i in range(1, submaps):
        time.sleep(1)
        job_log(job, f"Aligned submap {i + 1}/{submaps}")
    time.sleep(1)
    job_log(job, "Writing merged atlas…")

    info.pop("model_id", None)
    info["submaps"] = 1
    (model_dir(model_id) / "info.json").write_text(json.dumps(info, indent=2))
    job_log(job, "Merge complete — submaps reduced to 1.")


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