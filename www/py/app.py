import io
import json
import os
import time
import shutil
import threading
from typing import Sequence

from flask import Flask, render_template, Response, send_file, send_from_directory, abort, jsonify, request, jsonify, request
from pathlib import Path
from glob import glob
from collections import OrderedDict
from dataclasses import dataclass

from rosbags.rosbag2 import Reader
from zipstream import ZipStream

from utils import ROOT_DIR, BAG_NAME, load_ts_map, get_raw_image_from_timestamp
from frogcore_client import FrogCoreClient

WEB_ROOT = Path(__file__).parent.parent
from wifi import wifi_bp
from model import model_bp

app = Flask(__name__, template_folder=WEB_ROOT / "templates")
app.register_blueprint(wifi_bp, url_prefix="/wifi")
app.register_blueprint(model_bp, url_prefix="/model")
app.jinja_env.trim_blocks = True
app.jinja_env.lstrip_blocks = True
app.config['CYCLOPS_MODELS_DIR'] = '/data/trips'


def name_from_fname(fname: Path):
    base_name = fname.stem
    return base_name.split("_", maxsplit=1)[-1].replace("_", " ")

def get_files(p: Path):
    entries = OrderedDict()
    for f in sorted(p.iterdir()):
        if f.is_dir():
            entries[name_from_fname(f)] = get_files(f)
        else:
            entries[name_from_fname(f)] = f.relative_to(app.template_folder)
    return entries

def sum_dirs(p: Path):
    if p.is_dir():
        return sum(sum_dirs(x) for x in p.iterdir())
    else:
        return p.stat().st_size

# FrogCore account details are entered via the web UI and kept here, readable only by us
FROGCORE_CONFIG = Path.home() / ".config" / "cyclops" / "frogcore.json"
FROGCORE_URL = "https://frog.shetlandattackpony.co.uk"
FROGCORE_FILES = ["images.tar", "atlas.osa"]
SURVEY_FILE = "frogcore.json"

def load_frogcore_config() -> dict:
    config = {"username": "", "password": ""}
    try:
        config.update(json.loads(FROGCORE_CONFIG.read_text()))
    except (IOError, ValueError):
        pass
    return config


def save_frogcore_config(config: dict):
    FROGCORE_CONFIG.parent.mkdir(parents=True, exist_ok=True)
    # create with owner-only permissions before writing the password
    fd = os.open(FROGCORE_CONFIG, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, "w") as f:
        json.dump(config, f)


def frogcore_client() -> FrogCoreClient:
    config = load_frogcore_config()
    if not (config["username"] and config["password"]):
        raise ValueError("FrogCore account not configured")
    return FrogCoreClient(FROGCORE_URL, config["username"], config["password"])


@app.route("/api/frogcore/account", methods=["GET"])
def get_frogcore_account():
    config = load_frogcore_config()
    # never send the password back to the browser
    return jsonify(username=config["username"], has_password=bool(config["password"]))


@app.route("/api/frogcore/account", methods=["POST"])
def set_frogcore_account():
    data = request.get_json(silent=True) or {}
    config = load_frogcore_config()
    config["username"] = data.get("username", config["username"]).strip()
    if data.get("password"):  # blank means keep the existing password
        config["password"] = data["password"]
    try:
        FrogCoreClient(FROGCORE_URL, config["username"], config["password"])
    except Exception as e:
        return jsonify(success=False, message=f"Login failed: {e}"), 400
    save_frogcore_config(config)
    return jsonify(success=True)


def load_survey(p: Path) -> dict:
    try:
        return json.loads((p / SURVEY_FILE).read_text())
    except (IOError, ValueError):
        return {}


def save_survey(p: Path, **kwargs):
    survey = load_survey(p)
    survey.update(kwargs)
    (p / SURVEY_FILE).write_text(json.dumps(survey))
    return survey


@dataclass
class Model:
    name: str
    size: int
    has_rough: bool
    has_refined: bool
    survey: dict

    @classmethod
    def fromPath(cls, p: Path):
        name = p.stem
        size = sum_dirs(p)
        has_rough = (p / "map_1" / "rough.obj").exists()
        has_refined = (p / "map_1" / "refined.obj").exists()
        return cls(name, size, has_rough, has_refined, load_survey(p))


def get_models():
   return [Model.fromPath(x) for x in sorted(ROOT_DIR.iterdir())]


@app.route("/")
def hello_world():
    js_dir = WEB_ROOT / "js" / "index"
    entries = get_files(Path(app.template_folder)/"main")
    js_files = [x.relative_to(WEB_ROOT) for x in sorted(js_dir.iterdir())]
    models = get_models()
    return render_template("index.html", entries=entries, js_files=js_files, models=models)

@app.route("/api/getRecordings")
def api_link():
    models = get_models()
    return render_template("recordings_fragment.html", models=models)

@app.route("/api/rename/<src>/<dest>")
def rename(src: str, dest: str):
    src = ROOT_DIR / src
    dest = ROOT_DIR / dest
    try:
        src.rename(dest)
    except IOError:
        return {"success": False}
    return {"success": True}

@app.route("/api/delete/<target>")
def delete(target: str):
    target = ROOT_DIR / target
    try:
        shutil.rmtree(target)
    except IOError:
        return {"success": False}
    return {"success": True}

def zip_response(paths: Sequence[Path], name: str):
    zs = ZipStream(sized=True)
    for p in paths:
        zs.add_path(p)
    return Response(
        zs,
        mimetype="application/zip",
        headers={
            "Content-Disposition": f"attachment; filename={name}.zip",
            "Content-Length": len(zs),
            "Last-Modified": zs.last_modified,
        }
    )



def upload_survey(target_path: Path):
    try:
        client = frogcore_client()
        survey_id = client.create_survey(target_path.name)
        save_survey(target_path, id=survey_id, status="uploading", message="")
        for i, fname in enumerate(FROGCORE_FILES):
            f = target_path / fname
            save_survey(target_path, message=f"Uploading {fname} ({i + 1}/{len(FROGCORE_FILES)})")
            client.upload(survey_id, f)
        save_survey(target_path, status="uploaded", message=f"Uploaded {len(FROGCORE_FILES)} files")
    except Exception as e:
        save_survey(target_path, status="error", message=f"Upload failed: {e}")


@app.route("/api/survey/<target>/upload", methods=["POST"])
def survey_upload(target: str):
    target_path = ROOT_DIR / target
    if not target_path.is_dir():
        return jsonify(success=False, message="No such recording"), 404
    config = load_frogcore_config()
    if not (config["username"] and config["password"]):
        return jsonify(success=False, message="Set up your FrogCore account first"), 400
    if load_survey(target_path).get("status") == "uploading":
        return jsonify(success=False, message="Upload already in progress"), 409
    save_survey(target_path, id=None, status="uploading", message="Creating survey")
    threading.Thread(target=upload_survey, args=(target_path,), daemon=True).start()
    return jsonify(success=True)


@app.route("/api/survey/<target>/status")
def survey_status(target: str):
    return jsonify(load_survey(ROOT_DIR / target))


@app.route("/api/survey/<target>/build", methods=["POST"])
def survey_build(target: str):
    target_path = ROOT_DIR / target
    survey = load_survey(target_path)
    if survey.get("status") not in ("uploaded", "building") or not survey.get("id"):
        return jsonify(success=False, message="Survey has not been uploaded"), 409
    try:
        client = frogcore_client()
        client.build(survey["id"])
    except Exception as e:
        return jsonify(success=False, message=f"Build failed: {e}"), 502
    save_survey(target_path, status="building", message="Build requested")
    return jsonify(success=True)


@app.route("/download/<target>/<payload>")
def downloads(target: str, payload: str):
    if payload == "bag":
        target_path = ROOT_DIR / target / BAG_NAME
        return zip_response([target_path], f"{target}_bag")
    target_path = ROOT_DIR / target / "map_1"
    files = target_path.glob(f"{payload}*.*")
    return zip_response(files, f"{target}_{payload}")

@app.route("/viewer/<target>/<style>")
def ply_viewer(target: str, style: str):
    return render_template("ply_viewer.html", target=target, style=style)

@app.route("/viewer/<target>/image/<int:index>")
def image(target: str, index: int):
    bag = ROOT_DIR / target / BAG_NAME
    frame_dict = load_ts_map(target)
    frame_ts = sorted(frame_dict.keys())[index]
    with Reader(bag) as r:
        msg = get_raw_image_from_timestamp(r, frame_dict, frame_ts)
    buffer = io.BytesIO(msg.data)
    return send_file(buffer,"image/jpeg")

@app.route("/viewer/<target>/slideshow")
def slideshow(target: str):
    bag = ROOT_DIR / target / BAG_NAME
    frame_dict = load_ts_map(target)
    frame_ts = sorted(frame_dict.keys())
    def gen():
        with Reader(bag) as r:
            msgs = [get_raw_image_from_timestamp(r, frame_dict, ts) for ts in frame_ts]

        while True:
            for msg in msgs:
                data = msg.data.tobytes()
                yield(b'--frame\r\n' +
                      b'Content-Type: image/jpeg\r\n' +
                      f'Content-Length: {len(data)}\r\n\r\n'.encode() +
                      msg.data.tobytes() +
                      b'\r\n')
                time.sleep(0.1)
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame',
                    headers={'X-Accel-Buffering': 'no', 'Cache-Control': 'no-cache'})



if app.config['DEBUG']:
    @app.route('/js/<path:path>')
    def send_js(path):
        return send_from_directory(WEB_ROOT / "js", path)

    @app.route('/css/<path:path>')
    def send_css(path):
        return send_from_directory(WEB_ROOT / "css", path)

