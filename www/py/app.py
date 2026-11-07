import io
import json
import tarfile
import time
import shutil
from typing import Sequence

from flask import Flask, render_template, Response, send_file, send_from_directory, abort, jsonify, request
from pathlib import Path
from glob import glob
from collections import OrderedDict
from dataclasses import dataclass

from zipstream import ZipStream

from utils import ROOT_DIR as DEFAULT_ROOT_DIR, BAG_NAME
from frogcore import frogcore_bp

WEB_ROOT = Path(__file__).parent.parent
from wifi import wifi_bp
from model import model_bp, image_sources

app = Flask(__name__, template_folder=WEB_ROOT / "templates")
app.register_blueprint(wifi_bp, url_prefix="/wifi")
app.register_blueprint(model_bp, url_prefix="/model")
app.register_blueprint(frogcore_bp)
app.jinja_env.trim_blocks = True
app.jinja_env.lstrip_blocks = True
app.config['CYCLOPS_MODELS_DIR'] = '/data/trips'
app.config['ROOT_DIR'] = DEFAULT_ROOT_DIR
# FLASK_ROOT_DIR=/some/dir overrides the default; app.config['ROOT_DIR'] can also be set directly
app.config.from_prefixed_env()
app.config['ROOT_DIR'] = Path(app.config['ROOT_DIR'])


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
    if p.is_symlink():  # e.g. a combined model's links to its sub-surveys - counted under their own names
        return 0
    if p.is_dir():
        return sum(sum_dirs(x) for x in p.iterdir())
    else:
        return p.stat().st_size

@dataclass
class Model:
    name: str
    size: int
    has_rough: bool
    has_refined: bool

    @classmethod
    def fromPath(cls, p: Path):
        name = p.stem
        size = sum_dirs(p)
        has_rough = (p / "map_1" / "rough.obj").exists()
        has_refined = (p / "map_1" / "refined.obj").exists()
        return cls(name, size, has_rough, has_refined)


def get_models():
   return [Model.fromPath(x) for x in sorted(app.config['ROOT_DIR'].iterdir())]


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
    src = app.config['ROOT_DIR'] / src
    dest = app.config['ROOT_DIR'] / dest
    try:
        src.rename(dest)
    except IOError:
        return {"success": False}
    return {"success": True}

@app.route("/api/delete/<target>")
def delete(target: str):
    target = app.config['ROOT_DIR'] / target
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



@app.route("/download/<target>/<payload>")
def downloads(target: str, payload: str):
    if payload == "bag":
        target_path = app.config['ROOT_DIR'] / target / BAG_NAME
        return zip_response([target_path], f"{target}_bag")
    target_path = app.config['ROOT_DIR'] / target / "map_1"
    files = target_path.glob(f"{payload}*.*")
    return zip_response(files, f"{target}_{payload}")

@app.route("/viewer/<target>/<style>")
def ply_viewer(target: str, style: str):
    return render_template("ply_viewer.html", target=target, style=style)

def keyframe_images(target: str) -> list[tuple[Path, tarfile.TarInfo]]:
    """(images.tar, member) for each keyframe image of a model, in keyframe order. A combined model's
    images come from its sub-surveys, in order. Images in each tar are named {keyframe}.jpg"""
    images = []
    for src in image_sources(app.config['ROOT_DIR'] / target):
        tar_path = src / "images.tar"
        with tarfile.open(tar_path) as tar:
            members = [m for m in tar.getmembers() if m.isfile() and Path(m.name).stem.isdigit()]
        members.sort(key=lambda m: int(Path(m.name).stem))
        images.extend((tar_path, m) for m in members)
    return images

def read_member(tars: dict[Path, tarfile.TarFile], tar_path: Path, member: tarfile.TarInfo) -> bytes:
    if tar_path not in tars:
        tars[tar_path] = tarfile.open(tar_path)
    return tars[tar_path].extractfile(member).read()

@app.route("/viewer/<target>/image/<int:index>")
def image(target: str, index: int):
    images = keyframe_images(target)
    if not 0 <= index < len(images):
        abort(404)
    tar_path, member = images[index]
    with tarfile.open(tar_path) as tar:
        data = tar.extractfile(member).read()
    return send_file(io.BytesIO(data), "image/jpeg")

@app.route("/viewer/<target>/slideshow")
def slideshow(target: str):
    images = keyframe_images(target)
    def gen():
        tars = {}
        try:
            frames = [read_member(tars, tar_path, member) for tar_path, member in images]
        finally:
            for tar in tars.values():
                tar.close()
        while True:
            for data in frames:
                yield(b'--frame\r\n' +
                      b'Content-Type: image/jpeg\r\n' +
                      f'Content-Length: {len(data)}\r\n\r\n'.encode() +
                      data +
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

