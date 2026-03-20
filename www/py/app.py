import io
import json
import shutil
from typing import Sequence

from flask import Flask, render_template, Response, send_file
from pathlib import Path
from glob import glob
from collections import OrderedDict
from dataclasses import dataclass

from rosbags.rosbag2 import Reader
from zipstream import ZipStream

from utils import ROOT_DIR, BAG_NAME, load_ts_map, get_raw_image_from_timestamp

WEB_ROOT = Path(__file__).parent.parent


app = Flask(__name__, template_folder=WEB_ROOT / "templates")
app.jinja_env.trim_blocks = True
app.jinja_env.lstrip_blocks = True


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