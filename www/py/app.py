import shutil

from flask import Flask, render_template
from pathlib import Path
from glob import glob
from collections import OrderedDict
from dataclasses import dataclass

WEB_ROOT = Path(__file__).parent.parent
MODEL_ROOT = Path("/data/trips")

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
        has_rough = (p / "map_1" / "rough.ply").exists()
        has_refined = (p / "map_1" / "refined.ply").exists()
        return cls(name, size, has_rough, has_refined)


def get_models():
   return [Model.fromPath(x) for x in MODEL_ROOT.iterdir()]


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
    src = MODEL_ROOT / src
    dest = MODEL_ROOT / dest
    try:
        src.rename(dest)
    except IOError:
        return {"success": False}
    return {"success": True}

@app.route("/api/delete/<target>")
def delete(target: str):
    target = MODEL_ROOT /target
    try:
        shutil.rmtree(target)
    except IOError:
        return {"success": False}
    return {"success": True}