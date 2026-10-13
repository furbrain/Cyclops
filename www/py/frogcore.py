"""
FrogCore integration: Flask Blueprint plus API client.

Uploads Cyclops recordings to FrogCore as surveys and asks FrogCore to build them.
Register without a url prefix so the routes live at /api/frogcore/... and /api/survey/...:

    from frogcore import frogcore_bp, load_survey
    app.register_blueprint(frogcore_bp)
"""
import json
import os
import threading
from pathlib import Path

import requests
from flask import Blueprint, jsonify, request

from utils import ROOT_DIR

frogcore_bp = Blueprint("frogcore", __name__)

# FrogCore account details are entered via the web UI and kept here, readable only by us
FROGCORE_CONFIG = Path.home() / ".config" / "cyclops" / "frogcore.json"
FROGCORE_URL = "https://frog.shetlandattackpony.co.uk"
FROGCORE_FILES = ["images.tar", "atlas.osa"]
SURVEY_FILE = "frogcore.json"


class FrogCoreClient:
    def __init__(self, username: str, password: str):
        self.http = requests.Session()
        r = self.http.post(f"{FROGCORE_URL}/api/login", auth=(username, password), timeout=30)
        r.raise_for_status()
        self.http.headers["Authorization"] = f"Bearer {r.json()['token']}"

    @staticmethod
    def load_config() -> dict:
        config = {"username": "", "password": ""}
        try:
            config.update(json.loads(FROGCORE_CONFIG.read_text()))
        except (IOError, ValueError):
            pass
        return config

    @staticmethod
    def save_config(config: dict):
        FROGCORE_CONFIG.parent.mkdir(parents=True, exist_ok=True)
        # create with owner-only permissions before writing the password
        fd = os.open(FROGCORE_CONFIG, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
        with os.fdopen(fd, "w") as f:
            json.dump(config, f)

    @classmethod
    def is_configured(cls) -> bool:
        config = cls.load_config()
        return bool(config["username"] and config["password"])

    @classmethod
    def from_config(cls) -> "FrogCoreClient":
        if not cls.is_configured():
            raise ValueError("FrogCore account not configured")
        config = cls.load_config()
        return cls(config["username"], config["password"])

    def create_survey(self, name: str) -> str:
        r = self.http.post(f"{FROGCORE_URL}/api/surveys", json={"name": name}, timeout=30)
        r.raise_for_status()
        return r.json()["id"]

    def upload(self, survey_id: str, path: Path) -> None:
        with path.open("rb") as fh:  # requests sets Content-Length from the file size
            r = self.http.put(f"{FROGCORE_URL}/api/surveys/{survey_id}/files/{path.name}",
                              data=fh, timeout=600)
        r.raise_for_status()

    def upload_recording(self, target_path: Path) -> None:
        """Create a survey for a recording and upload its files, recording progress alongside it"""
        survey_id = self.create_survey(target_path.name)
        save_survey(target_path, id=survey_id, status="uploading", message="")
        for i, fname in enumerate(FROGCORE_FILES):
            save_survey(target_path, message=f"Uploading {fname} ({i + 1}/{len(FROGCORE_FILES)})")
            self.upload(survey_id, target_path / fname)
        save_survey(target_path, status="uploaded", message=f"Uploaded {len(FROGCORE_FILES)} files")

    def build(self, survey_id: str) -> None:
        self.http.post(f"{FROGCORE_URL}/api/surveys/{survey_id}/build", timeout=30).raise_for_status()


@frogcore_bp.route("/api/frogcore/account", methods=["GET"])
def get_frogcore_account():
    config = FrogCoreClient.load_config()
    # never send the password back to the browser
    return jsonify(username=config["username"], has_password=bool(config["password"]))


@frogcore_bp.route("/api/frogcore/account", methods=["POST"])
def set_frogcore_account():
    data = request.get_json(silent=True) or {}
    config = FrogCoreClient.load_config()
    config["username"] = data.get("username", config["username"]).strip()
    if data.get("password"):  # blank means keep the existing password
        config["password"] = data["password"]
    try:
        FrogCoreClient(config["username"], config["password"])
    except Exception as e:
        return jsonify(success=False, message=f"Login failed: {e}"), 400
    FrogCoreClient.save_config(config)
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


def upload_survey(target_path: Path):
    try:
        FrogCoreClient.from_config().upload_recording(target_path)
    except Exception as e:
        save_survey(target_path, status="error", message=f"Upload failed: {e}")


@frogcore_bp.route("/api/survey/<target>/upload", methods=["POST"])
def survey_upload(target: str):
    target_path = ROOT_DIR / target
    if not target_path.is_dir():
        return jsonify(success=False, message="No such recording"), 404
    if not FrogCoreClient.is_configured():
        return jsonify(success=False, message="Set up your FrogCore account first"), 400
    if load_survey(target_path).get("status") == "uploading":
        return jsonify(success=False, message="Upload already in progress"), 409
    save_survey(target_path, id=None, status="uploading", message="Creating survey")
    threading.Thread(target=upload_survey, args=(target_path,), daemon=True).start()
    return jsonify(success=True)


@frogcore_bp.route("/api/survey/<target>/status")
def survey_status(target: str):
    return jsonify(load_survey(ROOT_DIR / target))


@frogcore_bp.route("/api/survey/<target>/build", methods=["POST"])
def survey_build(target: str):
    target_path = ROOT_DIR / target
    survey = load_survey(target_path)
    if survey.get("status") not in ("uploaded", "building") or not survey.get("id"):
        return jsonify(success=False, message="Survey has not been uploaded"), 409
    try:
        FrogCoreClient.from_config().build(survey["id"])
    except Exception as e:
        return jsonify(success=False, message=f"Build failed: {e}"), 502
    save_survey(target_path, status="building", message="Build requested")
    return jsonify(success=True)
