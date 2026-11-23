"""
Software updates: whether a new TacticalFrog release is available, and starting the update.

The work is done by raspbian/tacticalfrog_update.py: "check" here, as pi, and "install" as root in
tacticalfrog-update.service, which pi may start (a polkit rule from bootstrap.sh). Updates are only offered
with internet access and the battery charging or over 50%, which the updater checks again itself.

    from update import update_bp
    app.register_blueprint(update_bp)
"""
import json
import subprocess
import sys
import time
from pathlib import Path

from flask import Blueprint, jsonify, request

update_bp = Blueprint("update", __name__)

UPDATER = Path(__file__).resolve().parents[2] / "raspbian" / "tacticalfrog_update.py"
CHECK_EVERY = 5 * 60  # seconds: GitHub allows 60 unauthenticated requests an hour, and the battery changes
_cache = {"time": 0.0, "info": None}


def run_updater(*args, timeout=90) -> dict:
    result = subprocess.run([sys.executable, str(UPDATER), *args], capture_output=True, text=True,
                            timeout=timeout)
    if result.returncode != 0:
        raise RuntimeError(result.stderr.strip() or f"tacticalfrog_update.py {args[0]} failed")
    return json.loads(result.stdout)


def check(fresh=False) -> dict:
    if fresh or _cache["info"] is None or time.time() - _cache["time"] > CHECK_EVERY:
        _cache["info"] = run_updater("check")
        _cache["time"] = time.time()
    return _cache["info"]


def progress() -> dict:
    """The parts of the check that change during an update, without asking GitHub again"""
    status = None
    try:
        status = json.loads(Path("/var/lib/tacticalfrog-update/status.json").read_text())
    except (OSError, ValueError):
        pass
    state = subprocess.run(["systemctl", "show", "-P", "ActiveState", "tacticalfrog-update.service"],
                           capture_output=True, text=True).stdout.strip()
    return {"status": status, "running": state in ("activating", "reloading", "active")}


@update_bp.route("/api/update/status")
def api_status():
    """?refresh=1 checks GitHub now, ?progress=1 only returns status and running"""
    if request.args.get("progress"):
        return jsonify(progress())
    try:
        info = dict(check(fresh=bool(request.args.get("refresh"))))
    except (RuntimeError, ValueError, subprocess.TimeoutExpired) as e:
        return jsonify({"error": f"couldn't check for updates: {e}"}), 500
    info.update(progress())
    if info["running"]:
        info["allowed"] = False
        info["reason"] = "an update is running"
    return jsonify(info)


@update_bp.route("/api/update/start", methods=["POST"])
def api_start():
    try:
        info = check(fresh=True)  # the battery or the connection may have changed since the page asked
    except (RuntimeError, ValueError, subprocess.TimeoutExpired) as e:
        return jsonify({"success": False, "message": f"couldn't check for updates: {e}"}), 500
    if not info["allowed"]:
        return jsonify({"success": False, "message": f"Can't update: {info['reason']}"}), 409
    result = subprocess.run(["systemctl", "start", "--no-block", "tacticalfrog-update.service"],
                            capture_output=True, text=True)
    if result.returncode != 0:
        return jsonify({"success": False, "message": f"couldn't start the update: {result.stderr.strip()}"}), 500
    _cache["info"] = None  # what's installed is about to change
    return jsonify({"success": True, "message": "Update started"})
