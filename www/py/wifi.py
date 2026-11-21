#!/usr/bin/env python3
"""
TacticalFrog Wi-Fi Manager
----------------------
Small Flask app for scanning and connecting to Wi-Fi networks via
NetworkManager (nmcli). Targets Raspberry Pi OS Bookworm on the CM5.

This build assumes the UI is reached over a hotspot hosted on the SAME
physical radio that will be used to join the target network (single-radio,
no AP+STA concurrency). That means the moment we start joining a new
network, the browser's connection to this page drops. So /api/connect:
  1. Acks the request immediately, before touching the radio.
  2. Attempts the join + a reachability check in a background thread.
  3. If the join fails or is unreachable, re-activates whatever connection
     was active beforehand (falling back to the hotspot), so you aren't
     locked out with no way back in.
  4. Records the outcome so the browser can poll for it once it's back
     on a network that can reach this device (the restored hotspot on
     failure; on success you're on the real network — see README re: mDNS
     for finding the device afterwards).
"""

import re
import subprocess
import threading
import time

from flask import jsonify, render_template, request, Blueprint

wifi_bp= Blueprint("wifi", __name__, template_folder="templates")

NMCLI = "nmcli"

# --- EDIT THIS to match how your hotspot is actually brought up. ---------
CONFIG = {
    # Shared wlan interface used for both the hotspot and client connections.
    "wifi_device": "wlan0",
    # NetworkManager profile for the hotspot. Used as the fallback when a
    # connect attempt fails and nothing was active beforehand (or the previous
    # connection can't be brought back).
    "ap_connection": "AP",
    "connect_timeout": 25,        # seconds to wait for nmcli to join the network
    "gateway_check_timeout": 8,   # seconds to wait for a ping reply from the gateway
}
# ---------------------------------------------------------------------------

# Tracks the outcome of the most recent background connect attempt so the
# browser can poll for it after losing (and possibly regaining) the link.
connect_state = {"status": "idle", "ssid": None, "error": None,
                 "restored": None, "restore_error": None}
connect_state_lock = threading.Lock()


def set_connect_state(**kwargs):
    with connect_state_lock:
        connect_state.update(kwargs)


def get_connect_state():
    with connect_state_lock:
        return dict(connect_state)


def run_nmcli(args, timeout=15):
    """Run an nmcli command, returning (returncode, stdout, stderr)."""
    try:
        result = subprocess.run(
            [NMCLI] + args,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        return result.returncode, result.stdout.strip(), result.stderr.strip()
    except FileNotFoundError:
        return 127, "", "nmcli not found - is NetworkManager installed?"
    except subprocess.TimeoutExpired:
        return 124, "", "nmcli command timed out"


def get_active_connection():
    """Return info about the currently active Wi-Fi connection, or None."""
    rc, out, _ = run_nmcli(["-t", "-f", "DEVICE,TYPE,STATE,CONNECTION", "device", "status"])
    if rc != 0:
        return None
    for line in out.splitlines():
        parts = line.split(":")
        if len(parts) >= 4 and parts[1] == "wifi" and parts[2] == "connected":
            device, ssid = parts[0], parts[3]
            return {"device": device, "ssid": ssid, "ip": get_ip_for_device(device)}
    return None


def get_ip_for_device(device):
    rc, out, _ = run_nmcli(["-t", "-f", "IP4.ADDRESS", "device", "show", device])
    if rc != 0:
        return None
    for line in out.splitlines():
        if line.startswith("IP4.ADDRESS"):
            return line.split(":", 1)[1].split("/")[0]
    return None


def get_gateway_for_device(device):
    rc, out, _ = run_nmcli(["-t", "-f", "IP4.GATEWAY", "device", "show", device])
    if rc != 0:
        return None
    for line in out.splitlines():
        if line.startswith("IP4.GATEWAY"):
            gw = line.split(":", 1)[1].strip()
            return gw or None
    return None


def ping_ok(host, timeout):
    try:
        result = subprocess.run(
            ["ping", "-c", "2", "-W", str(timeout), host],
            capture_output=True, timeout=timeout + 3,
        )
        return result.returncode == 0
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return False


def restore_previous(previous):
    """
    After a failed/unreachable attempt, bring back the connection that was
    active before (so we're reachable the same way as before), falling back to
    the hotspot. Returns (restored_name, error) — restored_name is None if
    nothing could be brought up.
    """
    candidates = [c for c in (previous, CONFIG["ap_connection"]) if c]
    errors = []
    for name in dict.fromkeys(candidates):  # de-dupe, keep order
        rc, out, err = run_nmcli(["connection", "up", name], timeout=30)
        if rc == 0:
            return name, None
        errors.append(f"{name}: {err or out or 'failed'}")
    return None, "; ".join(errors)


def fail(ssid, previous, error):
    restored, restore_err = restore_previous(previous)
    set_connect_state(status="failed", ssid=ssid, error=error,
                      restored=restored, restore_error=restore_err)


def saved_profiles():
    """
    Map SSID -> {"uuid", "name"} for saved client (non-hotspot) Wi-Fi profiles.
    Matched on the profile's stored SSID, since profile names needn't match it
    (e.g. profile "Origami5" for SSID "origami5").
    """
    rc, out, _ = run_nmcli(["-t", "-f", "UUID,TYPE", "connection", "show"])
    if rc != 0:
        return {}
    profiles = {}
    for line in out.splitlines():
        uuid, _, ctype = line.partition(":")
        if ctype != "802-11-wireless":
            continue
        rc, out2, _ = run_nmcli(["-g", "connection.id,802-11-wireless.ssid,802-11-wireless.mode",
                                 "connection", "show", uuid])
        if rc != 0:
            continue
        fields = out2.splitlines()
        if len(fields) < 3 or fields[2] == "ap":
            continue  # skip the hotspot profile
        name, ssid = fields[0], fields[1]
        if ssid:
            profiles.setdefault(ssid, {"uuid": uuid, "name": name})
    return profiles


def background_connect(ssid, password):
    """Join a network off the request thread, verify it, revert on failure."""
    set_connect_state(status="connecting", ssid=ssid, error=None,
                      restored=None, restore_error=None)

    active = get_active_connection()
    previous = active["ssid"] if active else None  # NM connection profile name

    saved = saved_profiles().get(ssid)
    if saved and not password:
        # Already set up: bring up the saved profile with its stored secrets.
        args = ["connection", "up", saved["uuid"], "ifname", CONFIG["wifi_device"]]
    else:
        # New network, or a new password for a saved one (nmcli updates the
        # existing profile in that case).
        args = ["device", "wifi", "connect", ssid, "ifname", CONFIG["wifi_device"]]
        if password:
            args += ["password", password]

    rc, out, err = run_nmcli(args, timeout=CONFIG["connect_timeout"])

    if rc != 0:
        fail(ssid, previous, err or out or "Connection failed")
        return

    # Got nmcli success — now confirm the network is actually usable before
    # committing, since a wrong password can sometimes still yield an
    # associated-but-unusable link depending on driver/AP behaviour.
    device = CONFIG["wifi_device"]
    ip = get_ip_for_device(device)
    gateway = get_gateway_for_device(device)

    reachable = bool(ip) and (gateway is None or ping_ok(gateway, CONFIG["gateway_check_timeout"]))

    if not reachable:
        fail(ssid, previous, "Joined but not reachable (no IP or gateway unresponsive)")
        return

    set_connect_state(status="success", ssid=ssid, error=None)


def scan_networks(rescan=True):
    """Scan for Wi-Fi networks, returning a de-duplicated, signal-sorted list."""
    if rescan:
        run_nmcli(["device", "wifi", "rescan"], timeout=10)

    rc, out, err = run_nmcli(["-t", "-f", "SSID,SIGNAL,SECURITY,IN-USE", "device", "wifi", "list"])
    if rc != 0:
        return [], err

    saved = saved_profiles()
    networks = {}
    for line in out.splitlines():
        # nmcli terse output escapes literal ':' inside fields as '\:'
        fields = [f.replace("\\:", ":") for f in re.split(r"(?<!\\):", line)]
        if len(fields) < 4:
            continue
        ssid, signal, security, in_use = fields[0], fields[1], fields[2], fields[3]
        if not ssid:
            continue  # skip hidden/blank SSID entries
        try:
            signal = int(signal)
        except ValueError:
            signal = 0
        # Keep the strongest AP entry when the same SSID shows up more than once
        if ssid not in networks or signal > networks[ssid]["signal"]:
            networks[ssid] = {
                "ssid": ssid,
                "signal": signal,
                "security": security or "Open",
                "in_use": in_use == "*",
                "saved": ssid in saved,
            }
    return sorted(networks.values(), key=lambda n: n["signal"], reverse=True), None


@wifi_bp.route("/")
def index():
    return render_template("wifi.html")


@wifi_bp.route("/api/status")
def api_status():
    active = get_active_connection()
    return jsonify({"connected": active is not None, "connection": active})


@wifi_bp.route("/api/scan")
def api_scan():
    rescan = request.args.get("rescan", "1") != "0"
    networks, err = scan_networks(rescan=rescan)
    if err:
        return jsonify({"error": err}), 500
    return jsonify({"networks": networks})


@wifi_bp.route("/api/connect", methods=["POST"])
def api_connect():
    """
    Kicks off the join in a background thread and returns immediately.
    Because the hotspot and target network share one radio, this request's
    own connection will be dropped moments after this response is sent —
    that's expected. Poll /api/connect-status once you're back on a network
    that can reach this device to see how it went.
    """
    data = request.get_json(silent=True) or {}
    ssid = (data.get("ssid") or "").strip()
    password = data.get("password") or ""

    if not ssid:
        return jsonify({"success": False, "error": "SSID is required"}), 400

    if get_connect_state()["status"] == "connecting":
        return jsonify({"success": False, "error": "A connection attempt is already in progress"}), 409

    thread = threading.Thread(target=background_connect, args=(ssid, password), daemon=True)
    thread.start()

    return jsonify({
        "success": True,
        "status": "connecting",
        "message": "Attempting to join the network. The hotspot will drop shortly — "
                    "it will come back automatically if this fails.",
    })


@wifi_bp.route("/api/connect-status")
def api_connect_status():
    return jsonify(get_connect_state())


@wifi_bp.route("/api/disconnect", methods=["POST"])
def api_disconnect():
    data = request.get_json(silent=True) or {}
    device = data.get("device")
    if not device:
        active = get_active_connection()
        if not active:
            return jsonify({"success": False, "error": "No active connection"}), 400
        device = active["device"]

    rc, out, err = run_nmcli(["device", "disconnect", device])
    if rc == 0:
        return jsonify({"success": True})
    return jsonify({"success": False, "error": err or out}), 400


@wifi_bp.route("/api/forget", methods=["POST"])
def api_forget():
    """Delete the saved connection profile for an SSID."""
    data = request.get_json(silent=True) or {}
    ssid = (data.get("ssid") or "").strip()
    if not ssid:
        return jsonify({"success": False, "error": "SSID is required"}), 400
    saved = saved_profiles().get(ssid)
    if not saved:
        return jsonify({"success": False, "error": f"No saved profile for {ssid}"}), 404
    active = get_active_connection()
    if active and active["ssid"] == saved["name"]:
        # Deleting the live profile would drop the link with nothing to fall back to.
        return jsonify({"success": False,
                        "error": "Can't delete the network the device is currently using - "
                                 "connect to another network first."}), 409
    rc, out, err = run_nmcli(["connection", "delete", saved["uuid"]])
    if rc == 0:
        return jsonify({"success": True})
    return jsonify({"success": False, "error": err or out}), 400
