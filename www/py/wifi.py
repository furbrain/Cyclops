#!/usr/bin/env python3
"""
Cyclops Wi-Fi Manager
----------------------
Small Flask app for scanning and connecting to Wi-Fi networks via
NetworkManager (nmcli). Targets Raspberry Pi OS Bookworm on the CM5.

This build assumes the UI is reached over a hotspot hosted on the SAME
physical radio that will be used to join the target network (single-radio,
no AP+STA concurrency). That means the moment we start joining a new
network, the browser's connection to this page drops. So /api/connect:
  1. Acks the request immediately, before touching the radio.
  2. Attempts the join + a reachability check in a background thread.
  3. Auto-reverts to the hotspot if the join fails or is unreachable,
     so you can never get locked out with no way back in.
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
    # Command that re-establishes the hotspot. Examples:
    #   NetworkManager-hosted hotspot profile named "Hotspot":
    #     ["nmcli", "connection", "up", "Hotspot"]
    #   hostapd/dnsmasq run as systemd units:
    #     ["systemctl", "restart", "hostapd"]   (plus dnsmasq if it also drops)
    #   custom script:
    #     ["/usr/local/bin/start-ap.sh"]
    "ap_restore_cmd": ["nmcli", "connection", "up", "Hotspot"],
    "connect_timeout": 25,        # seconds to wait for nmcli to join the network
    "gateway_check_timeout": 8,   # seconds to wait for a ping reply from the gateway
}
# ---------------------------------------------------------------------------

# Tracks the outcome of the most recent background connect attempt so the
# browser can poll for it after losing (and possibly regaining) the link.
connect_state = {"status": "idle", "ssid": None, "error": None}
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


def restore_ap():
    """Bring the hotspot back up after a failed/unreachable connection attempt."""
    subprocess.run(CONFIG["ap_restore_cmd"], capture_output=True, timeout=30)


def background_connect(ssid, password):
    """Join a network off the request thread, verify it, revert on failure."""
    set_connect_state(status="connecting", ssid=ssid, error=None)

    args = ["device", "wifi", "connect", ssid, "ifname", CONFIG["wifi_device"]]
    if password:
        args += ["password", password]

    rc, out, err = run_nmcli(args, timeout=CONFIG["connect_timeout"])

    if rc != 0:
        restore_ap()
        set_connect_state(status="failed", ssid=ssid, error=err or out or "Connection failed")
        return

    # Got nmcli success — now confirm the network is actually usable before
    # committing, since a wrong password can sometimes still yield an
    # associated-but-unusable link depending on driver/AP behaviour.
    device = CONFIG["wifi_device"]
    ip = get_ip_for_device(device)
    gateway = get_gateway_for_device(device)

    reachable = bool(ip) and (gateway is None or ping_ok(gateway, CONFIG["gateway_check_timeout"]))

    if not reachable:
        restore_ap()
        set_connect_state(status="failed", ssid=ssid, error="Joined but not reachable (no IP or gateway unresponsive)")
        return

    set_connect_state(status="success", ssid=ssid, error=None)


def scan_networks(rescan=True):
    """Scan for Wi-Fi networks, returning a de-duplicated, signal-sorted list."""
    if rescan:
        run_nmcli(["device", "wifi", "rescan"], timeout=10)

    rc, out, err = run_nmcli(["-t", "-f", "SSID,SIGNAL,SECURITY,IN-USE", "device", "wifi", "list"])
    if rc != 0:
        return [], err

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
    """Delete a saved connection profile by SSID name."""
    data = request.get_json(silent=True) or {}
    ssid = (data.get("ssid") or "").strip()
    if not ssid:
        return jsonify({"success": False, "error": "SSID is required"}), 400
    rc, out, err = run_nmcli(["connection", "delete", ssid])
    if rc == 0:
        return jsonify({"success": True})
    return jsonify({"success": False, "error": err or out}), 400