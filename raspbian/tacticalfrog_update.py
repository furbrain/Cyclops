#!/usr/bin/env python3
"""Update TacticalFrog and the orb_slam3 and OpenMVS releases that go with it, from their GitHub releases.

    tacticalfrog_update.py check     what's installed, what's available and whether updating is allowed (JSON)
    tacticalfrog_update.py install   as root: install the latest TacticalFrog release and its partners, then
                                     restart the services. Run by tacticalfrog-update.service, which the
                                     web UI starts (see www/py/update.py)
    tacticalfrog_update.py battery   under ros_env.sh: the battery's state from the /battery topic (JSON)

Each TacticalFrog release has a RELEASES file (written by make_release.sh) with the orb_slam3 and OpenMVS
commits it was built and tested with, and their release tags, so the three are always updated together.
Everything is downloaded and unpacked next to the versions in use before any "current" link is switched,
and the previous versions are kept: to go back, point the three links at them again.
Updating is only allowed with internet access, and with the battery charging or over MIN_BATTERY percent.
Only the standard library is used, except by "battery".
"""
import fcntl
import hashlib
import json
import os
import platform
import re
import shutil
import subprocess
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path

OPT = Path("/opt")
# name in /opt and in the tarballs -> GitHub repository
REPOS = {
    "TacticalFrog": "furbrain/TacticalFrog",
    "orb_slam3": "furbrain/orb_slam3_ros",
    "OpenMVS": "furbrain/openMVS",
}
PARTNERS = ("orb_slam3", "OpenMVS")
MIN_BATTERY = 50  # percent, unless charging
CHARGING_AMPS = 0.02  # as www/js/battery.js: current above this (into the battery) is charging
# as the await-releases script that bootstrap.sh installs
SERVICES = ["multicast-loopback", "beep_server", "caddy", "smbd", "gunicorn", "ros2_tactical_frog"]
UNIT = "tacticalfrog-update.service"
STATUS = Path("/var/lib/tacticalfrog-update/status.json")  # the unit's StateDirectory, readable by pi
LOCK = Path("/run/lock/tacticalfrog-update.lock")
USER, HOME = "pi", "/home/pi"
API_LATEST = "https://api.github.com/repos/{repo}/releases/latest"
DOWNLOAD = "https://github.com/{repo}/releases/download/{tag}/{file}"
TIMEOUT = 20  # seconds, for each request


class UpdateError(Exception):
    pass


class Offline(UpdateError):
    pass


def version_name(tag: str) -> str:
    """The version in a release's file and folder names, as the release builds make it from the tag"""
    return re.sub(r"[^A-Za-z0-9._+-]", "-", tag)


def read_version(folder: Path):
    """(version, commit) from a release folder's VERSION file, or None"""
    try:
        version, commit = (folder / "VERSION").read_text().split()[:2]
    except (OSError, ValueError):
        return None
    return version, commit


def read_manifest(folder: Path) -> dict:
    """{partner: (commit, tag or None)} from a TacticalFrog release folder's RELEASES file"""
    manifest = {}
    try:
        lines = (folder / "RELEASES").read_text().splitlines()
    except OSError:
        return manifest
    for line in lines:
        fields = line.split()
        if len(fields) >= 3 and not line.startswith("#"):
            manifest[fields[0]] = (fields[1], None if fields[2] == "-" else fields[2])
    return manifest


def current(name: str) -> Path:
    return OPT / name / "current"


def is_development() -> bool:
    return (current("TacticalFrog") / ".git").exists()


def installed() -> dict:
    """What each "current" link points at"""
    result = {}
    for name in REPOS:
        link = current(name)
        version = read_version(link)
        result[name] = {
            "folder": os.readlink(link) if link.is_symlink() else None,
            "version": version[0] if version else None,
            "commit": version[1] if version else None,
        }
    return result


def mismatched(have: dict) -> list:
    """The partners that aren't the ones the installed TacticalFrog release was built with"""
    manifest = read_manifest(current("TacticalFrog"))
    return [name for name in PARTNERS if name in manifest and have[name]["commit"] != manifest[name][0]]


def http_get(url: str) -> bytes:
    request = urllib.request.Request(url, headers={
        "User-Agent": "TacticalFrog-updater", "Accept": "application/vnd.github+json"})
    try:
        with urllib.request.urlopen(request, timeout=TIMEOUT) as response:
            return response.read()
    except urllib.error.HTTPError:
        raise
    except (urllib.error.URLError, OSError) as e:  # no route, no DNS, timed out...
        raise Offline(f"no internet connection ({getattr(e, 'reason', e)})") from e


def latest_release(name: str):
    """The tag of a repository's latest release, None if it has none. Offline if GitHub can't be reached"""
    try:
        return json.loads(http_get(API_LATEST.format(repo=REPOS[name])))["tag_name"]
    except urllib.error.HTTPError as e:
        if e.code == 404:
            return None
        raise UpdateError(f"GitHub said {e.code} {e.reason} when asked for the latest {name} release") from e


def battery() -> dict | None:
    """{"percent", "charging"} from the /battery topic, or None if there's nothing on it"""
    command = [str(current("TacticalFrog") / "raspbian" / "ros_env.sh"), "/usr/bin/python3",
               str(Path(__file__).resolve()), "battery"]
    if os.geteuid() == 0:  # ROS runs as pi
        command = ["runuser", "-u", USER, "--", "env", "-i", f"HOME={HOME}", f"USER={USER}", "LANG=C.UTF-8",
                   "PATH=/usr/local/bin:/usr/bin:/bin"] + command
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=30, cwd="/")
        return json.loads(result.stdout.strip().splitlines()[-1])
    except (OSError, subprocess.TimeoutExpired, ValueError, IndexError):
        return None


def read_battery_topic() -> dict | None:
    import rclpy
    from sensor_msgs.msg import BatteryState

    rclpy.init()
    node = rclpy.create_node("tacticalfrog_update_battery")
    messages = []
    node.create_subscription(BatteryState, "/battery", messages.append, 5)
    end = time.monotonic() + 10  # discovery takes a few seconds, then it's published every second
    while not messages and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()
    if not messages:
        return None
    msg = messages[-1]
    percent = msg.percentage
    if percent != percent:  # NaN
        return None
    if percent <= 1.0:  # BatteryState's is 0-1, but allow for 0-100 as battery.js does
        percent *= 100
    return {"percent": round(max(0.0, min(100.0, percent))), "charging": msg.current > CHARGING_AMPS}


def battery_problem(state: dict | None) -> str | None:
    """Why the battery doesn't allow an update, or None if it does"""
    if state is None:
        return "the battery level isn't known"
    if not state["charging"] and state["percent"] <= MIN_BATTERY:
        return f"the battery is at {state['percent']}%: plug in the charger, or charge it to over {MIN_BATTERY}%"
    return None


def running() -> bool:
    result = subprocess.run(["systemctl", "is-active", "--quiet", UNIT], check=False)
    state = subprocess.run(["systemctl", "show", "-P", "ActiveState", UNIT],
                           capture_output=True, text=True, check=False).stdout.strip()
    return result.returncode == 0 or state in ("activating", "reloading")


def read_status() -> dict | None:
    try:
        return json.loads(STATUS.read_text())
    except (OSError, ValueError):
        return None


def check() -> dict:
    have = installed()
    info = {
        "installed": have,
        "development": is_development(),
        "repair": mismatched(have),
        "latest": None,
        "online": False,
        "update_available": False,
        "battery": None,
        "running": running(),
        "allowed": False,
        "reason": "",
        "status": read_status(),
    }
    if info["development"]:
        info["reason"] = "this is a development device (/opt/TacticalFrog/current is a git checkout)"
        return info
    try:
        info["latest"] = latest_release("TacticalFrog")
        info["online"] = True
    except Offline as e:
        info["reason"] = str(e)
        return info
    except UpdateError as e:
        info["online"] = True
        info["reason"] = str(e)
        return info
    if info["latest"] is None:
        info["reason"] = "no TacticalFrog release has been published"
        return info
    info["update_available"] = have["TacticalFrog"]["version"] != version_name(info["latest"])
    if not info["update_available"] and not info["repair"]:
        info["reason"] = "this is the latest release"
        return info
    info["battery"] = battery()
    problem = battery_problem(info["battery"])
    if info["running"]:
        info["reason"] = "an update is already running"
    elif problem:
        info["reason"] = problem
    else:
        info["allowed"] = True
    return info


# ----- installing, as root -----

def set_status(state: str, message: str, **extra):
    print(message, flush=True)
    STATUS.parent.mkdir(parents=True, exist_ok=True)
    data = {"state": state, "message": message, "time": time.time(), **extra}
    tmp = STATUS.with_suffix(".tmp")
    tmp.write_text(json.dumps(data))
    tmp.chmod(0o644)
    tmp.replace(STATUS)


def download(url: str, dest: Path):
    request = urllib.request.Request(url, headers={"User-Agent": "TacticalFrog-updater"})
    try:
        with urllib.request.urlopen(request, timeout=TIMEOUT) as response, open(dest, "wb") as f:
            shutil.copyfileobj(response, f, 1 << 20)
    except urllib.error.HTTPError as e:
        raise UpdateError(f"couldn't download {url}: {e.code} {e.reason}") from e
    except (urllib.error.URLError, OSError) as e:
        raise Offline(f"couldn't download {url}: {getattr(e, 'reason', e)}") from e


def fetch(name: str, tag: str, commit: str | None, progress) -> Path:
    """The folder in /opt/<name> for that release, downloading and unpacking it if it isn't there already"""
    version = version_name(tag)
    base = OPT / name
    folder = base / f"{name}-{version}"
    have = read_version(folder)
    if have and (commit is None or have[1] == commit):
        print(f"{folder} is already there")
        return folder
    tarball = f"{name}-{version}-{platform.machine()}.tar.gz"
    url = DOWNLOAD.format(repo=REPOS[name], tag=urllib.parse.quote(tag, safe=""), file=tarball)
    part = base / f".download-{tarball}"
    unpack = base / f".unpack-{version}"
    shutil.rmtree(unpack, ignore_errors=True)
    try:
        progress(f"Downloading {name} {tag}")
        download(url, part)
        download(url + ".sha256", part.with_name(part.name + ".sha256"))
        expected = part.with_name(part.name + ".sha256").read_text().split()[0]
        digest = hashlib.sha256()
        with open(part, "rb") as f:
            for block in iter(lambda: f.read(1 << 20), b""):
                digest.update(block)
        if digest.hexdigest() != expected:
            raise UpdateError(f"{tarball} is corrupt: its checksum is wrong")
        progress(f"Unpacking {name} {tag}")
        unpack.mkdir()
        subprocess.run(["tar", "-xzf", str(part), "-C", str(unpack), "--no-same-owner"], check=True)
        got = read_version(unpack / folder.name)
        if not got:
            raise UpdateError(f"{tarball} doesn't hold a {folder.name} release")
        if commit and got[1] != commit:
            raise UpdateError(f"{name} {tag} is commit {got[1][:10]}, but TacticalFrog needs {commit[:10]}")
        if folder.exists():  # an older build under the same name
            shutil.rmtree(folder)
        (unpack / folder.name).rename(folder)
        return folder
    finally:
        shutil.rmtree(unpack, ignore_errors=True)
        for f in (part, part.with_name(part.name + ".sha256")):
            f.unlink(missing_ok=True)


def switch(name: str, folder: Path):
    """Point /opt/<name>/current at folder, in one step"""
    tmp = OPT / name / ".current.new"
    tmp.unlink(missing_ok=True)
    os.symlink(folder.name, tmp)
    os.replace(tmp, current(name))


def prune(name: str, keep: set):
    """Remove releases other than the ones in keep, so that the disk doesn't fill up"""
    for folder in (OPT / name).glob(f"{name}-*"):
        if folder.is_dir() and not folder.is_symlink() and folder.name not in keep and read_version(folder):
            print(f"removing {folder}")
            shutil.rmtree(folder, ignore_errors=True)


def install():
    if os.geteuid() != 0:
        raise UpdateError("installing needs root: start tacticalfrog-update.service")
    if is_development():
        raise UpdateError("this is a development device (/opt/TacticalFrog/current is a git checkout)")
    progress = lambda message: set_status("running", message)
    progress("Checking for a new release")
    tag = latest_release("TacticalFrog")
    if tag is None:
        raise UpdateError("no TacticalFrog release has been published")
    problem = battery_problem(battery())
    if problem:
        raise UpdateError(f"not updating: {problem}")

    folders = {"TacticalFrog": fetch("TacticalFrog", tag, None, progress)}
    manifest = read_manifest(folders["TacticalFrog"])
    for name in PARTNERS:
        if name not in manifest:
            raise UpdateError(f"TacticalFrog {tag} doesn't say which {name} release goes with it")
        commit, partner_tag = manifest[name]
        if partner_tag is None:
            raise UpdateError(f"TacticalFrog {tag} was built with an {name} commit that has no release")
        folders[name] = fetch(name, partner_tag, commit, progress)

    problem = battery_problem(battery())  # downloading may have taken a while
    if problem:
        raise UpdateError(f"not updating: {problem}")
    before = installed()
    progress(f"Switching to TacticalFrog {tag}")
    for name in REPOS:
        switch(name, folders[name])
    subprocess.run(["systemctl", "daemon-reload"], check=True)
    subprocess.run(["systemctl", "--no-block", "restart"] + SERVICES, check=True)
    for name in REPOS:  # keep the new ones, and the ones they replaced to go back to
        prune(name, {folders[name].name, before[name]["folder"] or ""})
    set_status("done", f"Updated to TacticalFrog {tag}", version=version_name(tag),
               previous={name: before[name]["folder"] for name in REPOS})


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else ""
    if mode == "check":
        print(json.dumps(check(), indent=1))
    elif mode == "battery":
        print(json.dumps(read_battery_topic()))
    elif mode == "install":
        try:
            with open(LOCK, "w") as lock:
                try:
                    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError:
                    raise UpdateError("an update is already running")
                install()
        except (UpdateError, subprocess.CalledProcessError, OSError) as e:
            set_status("failed", f"Update failed: {e}")
            sys.exit(1)
    else:
        sys.exit(__doc__)


if __name__ == "__main__":
    main()
