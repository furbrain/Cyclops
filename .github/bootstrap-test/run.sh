#!/bin/bash
# End to end test of bootstrap.sh, for a throwaway arm64 machine (see .github/workflows/bootstrap-test.yml):
#   sudo .github/bootstrap-test/run.sh <folder with the three release tarballs>
# Boots stock Debian bookworm's cloud root filesystem as a systemd container, bootstraps it, reboots, installs
# the releases and checks that it all runs. It can't test the Pi's kernel, boot config or hardware.
# DON'T run it on a device: the container is privileged.
set -Eeuo pipefail

releases=$(realpath "${1:?folder with the release tarballs}")
repo=$(cd "$(dirname "$0")/../.." && pwd)
if grep -qsa 'Raspberry Pi' /proc/device-tree/model; then echo "not on a device" >&2; exit 1; fi

NAME=frog-bootstrap-test
IMAGE=https://cloud.debian.org/images/cloud/bookworm/latest
FILE=debian-12-genericcloud-arm64.tar.xz
work=$(mktemp -d)

step() { printf '\n######## %s\n' "$*"; }
inside() { docker exec "$NAME" "$@"; }
as_pi() { inside runuser -u pi -- env HOME=/home/pi "$@"; }
diagnose() {
  step "FAILED, diagnostics"
  inside systemctl --failed --no-pager || true
  inside journalctl --no-pager -n 250 || true
}
trap diagnose ERR

booted() {
  for _ in $(seq 1 60); do
    case $(inside systemctl is-system-running 2>/dev/null || true) in
      running | degraded)
        # docker mounts these over the system's own, which then can't be edited
        inside umount /etc/hosts /etc/hostname
        return 0 ;;
    esac
    sleep 3
  done
  echo "didn't boot" >&2
  return 1
}

reboot_it() {
  docker restart -t 60 "$NAME" >/dev/null
  booted
}

# retry <seconds> <command...>
retry() {
  local end=$(($(date +%s) + $1))
  shift
  until "$@" >/dev/null 2>&1; do
    if [ "$(date +%s)" -ge "$end" ]; then
      echo "still failing: $*" >&2
      return 1
    fi
    sleep 5
  done
  echo "ok: $*"
}

check_running() {
  local unit node
  # (beep_server writes to the Pi's registers, so it's kept off here)
  for unit in caddy gunicorn smbd multicast-loopback ros2_tactical_frog; do
    retry 180 inside systemctl is-active --quiet "$unit.service"
  done
  retry 120 inside curl -fsS -o /dev/null http://127.0.0.1/
  retry 60 inside curl -fsS -o /dev/null http://127.0.0.1/files/
  retry 60 inside curl -fsS -o /dev/null http://127.0.0.1:8080/
  retry 60 inside bash -c 'exec 3<>/dev/tcp/127.0.0.1/9090'
  retry 60 inside smbclient //127.0.0.1/trips -N -c ls
  for node in switcher rosbridge_websocket tf2_web_republisher; do
    retry 90 as_pi bash -c "/opt/TacticalFrog/current/raspbian/ros_env.sh \
      ros2 node list --no-daemon --spin-time 8 | grep -q $node"
  done
  retry 30 as_pi bash -c "nmcli general permissions | grep 'settings.modify.system' | grep -q yes"
}

step "Stock Debian bookworm"
curl -fsSL --retry 5 -o "$work/$FILE" "$IMAGE/$FILE"
curl -fsSL --retry 5 -o "$work/SUMS" "$IMAGE/SHA512SUMS"
(cd "$work" && grep " $FILE\$" SUMS | sha512sum -c -)
tar -C "$work" -xJf "$work/$FILE"
loop=$(losetup --find --show --partscan "$work/disk.raw")
udevadm settle
mkdir "$work/root"
mount "${loop}p1" "$work/root"
r=$work/root
# only what's needed to boot it as a container
: > "$r/etc/fstab"
touch "$r/etc/cloud/cloud-init.disabled"
for unit in systemd-networkd.service systemd-networkd.socket systemd-networkd-wait-online.service \
  systemd-resolved.service systemd-udevd.service systemd-udev-trigger.service getty@tty1.service apparmor.service; do
  ln -sfn /dev/null "$r/etc/systemd/system/$unit"
done
rm -f "$r/etc/resolv.conf"
touch "$r/etc/resolv.conf"
install -d "$r/etc/NetworkManager/conf.d" "$r/etc/systemd/system/beep_server.service.d"
printf '[keyfile]\nunmanaged-devices=interface-name:eth0\n' > "$r/etc/NetworkManager/conf.d/test-container.conf"
printf '[Unit]\nConditionVirtualization=no\n' > "$r/etc/systemd/system/beep_server.service.d/test.conf"
tar -C "$r" --numeric-owner -c . | docker import - "$NAME" >/dev/null
umount "$r"
losetup -d "$loop"
rm -rf "$work"

# not docker's usual subnet, as the docker inside uses that
docker network create --subnet 192.168.250.0/24 "$NAME" >/dev/null
docker run -d --name "$NAME" --network "$NAME" --privileged --cgroupns=private --hostname stock \
  --env container=docker --tmpfs /run --tmpfs /run/lock --volume "$NAME:/var/lib/docker" \
  --stop-signal SIGRTMIN+3 "$NAME" /sbin/init >/dev/null
booted
if inside id pi 2>/dev/null; then echo "not a stock system" >&2; exit 1; fi

step "Bootstrap"
docker cp "$repo/bootstrap.sh" "$NAME:/root/bootstrap.sh"
inside env SKIP_KALIBR="${SKIP_KALIBR:-0}" bash /root/bootstrap.sh

step "Reboot, with no releases yet"
reboot_it
retry 60 inside systemctl is-active --quiet tacticalfrog-await-releases.service

step "Install the releases"
for name in OpenMVS orb_slam3 TacticalFrog; do
  tarball=$(find "$releases" -name "$name-*.tar.gz" -print -quit)
  [ -n "$tarball" ] || { echo "no $name release" >&2; false; }
  base=$(basename "$tarball")
  docker cp "$tarball" "$NAME:/root/$base"
  inside tar -xzf "/root/$base" -C "/opt/$name" --no-same-owner
  inside ln -sfn "${base%-aarch64.tar.gz}" "/opt/$name/current"
done

step "It starts by itself"
check_running

step "The releases load"
as_pi /opt/TacticalFrog/current/raspbian/ros_env.sh python3 -c '
import orb_slam3_py
from orb_slam3.msg import Atlas
from tactical_frog.srv import SetMode
print("orb_slam3 and tactical_frog load")'
for script in colmap make_info make_json merge_maps combine; do
  as_pi /opt/TacticalFrog/current/raspbian/ros_env.sh \
    "/opt/TacticalFrog/current/modelling/$script.py" --help >/dev/null
  echo "ok: $script.py"
done
inside bash -c 'for f in /opt/OpenMVS/current/*; do
  if ldd "$f" 2>/dev/null | grep "not found"; then exit 1; fi
done'
if [ "${SKIP_KALIBR:-0}" != 1 ]; then
  # not piped straight into grep -q: that stops reading early, and pipefail then fails on docker's broken pipe
  kalibr=(rosrun kalibr kalibr_calibrate_imu_camera --help)
  status=0
  kalibr_help=$(as_pi docker run --rm furbrain/kalibr:latest "${kalibr[@]}" 2>&1) || status=$?
  # its --help always exits with 2: a bare "except:" round parse_args() turns argparse's exit(0) into exit(2)
  if [ "$status" != 0 ] && [ "$status" != 2 ]; then
    echo "kalibr failed:" >&2
    # again with unbuffered output and Python's traceback on a crash, to show where it got to
    as_pi docker run --rm -e PYTHONUNBUFFERED=1 -e PYTHONFAULTHANDLER=1 furbrain/kalibr:latest "${kalibr[@]}" >&2 ||
      echo "exit status $?" >&2
    false
  fi
  grep -q -- --max-iter <<< "$kalibr_help" || { echo "$kalibr_help" >&2; echo "kalibr has no --max-iter" >&2; false; }
  echo "ok: kalibr, as pi"
fi

step "It still runs after a reboot"
reboot_it
check_running

step "Bootstrap again"
inside env SKIP_KALIBR="${SKIP_KALIBR:-0}" bash /root/bootstrap.sh
reboot_it
check_running

step "It refuses to touch a development device"
inside touch /opt/TacticalFrog/current/.git
if inside bash /root/bootstrap.sh; then echo "didn't refuse" >&2; false; fi

step "What failed without the hardware, for information"
inside systemctl --failed --no-pager || true
inside bash -c "journalctl -b -u ros2_tactical_frog --no-pager | grep -iE 'process has died|error' | head -30" || true

step "PASSED"
