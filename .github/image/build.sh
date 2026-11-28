#!/bin/bash
# Build a disk image for the CM5 (see .github/workflows/image.yml), for a throwaway arm64 machine:
#   sudo .github/image/build.sh <folder with the three release tarballs> <output folder>
# It's Raspberry Pi OS Lite (bookworm), set up by bootstrap.sh with the releases installed, shrunk to the
# smallest size that holds it. On its first boot Raspberry Pi OS grows it to fill the card or eMMC.
# Like the bootstrap test, it boots the OS as a systemd container to bootstrap it, as that needs systemd.
# DON'T run it on a device: the container is privileged.
#   WORK=/mnt       where to work: needs about 25GB
set -Eeuo pipefail

releases=$(realpath "${1:?folder with the release tarballs}")
out=$(realpath -m "${2:?output folder}")
repo=$(cd "$(dirname "$0")/../.." && pwd)
if grep -qsa 'Raspberry Pi' /proc/device-tree/model; then echo "not on a device" >&2; exit 1; fi

# Raspberry Pi OS Lite, the last bookworm one (bookworm is "oldstable" since trixie came out)
BASE_URL=https://downloads.raspberrypi.com/raspios_oldstable_lite_arm64/images/raspios_oldstable_lite_arm64-2026-10-06/2026-10-06-raspios-bookworm-arm64-lite.img.xz
BASE_SHA256=9e938c5a6981c3f498b3b564c527f3e1fd58ef20bbf703be0eea748384a6c1af
NAME=frog-image
# units that can't or mustn't run while it's a container, masked only for then
CONTAINER_MASKS=(
  systemd-udevd.service systemd-udevd-control.socket systemd-udevd-kernel.socket systemd-udev-trigger.service
  getty@tty1.service serial-getty@ttyAMA10.service userconfig.service regenerate_ssh_host_keys.service
  sshswitch.service rpi-eeprom-update.service dphys-swapfile.service ModemManager.service
  systemd-networkd-wait-online.service NetworkManager-wait-online.service
)

work=$(mktemp -d -p "${WORK:-/tmp}")
image=
mkdir -p "$out"
step() { printf '\n######## %s\n' "$*"; }
inside() { docker exec "$NAME" "$@"; }
cleanup() {
  set +e
  docker rm -f "$NAME" >/dev/null 2>&1
  for m in "$work"/new/boot/firmware "$work"/new "$work"/base/boot/firmware "$work"/base; do
    mountpoint -q "$m" && umount "$m"
  done
  for l in $(losetup -j "$work/base.img" -O NAME -n) $(losetup -j "$out/$image" -O NAME -n 2>/dev/null); do
    losetup -d "$l"
  done
  rm -rf "$work"
}
trap cleanup EXIT
diagnose() {
  step "FAILED, diagnostics"
  inside systemctl --failed --no-pager || true
  inside journalctl --no-pager -n 150 || true
}
trap diagnose ERR

booted() {
  for _ in $(seq 1 60); do
    case $(inside systemctl is-system-running 2>/dev/null || true) in
      running | degraded)
        inside umount /etc/hosts /etc/hostname # docker mounts these over the system's own
        return 0 ;;
    esac
    sleep 3
  done
  echo "didn't boot" >&2
  return 1
}

# e2fsprogs etc from Raspberry Pi OS itself, so the filesystem suits its kernel and its first boot resize
tools() { docker run --rm --privileged -v /dev:/dev -v "$work:$work" "$NAME-base" "$@"; }

partitions() { # partitions <image>: the loop device, with its partitions
  local loop
  loop=$(losetup --find --show --partscan "$1")
  udevadm settle
  echo "$loop"
}

step "Raspberry Pi OS"
curl -fsSL --retry 5 -o "$work/base.img.xz" "$BASE_URL"
echo "$BASE_SHA256  $work/base.img.xz" | sha256sum -c -
xz -dc "$work/base.img.xz" > "$work/base.img"
rm "$work/base.img.xz"
loop=$(partitions "$work/base.img")
mkdir "$work/base"
mount -o ro "${loop}p2" "$work/base"
mount -o ro "${loop}p1" "$work/base/boot/firmware"
tar -C "$work/base" --xattrs --xattrs-include='*' --numeric-owner -c . | docker import - "$NAME-base" >/dev/null
# the start of the partition table and the boot partition are kept as they are; the root filesystem is made anew
p2_start=$(sfdisk --json "$work/base.img" | python3 -c 'import json,sys; print(json.load(sys.stdin)["partitiontable"]["partitions"][1]["start"])')
cp "$work/base/etc/fstab" "$work/fstab"
cp "$work/base/etc/resolv.conf" "$work/resolv.conf"
umount "$work/base/boot/firmware" "$work/base"
losetup -d "$loop"

step "Boot it as a container"
{
  echo "FROM $NAME-base"
  # no fstab: the container's root is already mounted; eth0 is docker's, not NetworkManager's
  echo "RUN : > /etc/fstab && mkdir -p /etc/NetworkManager/conf.d && \\"
  echo "  printf '[keyfile]\\nunmanaged-devices=interface-name:eth0\\n' > /etc/NetworkManager/conf.d/image-build.conf && \\"
  # there's no root device to find the modules for, so take the usual ones (a kernel update builds an initramfs)
  echo "  mkdir -p /etc/initramfs-tools/conf.d && printf 'MODULES=most\\n' > /etc/initramfs-tools/conf.d/image-build.conf && \\"
  echo "  mkdir -p /etc/systemd/system/beep_server.service.d && \\"
  echo "  printf '[Unit]\\nConditionVirtualization=no\\n' > /etc/systemd/system/beep_server.service.d/image-build.conf && \\"
  printf '  for u in %s; do ln -sfn /dev/null /etc/systemd/system/$u; done\n' "${CONTAINER_MASKS[*]}"
} > "$work/Dockerfile"
docker build -q -t "$NAME-boot" "$work" >/dev/null
docker network create --subnet 192.168.251.0/24 "$NAME" >/dev/null 2>&1 || true
docker run -d --name "$NAME" --network "$NAME" --privileged --cgroupns=private --hostname raspberrypi \
  --env container=docker --tmpfs /run --tmpfs /run/lock \
  --stop-signal SIGRTMIN+3 "$NAME-boot" /sbin/init >/dev/null
booted
inside mkdir -p /boot/firmware
inside test -f /boot/firmware/config.txt

step "Bootstrap"
docker cp "$repo/bootstrap.sh" "$NAME:/root/bootstrap.sh"
inside env FROG_IMAGE=1 FROG_RELEASE=none bash /root/bootstrap.sh # the releases are installed below

step "Install the releases"
version=
for name in OpenMVS orb_slam3 TacticalFrog; do
  tarball=$(find "$releases" -name "$name-*-aarch64.tar.gz" -print -quit)
  [ -n "$tarball" ] || { echo "no $name release" >&2; false; }
  base=$(basename "$tarball" -aarch64.tar.gz)
  docker cp "$tarball" "$NAME:/root/release.tar.gz"
  inside tar -xzf /root/release.tar.gz -C "/opt/$name" --no-same-owner
  inside rm /root/release.tar.gz
  inside ln -sfn "$base" "/opt/$name/current"
  [ "$name" != TacticalFrog ] || version=${base#TacticalFrog-}
done
image=tacticalfrog-$version-cm5.img

step "Check the CM5 will boot the newest kernel"
inside bash -c '
  set -e
  kernel=$(ls /lib/modules | grep -- "-rpi-2712$" | sort -V | tail -n 1)
  echo "kernel $kernel"
  cmp "/boot/vmlinuz-$kernel" /boot/firmware/kernel_2712.img
  cmp "/boot/initrd.img-$kernel" /boot/firmware/initramfs_2712
  # the first boot resize is done in the initramfs
  lsinitramfs /boot/firmware/initramfs_2712 | grep -q "scripts/local-premount/firstboot"'

step "Check it starts"
# as tacticalfrog-await-releases does when the last release arrives
inside systemctl daemon-reload
inside systemctl --no-block restart multicast-loopback caddy smbd gunicorn ros2_tactical_frog
for unit in caddy gunicorn smbd ros2_tactical_frog; do
  for _ in $(seq 1 60); do inside systemctl is-active --quiet "$unit" && break; sleep 3; done
  inside systemctl is-active --quiet "$unit"
done
for _ in $(seq 1 40); do inside curl -fsS -o /dev/null http://127.0.0.1/ && break; sleep 3; done
inside curl -fsS -o /dev/null http://127.0.0.1/
echo "ok: the services run and the web UI answers"

step "Tidy up"
inside bash -c '
  set -e
  apt-get clean
  rm -rf /var/lib/apt/lists/* /var/cache/tacticalfrog-bootstrap /home/pi/ros2_ws/build /home/pi/ros2_ws/log
  journalctl --rotate >/dev/null 2>&1 || true
  rm -rf /var/log/journal/* /home/pi/.ros/log /root/.cache /home/pi/.cache /root/bootstrap.sh
  find /var/log -type f \( -name "*.gz" -o -name "*.[0-9]" \) -delete'
used_mb=$(inside du -sx -B1M --exclude=/boot/firmware / | cut -f1)
docker stop -t 120 "$NAME" >/dev/null
echo "root filesystem: ${used_mb}MB"

step "Write the image"
# room to unpack it into; it's shrunk to fit afterwards
root_mb=$((used_mb * 115 / 100 + 1024))
truncate -s 0 "$out/$image"
dd if="$work/base.img" of="$out/$image" bs=512 count="$p2_start" status=none
rm "$work/base.img"
truncate -s $((p2_start * 512 + root_mb * 1024 * 1024)) "$out/$image"
echo ", +" | sfdisk -q --no-reread -N 2 "$out/$image"
loop=$(partitions "$out/$image")
tools mkfs.ext4 -q -L rootfs "${loop}p2"
mkdir "$work/new"
mount "${loop}p2" "$work/new"
docker export "$NAME" | tar -C "$work/new" --xattrs --xattrs-include='*' --numeric-owner -xpf - \
  --exclude='boot/firmware/*' --exclude=.dockerenv
mkdir -p "$work/new/boot/firmware" "$work/boot"
docker export "$NAME" | tar -C "$work/boot" -xf - --wildcards 'boot/firmware/*'
mount "${loop}p1" "$work/new/boot/firmware"
rm -rf "${work:?}"/new/boot/firmware/*
cp -r --no-preserve=all "$work/boot/boot/firmware/." "$work/new/boot/firmware/"
docker rm "$NAME" >/dev/null

# undo what was only for the container, and leave what each device makes for itself on its first boot
r=$work/new
cp "$work/fstab" "$r/etc/fstab"
cp "$work/resolv.conf" "$r/etc/resolv.conf"
for unit in "${CONTAINER_MASKS[@]}"; do
  if [ "$(readlink "$r/etc/systemd/system/$unit")" = /dev/null ]; then rm "$r/etc/systemd/system/$unit"; fi
done
rm -f "$r/etc/initramfs-tools/conf.d/image-build.conf" "$r/etc/NetworkManager/conf.d/image-build.conf" "$r/etc/systemd/system/beep_server.service.d/image-build.conf"
rmdir "$r/etc/systemd/system/beep_server.service.d" 2>/dev/null || true
: > "$r/etc/machine-id"
chmod 444 "$r/etc/machine-id"
rm -f "$r"/etc/ssh/ssh_host_* "$r/var/swap" "$r/var/lib/systemd/random-seed"
# Raspberry Pi OS's first boot wizard would rename pi, which TacticalFrog's paths need. Only let it set the
# password from a userconf.txt (as Raspberry Pi Imager's settings write, with the username pi)
mkdir -p "$r/etc/systemd/system/userconfig.service.d"
printf '[Unit]\nConditionPathExists=|/boot/firmware/userconf.txt\nConditionPathExists=|/boot/firmware/userconf\n' \
  > "$r/etc/systemd/system/userconfig.service.d/tacticalfrog.conf"
grep -q 'init=/usr/lib/raspberrypi-sys-mods/firstboot' "$r/boot/firmware/cmdline.txt" ||
  { echo "cmdline.txt has lost the first boot resize" >&2; false; }
grep -q '^# BEGIN TacticalFrog' "$r/boot/firmware/config.txt" || { echo "config.txt wasn't set up" >&2; false; }
grep -q 'rfkill.default_state=1' "$r/boot/firmware/cmdline.txt" || { echo "wifi would start blocked" >&2; false; }
[ "$(readlink "$r/etc/systemd/system/systemd-rfkill.service")" = /dev/null ] || { echo "systemd-rfkill isn't masked" >&2; false; }
cat "$r/boot/firmware/cmdline.txt"
sync
umount "$r/boot/firmware" "$r"

step "Shrink it"
tools e2fsck -fy "${loop}p2" >/dev/null || [ $? -le 1 ]
tools resize2fs -M "${loop}p2"
blocks=$(tools dumpe2fs -h "${loop}p2" 2>/dev/null | awk -F: '/^Block count/ {print $2}' | tr -d ' ')
block_size=$(tools dumpe2fs -h "${loop}p2" 2>/dev/null | awk -F: '/^Block size/ {print $2}' | tr -d ' ')
losetup -d "$loop"
sectors=$((blocks * block_size / 512))
echo "$p2_start,$sectors" | sfdisk -q --no-reread -N 2 "$out/$image"
truncate -s $(((p2_start + sectors) * 512)) "$out/$image"
sfdisk -l "$out/$image"
loop=$(partitions "$out/$image")
tools e2fsck -fn "${loop}p2" >/dev/null
losetup -d "$loop"

step "Compress it"
xz -T0 -6 -f "$out/$image"
(cd "$out" && sha256sum "$image.xz" > "$image.xz.sha256")
ls -la "$out"
step "Done: $out/$image.xz"
