#!/bin/bash
# Make a stock Debian bookworm (arm64) install ready for the three TacticalFrog releases.
#
#   sudo ./bootstrap.sh        (or as root; needs internet, and takes an hour or more)
#   sudo reboot
# It installs the latest TacticalFrog release, and the orb_slam3 and OpenMVS releases that go with it, before the
# reboot: afterwards wifi is the device's own hotspot, with no internet. Everything starts after the reboot.
#
# It stands alone: it doesn't need a checkout of this repository. It installs everything that isn't in a
# release (OS packages, ROS, the camera stack, ~/ros2_ws, kalibr), sets up the system and installs the releases.
# It carries on if your ssh session drops (see the log), and can be run again, e.g. after a network failure. It refuses to run on a development device (one with
# /opt/TacticalFrog/current as a git checkout, or a ~/ros2_ws that it didn't make).
#
# Settings, e.g. sudo FROG_HOSTNAME=frog2 ./bootstrap.sh
#   FROG_HOSTNAME=frog   the device is then http://frog.local
#   WIFI_COUNTRY=GB      wifi regulatory domain, if one isn't set already
#   JOBS=                parallel compile jobs, default depends on CPUs and memory
#   FROG_RELEASE=latest  the TacticalFrog release to install (its tag), with its orb_slam3 and OpenMVS releases;
#                        "none" to install them by hand afterwards (see the end of the run)
#   FROG_IMAGE=0         1 when building a disk image for the CM5 somewhere else (.github/image/build.sh): set
#                        it up as a CM5 even though this isn't one
#
# Everything downloaded is pinned: change the versions below deliberately, and test on a spare device.
set -Eeuo pipefail

FROG_HOSTNAME=${FROG_HOSTNAME:-frog}
WIFI_COUNTRY=${WIFI_COUNTRY:-GB}
JOBS=${JOBS:-}
FROG_IMAGE=${FROG_IMAGE:-0}
FROG_RELEASE=${FROG_RELEASE:-latest}

# the services in the release run as pi, with these paths, so these aren't settings
FROG_USER=pi
FROG_HOME=/home/pi
WS=$FROG_HOME/ros2_ws
BOOT=/boot/firmware
CACHE=/var/cache/tacticalfrog-bootstrap
LOG=/var/log/tacticalfrog-bootstrap.log

# the same ROS as the releases are built against (.github/release/build.sh in each repository)
ROS_VERSION=0.3.2
ROS_DEB_URL=https://github.com/Ar-Ray-code/rpi-bullseye-ros2/releases/download/ros2-0.3.2/ros-jazzy-desktop-0.3.2_20240525_arm64.deb
ROS_DEB_SHA256=ddf1a5d9fea69f9dc2cd1a905eb8bb423b9f8404dd3b289c1d61e3f268600d32

RPI_KEY_URL=https://archive.raspberrypi.com/debian/raspberrypi.gpg.key
RPI_KEY_FINGERPRINT=CF8A1AF502A2AA2D763BAE7E82B129927FA3303E

# Arducam's libcamera, which has the IPA for the Pivariety cameras (Raspberry Pi's doesn't)
LIBCAMERA_VERSION=0.5.2+rpt20250909-1
LIBCAMERA_URL=https://github.com/ArduCAM/Arducam-Pivariety-V4L2-Driver/releases/download/libcamera-v0.0.5
LIBCAMERA_DEBS=(
  "libcamera0.5 85ae12c0f8c2809d97ecd8b0cb7ca641a178f5e5e5b6a1fa81ff61afc43763dd"
  "libcamera-ipa 247961da888ab177dfac1ceb8e0e75edf2cd2f8722be2fd46984041e87d12503"
  "libcamera-dev c56f7d09f0b1acedc8564ecda7aa395c9d01807fd7d0e7ae8aa27f2420fe4d03"
)

# kalibr for the camera-IMU calibration: furbrain's version, which needs neither ROS nor docker, built natively
KALIBR_REPO=furbrain/kalibr
KALIBR_COMMIT=3826babd1a7593fa30cf0e3676647f29d83fd553
KALIBR_PREFIX=/usr/local/lib/kalibr

# what goes in ~/ros2_ws/src:  <directory> <repository on github> <commit>
SOURCES='
async_web_server_cpp                fkie/async_web_server_cpp                 d1876da58215b853ac6fd66ef704442babc5f873
autonode                            furbrain/autonode                         2833fe0dd1e3c03ef229b70c6a64b36154a508a2
camera_ros                          furbrain/camera_ros                       f49f2980af22e3fe3e06923d189218a1e4063e10
diagnostics                         ros/diagnostics                           9b2d945722625811d0dfd3d2c946ceb01abc5607
icm20948_ros2                       furbrain/icm20948_ros2                    512f274bf23ba24939ca6369a98e21cafc926fdb
icm20948_ros2/ICM20948_I2CDev       furbrain/ICM20948_I2CDev                  4bfef4b6e74315b63d9f14a05b8d72f1d6169a37
image_common                        furbrain/image_common                     06a99a6c1b8d2c96ee791952f9562564196484b6
image_transport_plugins             ros-perception/image_transport_plugins    f3376b5c38867a0b305f3d918fc5d3c4423e2f75
imu_tools                           CCNYRoboticsLab/imu_tools                 69261f53be33cb489022cdedf217c7358905d888
ros2-system-monitor                 AgoraRobotics/ros2-system-monitor         a77b8ed8241cb8741d8925c84d95647d39e24af7
ros_mimu_calibrate                  furbrain/ros_mimu_calibrate               db56129db039dcbd4d0d80e79f34aa5ebb315d4c
rosbridge_suite                     RobotWebTools/rosbridge_suite             b1f6b4c0009da86c39132ad36e73cc54ae196a4e
tf2_web_republisher                 furbrain/tf2_web_republisher              8b8a46fb5cd0131dcd8f5044e7fa51af9d71e137
vision_opencv                       ros-perception/vision_opencv              27de9ecf9862e6fba509b7e49e3c2511c7d11627
web_video_server                    RobotWebTools/web_video_server            9eaa34357bae457fc6a41951f0c57a40c57c2b12
'

# Only these are built. In particular NOT image_common/image_transport: orb_slam3 releases are built
# against the one in /opt/ros, and a newer one in the workspace would hide it.
WS_PACKAGES=(
  async_web_server_cpp
  autonode
  camera_ros
  diagnostics/diagnostic_updater
  icm20948_ros2
  image_common/camera_calibration_parsers
  image_common/camera_info_manager
  image_transport_plugins/compressed_image_transport
  imu_tools/imu_filter_madgwick
  ros2-system-monitor
  ros_mimu_calibrate
  rosbridge_suite/rosapi
  rosbridge_suite/rosapi_msgs
  rosbridge_suite/rosbridge_library
  rosbridge_suite/rosbridge_msgs
  rosbridge_suite/rosbridge_server
  rosbridge_suite/rosbridge_test_msgs
  tf2_web_republisher/tf2_web_republisher
  tf2_web_republisher/tf2_web_republisher_interfaces
  vision_opencv/cv_bridge
  web_video_server
)

# Changes to the above that the devices have but which aren't in those commits.
# When one gets committed upstream, move the commit on and delete it from here.
workspace_patch() {
  case $1 in
  diagnostics) cat <<'EOF_PATCH'
diff --git a/diagnostic_updater/CMakeLists.txt b/diagnostic_updater/CMakeLists.txt
index 46c2ecc..50de257 100644
--- a/diagnostic_updater/CMakeLists.txt
+++ b/diagnostic_updater/CMakeLists.txt
@@ -7,7 +7,7 @@ if(NOT CMAKE_CXX_STANDARD)
 endif()
 
 if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_C_COMPILER_ID MATCHES "Clang")
-  add_compile_options(-Wall -Wextra -Wpedantic)
+  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
 endif()
 set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
 
EOF_PATCH
  ;;
  # subscriber.hpp only arrived in message_filters 4.11.6, and this ROS has 4.11.1 (a newer one in the workspace
  # would hide /opt/ros's from orb_slam3). From 4.11.6 subscriber.h just includes subscriber.hpp: the same code.
  ros_mimu_calibrate) cat <<'EOF_PATCH'
diff --git a/src/collate_node.cpp b/src/collate_node.cpp
index 26a2ed5..485f5cd 100644
--- a/src/collate_node.cpp
+++ b/src/collate_node.cpp
@@ -4,7 +4,7 @@
 #include <numeric>
 #include <vector>
 
-#include "message_filters/subscriber.hpp"
+#include "message_filters/subscriber.h"
 #include "message_filters/sync_policies/approximate_time.h"
 #include "message_filters/synchronizer.h"
 #include "rclcpp/rclcpp.hpp"
EOF_PATCH
  ;;
  rosbridge_suite) cat <<'EOF_PATCH'
diff --git a/rosbridge_server/src/rosbridge_server/websocket_handler.py b/rosbridge_server/src/rosbridge_server/websocket_handler.py
index 930a1c5..55a9366 100644
--- a/rosbridge_server/src/rosbridge_server/websocket_handler.py
+++ b/rosbridge_server/src/rosbridge_server/websocket_handler.py
@@ -192,6 +192,7 @@ class RosbridgeWebSocket(WebSocketHandler):
                 "WebSocketClosedError: Tried to write to a closed websocket",
                 throttle_duration_sec=1.0,
             )
+            self.close()
             # If we end up here, a client has disconnected before its message callback(s) could be removed.
         except StreamClosedError:
             cls.node_handle.get_logger().warning(
EOF_PATCH
  ;;
  *) return 0 ;;
  esac
}

APT_PACKAGES=(
  # system
  sudo openssh-server ca-certificates curl gnupg git rsync unzip procps psmisc kmod udev dbus polkitd
  iproute2 iputils-ping iw rfkill systemd-timesyncd fake-hwclock i2c-tools v4l-utils
  # network: dnsmasq-base is for NetworkManager's hotspot (the full dnsmasq would fight it for the port)
  network-manager wpasupplicant wireless-regdb dnsmasq-base nftables iptables avahi-daemon libnss-mdns
  # web and file sharing
  caddy samba samba-vfs-modules smbclient wsdd
  python3-flask gunicorn python3-gunicorn python3-gevent python3-zipstream-ng python3-requests
  # building the workspace
  build-essential cmake pkg-config python3-dev python3-pip python3-setuptools python3-empy
  libeigen3-dev libyaml-cpp-dev libssl-dev zlib1g-dev pybind11-dev libopencv-dev
  libboost-dev libboost-system-dev libboost-filesystem-dev libboost-regex-dev libboost-thread-dev
  libboost-python-dev
  libavcodec-dev libavformat-dev libavutil-dev libswscale-dev libpisp-dev
  # needed by the releases' binaries
  libboost-serialization-dev libboost-iostreams-dev libgomp1
  # python, for the nodes, rosbridge and modelling
  python3-numpy python3-opencv python3-matplotlib python3-networkx python3-psutil python3-smbus2
  python3-gpiozero python3-rpi-lgpio python3-systemd python3-yaml python3-ruamel.yaml python3-lz4
  python3-pyparsing python3-packaging python3-pil python3-bson python3-ujson python3-tornado
  python3-twisted
  # modelling
  survex nodejs npm fuse3 libfuse3-3
  # ros2_system_monitor runs these
  sysstat ifstat lm-sensors
  # kalibr
  libsuitesparse-dev python3-igraph python3-pyx
)

# Installed for everyone in /usr/local, without dependencies, so that pip can't replace Debian's packages:
# anything new these need has to be added here (or to APT_PACKAGES).
# scipy has to be >=1.16 for RigidTransform, so numpy is newer than Debian's, but must be <2 for gtsam and cv2.
PIP_PACKAGES=(
  numpy==1.26.4 scipy==1.17.0 gtsam==4.2.2
  dataclasses-json==0.6.7 marshmallow==3.26.1 typing-inspect==0.9.0 mypy-extensions==1.0.0
  typing_extensions==4.15.0
  rosbags==0.10.11 zstandard==0.24.0
  circuitpython-mag-cal==1.4.1
  ratarmount==1.3.0 ratarmountcore==0.11.1 mfusepy==3.1.1
  empy==3.3.4 catkin-pkg==0.5.2 distlib==0.4.0 pluggy==1.6.0
  colcon-core==0.20.0 colcon-common-extensions==0.3.0 colcon-argcomplete==0.3.3 colcon-bash==0.5.0
  colcon-cd==0.1.1 colcon-cmake==0.2.29 colcon-defaults==0.2.9 colcon-devtools==0.3.0
  colcon-library-path==0.2.1 colcon-metadata==0.2.5 colcon-notification==0.3.0 colcon-output==0.2.13
  colcon-package-information==0.4.0 colcon-package-selection==0.2.10 colcon-parallel-executor==0.3.0
  colcon-pkg-config==0.1.0 colcon-powershell==0.4.0 colcon-python-setup-py==0.2.9
  colcon-recursive-crawl==0.2.3 colcon-ros==0.5.0 colcon-test-result==0.3.8 colcon-zsh==0.5.0
)

# units in the TacticalFrog release, and what they are WantedBy
UNITS='
beep_server          default.target
caddy                multi-user.target
gunicorn             multi-user.target
multicast-loopback   multi-user.target
ros2_tactical_frog   multi-user.target
smbd                 multi-user.target
'

# ---------------------------------------------------------------------------------------------------------

WARNINGS=()
step() { printf '\n=== %s\n' "$*"; }
warn() { echo "WARNING: $*" >&2; WARNINGS+=("$*"); }
die() { echo "bootstrap.sh: $*" >&2; exit 1; }

apt_get() {
  DEBIAN_FRONTEND=noninteractive apt-get -o DPkg::Lock::Timeout=600 -o Acquire::Retries=5 \
    -o Dpkg::Options::=--force-confdef -o Dpkg::Options::=--force-confold "$@"
}

as_user() {
  runuser -u "$FROG_USER" -- env -i HOME="$FROG_HOME" USER="$FROG_USER" LANG=C.UTF-8 \
    PATH=/usr/local/bin:/usr/bin:/bin "$@"
}

# fetch <url> <sha256> <file>
fetch() {
  if ! echo "$2  $3" | sha256sum -c --status 2>/dev/null; then
    curl -fL --no-progress-meter --retry 5 --retry-all-errors -o "$3.part" "$1"
    echo "$2  $3.part" | sha256sum -c --quiet || die "wrong checksum for $1"
    mv "$3.part" "$3"
  fi
}

installed_version() { dpkg-query -W -f='${db:Status-Status} ${Version}' "$1" 2>/dev/null || true; }

is_pi() { [ "$FROG_IMAGE" = 1 ] || grep -qsa 'Raspberry Pi' /proc/device-tree/model; }

preflight() {
  [ "$(id -u)" = 0 ] || die "run this as root (sudo ./bootstrap.sh)"
  (. /etc/os-release && [ "${VERSION_CODENAME:-}" = bookworm ]) || die "this needs Debian 12 (bookworm)"
  [ "$(dpkg --print-architecture)" = arm64 ] || die "this needs a 64 bit ARM (arm64) install"
  [ -d /run/systemd/system ] || die "this needs a booted system with systemd, not a chroot or container"

  # don't wreck a development device
  if [ -e /opt/TacticalFrog/current/.git ]; then
    die "/opt/TacticalFrog/current is a git checkout: this is a development device, nothing has been changed"
  fi
  if [ -e "$WS" ] && [ ! -e "$WS/.bootstrap" ]; then
    die "$WS exists and wasn't made by this script: move it out of the way, nothing has been changed"
  fi

  if [ ! -e "$WS/.bootstrap" ]; then
    local need=10 free
    free=$(df --output=avail -BG / | tail -n 1 | tr -dc 0-9)
    [ "$free" -ge "$need" ] || die "needs ${need}GB free on /, there is only ${free}GB"
  fi

  if [ -z "$JOBS" ]; then
    local by_memory
    by_memory=$(awk '/^MemTotal:/ {print int($2 / 1500000)}' /proc/meminfo)
    JOBS=$(nproc)
    [ "$by_memory" -ge "$JOBS" ] || JOBS=$by_memory
    [ "$JOBS" -ge 1 ] || JOBS=1
  fi
}

setup_apt() {
  step "Package sources"
  # installs from a DVD only know about the DVD
  if [ -f /etc/apt/sources.list ]; then
    sed -i 's/^deb cdrom:/# &/' /etc/apt/sources.list
  fi
  if ! grep -rqsE '^[^#]*(debian\.org|mirror\+file)' /etc/apt/sources.list /etc/apt/sources.list.d; then
    cat > /etc/apt/sources.list.d/debian-bookworm.list <<'EOF'
deb http://deb.debian.org/debian bookworm main contrib non-free non-free-firmware
deb http://deb.debian.org/debian bookworm-updates main contrib non-free non-free-firmware
deb http://deb.debian.org/debian-security bookworm-security main contrib non-free non-free-firmware
EOF
  fi
  # finish off anything a power cut or earlier failure left half installed
  DEBIAN_FRONTEND=noninteractive dpkg --configure -a --force-confdef --force-confold
  apt_get update
  apt_get install -y --no-install-recommends ca-certificates curl gnupg

  # Raspberry Pi's archive: for the kernel, firmware and the libraries that libcamera needs
  if ! grep -rqsE '^[^#]*archive\.raspberrypi\.(com|org)' /etc/apt/sources.list /etc/apt/sources.list.d; then
    local key=$CACHE/raspberrypi.key found
    curl -fsSL --retry 5 -o "$key" "$RPI_KEY_URL"
    found=$(gpg --show-keys --with-colons "$key" | awk -F: '/^fpr/ && !seen {print $10; seen=1}')
    [ "$found" = "$RPI_KEY_FINGERPRINT" ] || die "unexpected signing key for the Raspberry Pi archive"
    gpg --dearmor < "$key" > /usr/share/keyrings/tacticalfrog-raspberrypi.gpg
    chmod 644 /usr/share/keyrings/tacticalfrog-raspberrypi.gpg
    echo "deb [arch=arm64 signed-by=/usr/share/keyrings/tacticalfrog-raspberrypi.gpg]" \
      "http://archive.raspberrypi.com/debian/ bookworm main" > /etc/apt/sources.list.d/raspi.list
    apt_get update
  fi
}

install_packages() {
  step "OS packages"
  apt_get install -y --no-install-recommends "${APT_PACKAGES[@]}"
  if is_pi; then
    apt_get install -y --no-install-recommends linux-image-rpi-2712 raspi-firmware firmware-brcm80211
  fi
}

install_ros() {
  step "ROS 2 jazzy"
  [ "$(installed_version ros-jazzy-desktop)" != "installed $ROS_VERSION" ] || return 0
  fetch "$ROS_DEB_URL" "$ROS_DEB_SHA256" "$CACHE/ros.deb"
  apt_get install -y --no-install-recommends "$CACHE/ros.deb"
  rm -f "$CACHE/ros.deb"
}

install_libcamera() {
  step "Arducam's libcamera"
  local entry name sha debs=() names=() current=1
  for entry in "${LIBCAMERA_DEBS[@]}"; do
    read -r name sha <<< "$entry"
    names+=("$name")
    debs+=("$CACHE/$name.deb")
    [ "$(installed_version "$name")" = "installed $LIBCAMERA_VERSION" ] || current=0
  done
  if [ "$current" = 0 ]; then
    for entry in "${LIBCAMERA_DEBS[@]}"; do
      read -r name sha <<< "$entry"
      fetch "$LIBCAMERA_URL/${name}_${LIBCAMERA_VERSION}_arm64.deb" "$sha" "$CACHE/$name.deb"
    done
    apt_get install -y --no-install-recommends --allow-downgrades --allow-change-held-packages "${debs[@]}"
    rm -f "${debs[@]}"
  fi
  # an apt upgrade would swap in Raspberry Pi's, and the cameras would stop working
  apt-mark hold "${names[@]}"
}

install_python_packages() {
  step "Python packages"
  python3 -m pip install --break-system-packages --root-user-action=ignore --disable-pip-version-check \
    --no-deps --retries 10 --timeout 60 "${PIP_PACKAGES[@]}"
}

install_gltf_pipeline() {
  step "gltf-pipeline"
  local dir=/usr/local/lib/gltf-pipeline
  if [ ! -e "$dir/.done" ]; then
    mkdir -p "$dir"
    # cesium is pinned too, as newer ones don't all work with bookworm's node 18
    cat > "$dir/package.json" <<'EOF'
{
  "private": true,
  "dependencies": {"gltf-pipeline": "4.3.1"},
  "overrides": {"cesium": "1.146.0", "@cesium/engine": "26.4.0", "@cesium/widgets": "16.3.0"}
}
EOF
    (cd "$dir" && npm install --omit=dev --no-audit --no-fund)
    touch "$dir/.done"
  fi
  ln -sfn "$dir/node_modules/.bin/gltf-pipeline" /usr/local/bin/gltf-pipeline
}

setup_user() {
  step "User $FROG_USER, groups and directories"
  local group
  if ! id "$FROG_USER" >/dev/null 2>&1; then
    useradd --create-home --home-dir "$FROG_HOME" --user-group --shell /bin/bash "$FROG_USER"
  fi
  [ "$(getent passwd "$FROG_USER" | cut -d: -f6)" = "$FROG_HOME" ] || die "$FROG_USER's home must be $FROG_HOME"
  if [ "$(passwd -S "$FROG_USER" | cut -d' ' -f2)" != P ]; then
    warn "$FROG_USER has no password: set one with 'sudo passwd $FROG_USER' to be able to log in as it"
  fi
  for group in gpio i2c spi; do
    getent group "$group" >/dev/null || groupadd --system "$group"
  done
  for group in adm audio dialout video render input plugdev netdev gpio i2c spi; do
    if getent group "$group" >/dev/null; then usermod -aG "$group" "$FROG_USER"; fi
  done
  usermod -aG "$FROG_USER" caddy

  install -d -m 755 /opt/TacticalFrog /opt/orb_slam3 /opt/OpenMVS
  install -d -m 755 -o "$FROG_USER" -g "$FROG_USER" /data /data/trips /data/current
  install -d -m 755 -o caddy -g caddy /var/log/caddy
  # calibration fails if cam_imu isn't there to be deleted
  as_user mkdir -p "$FROG_HOME"/.ros/{cal,cam_imu,camera_info,transforms}

  # kalibr needs the IMU's noise figures: these are for the ICM20948
  local noise=$FROG_HOME/.ros/cal/imu_noise.yaml
  if [ ! -e "$noise" ]; then
    as_user tee "$noise" >/dev/null <<'EOF'
#Accelerometer
accelerometer_noise_density: 0.0009961252264620393
accelerometer_random_walk: 8.528895026931483e-05

#Gyroscope
gyroscope_noise_density: 0.0006310895475187677
gyroscope_random_walk: 8.581361665275098e-06

rostopic: '/imu_raw'
update_rate: 200
EOF
  fi

  # the rectify node dies without a calibration, so start with one that changes nothing
  local cal=$FROG_HOME/.ros/imu_calibration.yaml
  if [ ! -e "$cal" ]; then
    as_user tee "$cal" >/dev/null <<'EOF'
calibration:
  accel:
    sensor:
      axes: +X+Y+Z
      centre: [0, 0, 0]
      field_avg: null
      field_std: null
      rbfs: []
      transform: [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
  gyro:
    axes: +X+Y+Z
  magnetic:
    sensor:
      axes: +X+Y+Z
      centre: [0, 0, 0]
      field_avg: null
      field_std: null
      rbfs: []
      transform: [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
EOF
  fi
}

# checkout <dir> <github repository> <commit>
checkout() {
  local attempt
  as_user mkdir -p "$1"
  as_user git -C "$1" init -q
  for attempt in 1 2 3 4 5; do
    if as_user git -C "$1" fetch -q --depth 1 "https://github.com/$2.git" "$3"; then
      as_user git -C "$1" -c advice.detachedHead=false checkout -q -f FETCH_HEAD
      return 0
    fi
    sleep $((attempt * 5))
  done
  die "couldn't fetch $2 at $3"
}

build_workspace() {
  step "ROS workspace ($JOBS jobs)"
  local stamp dir repo commit paths=()
  stamp=$({ echo "$SOURCES" "${WS_PACKAGES[*]}"; declare -f workspace_patch build_workspace; } |
    sha256sum | cut -d' ' -f1)
  if [ -e "$WS/.bootstrap" ]; then
    case $(cat "$WS/.bootstrap") in
      "$stamp") echo "already built"; return 0 ;;
      building) rm -rf "$WS" ;;   # an unfinished run of this script
      *) mv "$WS" "$WS.old-$(date +%Y%m%d-%H%M%S)" ;;
    esac
  fi
  as_user mkdir -p "$WS/src"
  echo building | as_user tee "$WS/.bootstrap" >/dev/null

  while read -r dir repo commit; do
    [ -n "$dir" ] || continue
    echo "$dir"
    checkout "$WS/src/$dir" "$repo" "$commit" </dev/null
  done <<< "$SOURCES"

  while read -r dir repo commit; do
    [ -n "$dir" ] || continue
    if [ -n "$(workspace_patch "$dir")" ]; then
      echo "patching $dir"
      workspace_patch "$dir" | as_user git -C "$WS/src/$dir" apply -
    fi
  done <<< "$SOURCES"
  # ament's marker file was never committed
  as_user mkdir -p "$WS/src/autonode/resource"
  as_user touch "$WS/src/autonode/resource/autonode"

  for dir in "${WS_PACKAGES[@]}"; do paths+=("src/$dir"); done
  # Debug is what the devices have been tested with; -g0 as debug info needs lots of memory to build
  as_user COLCON_DEFAULTS_FILE=/dev/null MAKEFLAGS="-j$JOBS" bash -c '
    set -e
    cd "$1"; shift
    source /opt/ros/jazzy/setup.bash
    exec colcon build --executor sequential --base-paths "$@" \
      --allow-overriding camera_calibration_parsers camera_info_manager \
      --cmake-args -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=OFF \
        -DCMAKE_C_FLAGS_DEBUG=-g0 -DCMAKE_CXX_FLAGS_DEBUG=-g0
  ' build "$WS" "${paths[@]}"

  echo "$stamp" | as_user tee "$WS/.bootstrap" >/dev/null
}

setup_boot() {
  step "Boot configuration"
  if ! is_pi; then
    warn "this isn't a Raspberry Pi: skipped the boot configuration, so no cameras, IMU or beeper"
    return 0
  fi
  if [ "$FROG_IMAGE" != 1 ] && ! grep -qa 'Compute Module 5' /proc/device-tree/model; then
    warn "this isn't a CM5, which is all that's supported"
  fi
  if [ ! -f "$BOOT/config.txt" ]; then
    warn "no $BOOT/config.txt: set up the overlays in raspbian/config.txt by hand"
    return 0
  fi
  edit_config_txt "$BOOT/config.txt"
  [ ! -f "$BOOT/cmdline.txt" ] || edit_cmdline "$BOOT/cmdline.txt"
}

edit_config_txt() {
  local file=$1 begin='# BEGIN TacticalFrog' end='# END TacticalFrog' new
  [ -e "$file.pre-tacticalfrog" ] || cp "$file" "$file.pre-tacticalfrog"
  new=$(mktemp)
  # drop our old block, and comment out anything that would clash with it
  sed -e "/^$begin\$/,/^$end\$/d" "$file" | sed -E \
    -e 's/^[[:space:]]*(camera_auto_detect=|display_auto_detect=|dtparam=ant[12])/#&/' \
    -e 's/^[[:space:]]*dtoverlay=(dwc2|arducam-pivariety|i2c1-pi5|i2c3-pi5|pwm)(,|[[:space:]]|$)/#&/' > "$new"
  cat >> "$new" <<EOF
$begin
[all]
arm_64bit=1
i2c_arm=on
i2c_dsi_csi=on
camera_auto_detect=0
display_auto_detect=0
[cm5]
dtoverlay=dwc2,dr_mode=peripheral
dtoverlay=i2c1-pi5,baudrate=400000
dtoverlay=i2c3-pi5,pins_14_15,baudrate=100000
[all]
dtoverlay=arducam-pivariety,cam0
dtoverlay=arducam-pivariety,cam1
dtoverlay=pwm,pin=19,func=2
dtparam=ant2
$end
EOF
  cp "$new" "$file"
  rm -f "$new"
}

edit_cmdline() {
  local file=$1 line
  [ -e "$file.pre-tacticalfrog" ] || cp "$file" "$file.pre-tacticalfrog"
  line=$(head -n 1 "$file")
  [ -n "$line" ] || die "$file is empty"
  # a serial console over USB
  if [[ $line != *g_serial* ]]; then
    if [[ $line == *modules-load=* ]]; then
      line=$(sed -E 's/modules-load=[^ ]*/&,dwc2,g_serial/' <<< "$line")
    else
      line="$line modules-load=dwc2,g_serial"
    fi
  fi
  [[ $line == *cfg80211.ieee80211_regdom=* ]] || line="$line cfg80211.ieee80211_regdom=$WIFI_COUNTRY"
  # radios start unblocked: modprobe applies this after Raspberry Pi OS's modprobe.d (rfkill is a module)
  line=$(sed -E 's/ ?rfkill\.default_state=[^ ]*//g' <<< "$line")
  line="$line rfkill.default_state=1"
  echo "$line" > "$file"
}

setup_system() {
  step "System configuration"
  echo i2c-dev > /etc/modules-load.d/tacticalfrog.conf

  # Raspberry Pi OS has these already, Debian doesn't
  cat > /etc/udev/rules.d/90-tacticalfrog.rules <<'EOF'
SUBSYSTEM=="i2c-dev", GROUP="i2c", MODE="0660"
SUBSYSTEM=="spidev", GROUP="spi", MODE="0660"
SUBSYSTEM=="gpio", GROUP="gpio", MODE="0660"
SUBSYSTEM=="*gpiomem*", GROUP="gpio", MODE="0660"
SUBSYSTEM=="dma_heap", GROUP="video", MODE="0660"
EOF

  install -d /etc/systemd/journald.conf.d
  printf '[Journal]\nStorage=persistent\n' > /etc/systemd/journald.conf.d/tacticalfrog.conf

  if [ -d /etc/cloud/cloud.cfg.d ]; then
    echo 'preserve_hostname: true' > /etc/cloud/cloud.cfg.d/99-tacticalfrog.cfg
  fi
  echo "$FROG_HOSTNAME" > /etc/hostname
  hostname "$FROG_HOSTNAME"
  if grep -q '^127\.0\.1\.1' /etc/hosts; then
    sed -i "s/^127\.0\.1\.1.*/127.0.1.1\t$FROG_HOSTNAME/" /etc/hosts
  else
    printf '127.0.1.1\t%s\n' "$FROG_HOSTNAME" >> /etc/hosts
  fi

  if is_pi; then systemctl enable serial-getty@ttyGS0.service; fi
}

setup_network() {
  step "Network"
  # as raspbian/10-nm-wifi.rules: lets the web UI manage wifi. The name must sort before 49-polkit-pkla-compat
  cat > /etc/polkit-1/rules.d/10-nm-wifi.rules <<'EOF'
polkit.addRule(function(action, subject) {
     if (action.id.indexOf("org.freedesktop.NetworkManager.") === 0 &&
         subject.user == "pi") {
       return polkit.Result.YES;
     }
   });
EOF
  chmod 644 /etc/polkit-1/rules.d/10-nm-wifi.rules
  # and to start the software update (raspbian/tacticalfrog-update.service), which runs as root
  cat > /etc/polkit-1/rules.d/10-tacticalfrog-update.rules <<'EOF'
polkit.addRule(function(action, subject) {
     if (action.id == "org.freedesktop.systemd1.manage-units" &&
         action.lookup("unit") == "tacticalfrog-update.service" &&
         action.lookup("verb") == "start" &&
         subject.user == "pi") {
       return polkit.Result.YES;
     }
   });
EOF
  chmod 644 /etc/polkit-1/rules.d/10-tacticalfrog-update.rules

  install -d /etc/NetworkManager/conf.d /etc/NetworkManager/system-connections
  printf '[connection]\nwifi.powersave=2\n' > /etc/NetworkManager/conf.d/tacticalfrog-powersave.conf

  # as raspbian/AP.nmconnection. NetworkManager isn't reloaded, so this can't cut off an ssh session
  local ap=/etc/NetworkManager/system-connections/AP.nmconnection
  if [ ! -e "$ap" ]; then
    install -m 600 /dev/null "$ap"
    cat > "$ap" <<'EOF'
[connection]
id=AP
uuid=a3384243-4a6d-4ec9-a078-00587d6a097f
type=wifi
autoconnect-priority=1
interface-name=wlan0

[wifi]
band=bg
channel=2
mode=ap
ssid=TacticalFrog

[ipv4]
method=shared

[ipv6]
addr-gen-mode=default
method=auto

[proxy]
EOF
  fi
  if grep -rqsE '^[^#]*iface[[:space:]]+wlan0' /etc/network/interfaces /etc/network/interfaces.d; then
    warn "wlan0 is in /etc/network/interfaces: the hotspot and wifi page need it removed from there"
  fi
  systemctl enable NetworkManager.service avahi-daemon.service

  # The hotspot needs wifi, so radios are never blocked: systemd-rfkill would save a block and restore it at
  # boot, and Raspberry Pi OS blocks them all to start with (rfkill default_state=0 in its modprobe.d, which
  # edit_cmdline overrides)
  systemctl mask systemd-rfkill.service systemd-rfkill.socket
  if is_pi && [ "$FROG_IMAGE" != 1 ]; then rfkill unblock wifi || warn "couldn't unblock wifi"; fi
}

setup_services() {
  step "Services"
  # the packaged caddy, smbd and friends started when they were installed; the release's replace them
  systemctl stop caddy.service smbd.service 2>/dev/null || true
  systemctl disable --now nmbd.service 2>/dev/null || true
  systemctl mask --now samba-ad-dc.service
  systemctl enable wsdd.service

  # systemctl enable won't take units that don't exist yet, so make the same links it would
  local unit target
  while read -r unit target; do
    [ -n "$unit" ] || continue
    install -d "/etc/systemd/system/$target.wants"
    ln -sfn "/opt/TacticalFrog/current/raspbian/$unit.service" "/etc/systemd/system/$unit.service"
    ln -sfn "/etc/systemd/system/$unit.service" "/etc/systemd/system/$target.wants/$unit.service"
  done <<< "$UNITS"
  # not enabled: the web UI starts it
  ln -sfn /opt/TacticalFrog/current/raspbian/tacticalfrog-update.service \
    /etc/systemd/system/tacticalfrog-update.service

  # start everything when the last release arrives, so that it doesn't need another reboot
  cat > /usr/local/sbin/tacticalfrog-await-releases <<'EOF'
#!/bin/bash
ready() {
  [ -f /opt/TacticalFrog/current/raspbian/ros_env.sh ] &&
    [ -f /opt/orb_slam3/current/share/orb_slam3/package.bash ] &&
    [ -x /opt/OpenMVS/current/DensifyPointCloud ]
}
until ready; do sleep 5; done
sleep 10 # in case one is still being copied
systemctl daemon-reload
exec systemctl --no-block restart multicast-loopback beep_server caddy smbd gunicorn ros2_tactical_frog
EOF
  chmod 755 /usr/local/sbin/tacticalfrog-await-releases

  cat > /etc/systemd/system/tacticalfrog-await-releases.service <<'EOF'
[Unit]
Description=Start TacticalFrog once its releases are installed in /opt
ConditionPathExists=|!/opt/TacticalFrog/current/raspbian/ros_env.sh
ConditionPathExists=|!/opt/orb_slam3/current/share/orb_slam3/package.bash
ConditionPathExists=|!/opt/OpenMVS/current/DensifyPointCloud

[Service]
Type=exec
ExecStart=/usr/local/sbin/tacticalfrog-await-releases

[Install]
WantedBy=multi-user.target
EOF
  systemctl daemon-reload
  systemctl enable tacticalfrog-await-releases.service
}

install_kalibr() {
  step "kalibr"
  local dir=$CACHE/kalibr stamp attempt site
  # rebuilt if numpy changes too, as its extension modules are built against it
  stamp="$KALIBR_COMMIT numpy-$(python3 -c 'import numpy; print(numpy.__version__)')"
  if [ "$(cat "$KALIBR_PREFIX/.bootstrap" 2>/dev/null)" = "$stamp" ]; then
    echo "already built"
  else
    rm -rf "$dir"
    git init -q "$dir/src"
    for attempt in 1 2 3 4 5; do
      git -C "$dir/src" fetch -q --depth 1 "https://github.com/$KALIBR_REPO.git" "$KALIBR_COMMIT" && break
      [ "$attempt" -lt 5 ] || die "couldn't fetch $KALIBR_REPO at $KALIBR_COMMIT"
      sleep $((attempt * 5))
    done
    git -C "$dir/src" -c advice.detachedHead=false checkout -q FETCH_HEAD
    # against the numpy in /usr/local (see PIP_PACKAGES), which it runs with
    cmake -S "$dir/src" -B "$dir/build" -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3 \
      -DCMAKE_INSTALL_PREFIX="$KALIBR_PREFIX" -DKALIBR_PYTHON_INSTALL_DIR="$KALIBR_PREFIX/python" \
      -DKALIBR_BIN_INSTALL_DIR="$KALIBR_PREFIX/bin"
    cmake --build "$dir/build" --parallel "$JOBS"
    rm -rf "$KALIBR_PREFIX"
    cmake --install "$dir/build"
    echo "$stamp" > "$KALIBR_PREFIX/.bootstrap"
    rm -rf "$dir"
  fi
  # its Python packages, for everyone's python3, and its tools on the PATH
  site=$(python3 -c 'import site; print(site.getsitepackages()[0])') # /usr/local/lib/python3.X/dist-packages
  mkdir -p "$site"
  echo "$KALIBR_PREFIX/python" > "$site/kalibr.pth"
  ln -sfn "$KALIBR_PREFIX"/bin/kalibr_* /usr/local/bin/
}

# the GitHub repository each release comes from
release_repo() {
  case $1 in
    TacticalFrog) echo furbrain/TacticalFrog ;;
    orb_slam3) echo furbrain/orb_slam3_ros ;;
    OpenMVS) echo furbrain/openMVS ;;
  esac
}

# fetch_release <name> <tag>: download a release's tarball into $CACHE, check it, and print where it is
fetch_release() {
  local version file url
  version=$(printf %s "$2" | tr -c 'A-Za-z0-9._+-' '-') # as the release builds name their files
  file=$1-$version-aarch64.tar.gz
  url=https://github.com/$(release_repo "$1")/releases/download/$2/$file
  curl -fsSL --retry 5 --retry-all-errors -o "$CACHE/$file.sha256" "$url.sha256" >&2 || return 1
  curl -fL --no-progress-meter --retry 5 --retry-all-errors -o "$CACHE/$file" "$url" >&2 || return 1
  (cd "$CACHE" && sha256sum -c --quiet "$file.sha256" >&2) || die "$file is corrupt"
  echo "$CACHE/$file"
}

RELEASES_INSTALLED=
install_releases() {
  step "Releases"
  if [ "$FROG_RELEASE" = none ]; then
    echo "skipped (FROG_RELEASE=none)"
    return 0
  fi
  local tag=$FROG_RELEASE tf name commit partner_tag tarball got folder tarballs=()
  if [ "$tag" = latest ]; then
    if ! tag=$(curl -fsSL --retry 5 https://api.github.com/repos/furbrain/TacticalFrog/releases/latest |
      python3 -c 'import json, sys; print(json.load(sys.stdin)["tag_name"])' 2>/dev/null); then
      warn "no TacticalFrog release could be found on GitHub, so install the releases by hand"
      return 0
    fi
  fi
  # all three are downloaded and checked before any is installed
  echo "TacticalFrog $tag"
  tf=$(fetch_release TacticalFrog "$tag") || die "couldn't download TacticalFrog $tag"
  tarballs=("$tf")
  while read -r name commit partner_tag; do
    case $name in orb_slam3 | OpenMVS) ;; *) continue ;; esac
    [ "$partner_tag" != - ] || die "TacticalFrog $tag was built with an $name commit that has no release"
    echo "$name $partner_tag"
    tarball=$(fetch_release "$name" "$partner_tag") || die "couldn't download $name $partner_tag"
    got=$(tar -xzOf "$tarball" "$(basename "$tarball" -aarch64.tar.gz)/VERSION" | cut -d' ' -f2)
    [ "$got" = "$commit" ] || die "$name $partner_tag is commit $got, but TacticalFrog $tag needs $commit"
    tarballs+=("$tarball")
  done < <(tar -xzOf "$tf" "$(basename "$tf" -aarch64.tar.gz)/RELEASES" 2>/dev/null | grep -v '^#' || true)
  [ "${#tarballs[@]}" = 3 ] || die "TacticalFrog $tag doesn't say which orb_slam3 and OpenMVS releases go with it"
  for tarball in "${tarballs[@]}"; do
    folder=$(basename "$tarball" -aarch64.tar.gz)
    name=${folder%%-*}
    if [ ! -f "/opt/$name/$folder/VERSION" ]; then
      tar -xzf "$tarball" -C "/opt/$name" --no-same-owner
    fi
    ln -sfn "$folder" "/opt/$name/current"
    rm -f "$tarball" "$tarball.sha256"
  done
  RELEASES_INSTALLED=$tag
}

check_install() {
  step "Checking"
  # not a test of the hardware, just that everything the releases expect is there and loads
  as_user bash -c '
    set -e
    cd
    source "$HOME/ros2_ws/install/setup.bash"
    for tool in cavern dump3d gltf-pipeline ratarmount rosbags-convert nmcli make mpstat ifstat fusermount3 \
      kalibr_calibrate_imu_camera kalibr_calibrate_cameras; do
      command -v "$tool" >/dev/null || PATH=$PATH:/usr/sbin:/sbin command -v "$tool" >/dev/null ||
        { echo "missing: $tool" >&2; exit 1; }
    done
    gltf-pipeline --version >/dev/null
    ratarmount --version >/dev/null
    python3 - <<EOF
import cv2, dataclasses_json, flask, gevent, gpiozero, gtsam, matplotlib, networkx, psutil, smbus2
import systemd.daemon, mag_cal, rclpy, rosbags, tornado, bson
from scipy.spatial.transform import RigidTransform
import autonode, cv_bridge, rosbridge_library, rosapi
import sm, aslam_cv, kalibr_common, kalibr_imu_camera_calibration
EOF
    bad=$(find "$HOME/ros2_ws/install" /usr/local/lib/kalibr -type f \( -name "*.so" -o -path "*/lib/*" -perm -100 \) \
      -exec sh -c "ldd \"\$1\" 2>/dev/null | grep \"not found\" | sed \"s|^|\$1: |\"" sh {} \;)
    [ -z "$bad" ] || { echo "$bad" >&2; exit 1; }
  '
  echo ok
}

finish() {
  local w
  cat <<EOF

=== Done. Log: $LOG

EOF
  local have=1 name
  for name in TacticalFrog orb_slam3 OpenMVS; do [ -f "/opt/$name/current/VERSION" ] || have=; done
  if [ -n "$RELEASES_INSTALLED" ] || [ -n "$have" ]; then
    if [ -n "$RELEASES_INSTALLED" ]; then
      echo "TacticalFrog $RELEASES_INSTALLED is installed, with the orb_slam3 and OpenMVS releases that go with it."
    else
      echo "The releases already in /opt are kept: TacticalFrog $(cut -d' ' -f1 /opt/TacticalFrog/current/VERSION)."
    fi
    cat <<EOF

1. Reboot:  sudo reboot
   After that wifi starts as the open "TacticalFrog" hotspot, which takes priority over saved networks.
   Join it (or stay on ethernet) and go to http://$FROG_HOSTNAME.local

2. Calibrate the cameras and IMU from the web page, as they're different on every device.
EOF
  else
    cat <<EOF
The releases aren't installed. Download them now, while there's internet, from
https://github.com/furbrain/TacticalFrog/releases (the TacticalFrog release's RELEASES file names the
orb_slam3 and OpenMVS releases that go with it), then:

1. Install the three releases (in any order). For each of TacticalFrog, orb_slam3 and OpenMVS:
     sudo tar -xzf NAME-VERSION-aarch64.tar.gz -C /opt/NAME --no-same-owner
     sudo ln -sfn NAME-VERSION /opt/NAME/current

2. Reboot:  sudo reboot
   After that wifi starts as the open "TacticalFrog" hotspot, which takes priority over saved networks.
   Join it (or stay on ethernet) and go to http://$FROG_HOSTNAME.local
   (Installed after the reboot instead, it all starts by itself a few seconds after the third.)

3. Calibrate the cameras and IMU from the web page, as they're different on every device.
EOF
  fi
  cat <<EOF

Later releases install themselves from the web page, Settings > Software update, which updates all three together.

Optional, for the "frog" file sharing login that Windows needs:  sudo smbpasswd -a $FROG_USER
EOF
  for w in "${WARNINGS[@]}"; do echo "WARNING: $w"; done
}

main() {
  # su without - leaves out the sbins
  export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
  umask 022
  preflight
  # carry on if the ssh session drops: ignore the hangup, and keep logging when the terminal has gone
  trap '' HUP
  exec > >(trap '' INT; exec tee -p -a "$LOG") 2>&1
  trap 'echo "bootstrap.sh: FAILED at line $LINENO: $BASH_COMMAND (see $LOG; it is safe to run again)"' ERR
  cd /
  mkdir -p "$CACHE"

  setup_apt
  install_packages
  install_ros
  install_libcamera
  install_python_packages
  install_gltf_pipeline
  setup_user
  build_workspace
  setup_boot
  setup_system
  setup_network
  setup_services
  install_kalibr
  install_releases
  check_install
  finish
}

# not when sourced; stdin is closed as this may be being piped into bash
if [ "${BASH_SOURCE[0]:-$0}" = "$0" ]; then
  main "$@" </dev/null
fi
