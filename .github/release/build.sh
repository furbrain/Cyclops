#!/bin/bash
# Build a TacticalFrog release in a clean debian:bookworm arm64 container, using make_release.sh:
#   docker run --rm -v $PWD:/src:ro -v /some/dir:/out -e VERSION=v1 -e COMMIT=$(git rev-parse HEAD) \
#     debian:bookworm bash /src/.github/release/build.sh
set -euo pipefail
umask 022

: "${VERSION:?}" "${COMMIT:?}"
VERSION=$(printf %s "$VERSION" | tr -c 'A-Za-z0-9._+-' '-') # tags can have slashes etc in them
JOBS=${JOBS:-$(nproc)}
export DEBIAN_FRONTEND=noninteractive

# the same ROS as install.md puts on the devices
ROS_DEB_URL=https://github.com/Ar-Ray-code/rpi-bullseye-ros2/releases/download/ros2-0.3.2/ros-jazzy-desktop-0.3.2_20240525_arm64.deb
ROS_DEB_SHA256=ddf1a5d9fea69f9dc2cd1a905eb8bb423b9f8404dd3b289c1d61e3f268600d32

apt-get update -qq
apt-get install -qq --no-install-recommends ca-certificates curl >/dev/null
curl -fsSL --retry 5 -o /tmp/ros.deb "$ROS_DEB_URL"
echo "$ROS_DEB_SHA256  /tmp/ros.deb" | sha256sum -c -
apt-get install -qq --no-install-recommends /tmp/ros.deb \
  git python3-empy python3-numpy python3-yaml python3-dev g++ >/dev/null
pip install -q --break-system-packages --root-user-action=ignore \
  colcon-core==0.20.0 colcon-common-extensions==0.3.0 catkin-pkg==0.5.2

git config --global --add safe.directory '*' # /src belongs to the host's user
RELEASE_VERSION=$VERSION /src/make_release.sh "$COMMIT" /out

cd /out
tarball=$(ls TacticalFrog-"$VERSION"-*.tar.gz)

# check that it works from somewhere other than where it was built
test=$(mktemp -d)
tar -xzf "$tarball" -C "$test"
prefix=$test/TacticalFrog-$VERSION/ros_install/tactical_frog
(
  set +u
  source /opt/ros/jazzy/setup.bash
  export COLCON_CURRENT_PREFIX=$prefix
  source "$prefix/share/tactical_frog/package.bash"
  python3 -c '
from rclpy.type_support import check_for_type_support as check
from tactical_frog import msg, srv, action
for t in (msg.Sync, srv.SetMode, action.Calibrate):
    check(t)
print("tactical_frog release ok")'
)

sha256sum "$tarball" > "$tarball.sha256"
