#!/bin/bash
# Environment for the TacticalFrog services: ROS2 and the workspace, then the separately updated packages on top
set -e
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

# use_package <name> <prefix>: put a colcon-built package ahead of the workspace's copy
use_package() {
  # must be set: otherwise package.sh uses the path it was built in, rather than <prefix>
  export COLCON_CURRENT_PREFIX=$2
  source "$2/share/$1/package.bash"
  unset COLCON_CURRENT_PREFIX
}

orb=/opt/orb_slam3/current
if [ -f "$orb/share/orb_slam3/package.bash" ]; then
  use_package orb_slam3 "$orb"
else
  # don't stop the services (and so the web UI) because of a bad orb_slam3 update
  echo "ros_env.sh: no orb_slam3 in $orb, using workspace environment only" >&2
fi

# releases (see make_release.sh) bring their own build of tactical_frog; a git checkout uses the workspace's
frog=/opt/TacticalFrog/current
if [ -f "$frog/ros_install/tactical_frog/share/tactical_frog/package.bash" ]; then
  use_package tactical_frog "$frog/ros_install/tactical_frog"
elif [ ! -e "$frog/.git" ]; then
  echo "ros_env.sh: release in $frog has no tactical_frog build, using the workspace's" >&2
fi

exec "$@"
