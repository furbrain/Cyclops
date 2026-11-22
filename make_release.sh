#!/bin/bash
# Build a TacticalFrog release:  ./make_release.sh [git ref, default HEAD] [output dir, default dist/]
#
# Run it on a Pi with the same OS and ROS as the devices it's for, as it compiles the tactical_frog interfaces.
# Only committed files are included. The tarball holds one folder, TacticalFrog-<version>, which is what
# /opt/TacticalFrog/current should link to:
#   raspbian/ modelling/ www/     as in the repo
#   ros_install/tactical_frog/    the built ROS package, loaded by raspbian/ros_env.sh
#   VERSION
# orb_slam3 and OpenMVS aren't included, they are updated separately.
set -euo pipefail
umask 022

repo=$(cd "$(dirname "$0")" && pwd)
ref=${1:-HEAD}
out=${2:-$repo/dist}

commit=$(git -C "$repo" rev-parse --verify "$ref^{commit}")
version=$(git -C "$repo" describe --tags --always "$commit")
version=${version//\//-}
name=TacticalFrog-$version
if [ -n "$(git -C "$repo" status --porcelain --ignore-submodules=dirty)" ]; then
  echo "Warning: uncommitted changes will not be in the release" >&2
fi

work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
src=$work/src
dest=$work/$name
mkdir -p "$src" "$dest/ros_install" "$out"

git -C "$repo" -c tar.umask=022 archive "$commit" raspbian modelling www LICENSE requirements.txt ROS2/tactical_frog |
  tar -x -C "$src"
cp -a "$src"/{raspbian,modelling,www,LICENSE,requirements.txt} "$dest/"

# clean environment, so nothing from the workspace, ~/.colcon/defaults.yaml or distcc gets into the build
env -i HOME="$HOME" PATH=/usr/local/bin:/usr/bin:/bin COLCON_DEFAULTS_FILE=/dev/null bash -c '
  set -e
  work=$1
  source /opt/ros/jazzy/setup.bash
  cd "$work"
  colcon --log-base "$work/log" build \
    --base-paths "$work/src/ROS2/tactical_frog" \
    --build-base "$work/build" \
    --install-base "$work/install" \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
' build "$work"
cp -a "$work/install/tactical_frog" "$dest/ros_install/"

printf '%s %s\n' "$version" "$commit" > "$dest/VERSION"

tarball=$out/$name-$(uname -m).tar.gz
tar -C "$work" --owner=0 --group=0 -czf "$tarball.part" "$name"
mv "$tarball.part" "$tarball"
echo "Built $tarball ($(du -h "$tarball" | cut -f1))"
