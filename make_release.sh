#!/bin/bash
# Build a TacticalFrog release:  ./make_release.sh [git ref, default HEAD] [output dir, default dist/]
#
# Run it on a Pi with the same OS and ROS as the devices it's for, as it compiles the tactical_frog interfaces.
# (Pushing a tag does the same on GitHub, see .github/workflows/release.yml.) RELEASE_VERSION overrides the version.
# Only committed files are included. The tarball holds one folder, TacticalFrog-<version>, which is what
# /opt/TacticalFrog/current should link to:
#   raspbian/ modelling/ www/     as in the repo
#   ros_install/tactical_frog/    the built ROS package, loaded by raspbian/ros_env.sh
#   VERSION
#   RELEASES                      the orb_slam3 and OpenMVS releases that go with it (see below)
# orb_slam3 and OpenMVS aren't included, they have releases of their own. RELEASES names the ones this was built
# and tested with: the commits of the submodules, and their release tags, which devices update to along with
# this (raspbian/tacticalfrog_update.py). So release those first. With REQUIRE_PARTNER_RELEASES=1, as for a
# published release, it stops if either has no release; otherwise its tag is "-" and devices won't update to it.
set -euo pipefail
umask 022

repo=$(cd "$(dirname "$0")" && pwd)
ref=${1:-HEAD}
out=${2:-$repo/dist}

commit=$(git -C "$repo" rev-parse --verify "$ref^{commit}")
version=${RELEASE_VERSION:-$(git -C "$repo" describe --tags --always "$commit")}
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

# partner <name> <submodule> <repository>: "<name> <commit> <release tag>"
partner() {
  local sub tag
  sub=$(git -C "$repo" rev-parse "$commit:$2")
  tag=$(git ls-remote --tags "https://github.com/$3.git" |
    awk -v c="$sub" '$1 == c { t = $2; sub("^refs/tags/", "", t); sub("\\^\\{\\}$", "", t); print t }' |
    sort -uV | tail -n 1)
  if [ -z "$tag" ]; then
    if [ "${REQUIRE_PARTNER_RELEASES:-0}" = 1 ]; then
      echo "$1 $sub has no release: tag $3 at that commit first" >&2
      exit 1
    fi
    echo "Warning: $1 $sub has no release, so devices won't update to this one by themselves" >&2
    tag=-
  fi
  echo "$1 $sub $tag"
}
{
  echo "# <name> <commit> <release tag>: what this release was built with, and updates install with it"
  partner orb_slam3 ROS2/orb_slam3_ros furbrain/orb_slam3_ros
  partner OpenMVS openMVS furbrain/openMVS
} > "$dest/RELEASES"

tarball=$out/$name-$(uname -m).tar.gz
tar -C "$work" --owner=0 --group=0 -czf "$tarball.part" "$name"
mv "$tarball.part" "$tarball"
echo "Built $tarball ($(du -h "$tarball" | cut -f1))"
