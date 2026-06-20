#!/usr/bin/bash
#setup x for using cuda
source /opt/ros/foxy/setup.bash
XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

# --- argument check ---------------------------------------------------------
if [[ $# -ne 1 ]]; then
    echo "Usage: $(basename "$0") <name>" >&2
    exit 1
fi
ARG="$1"
 
# Basic sanity-check: reject empty strings or anything with path separators
# to avoid accidentally constructing a dangerous remote path.
if [[ -z "$ARG" || "$ARG" == *"/"* || "$ARG" == *".."* ]]; then
    echo "Error: argument must be a plain name with no slashes or '..'" >&2
    exit 1
fi

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
# copy recoding from pi
scp -r pi@cm5.local:/data/trips/$ARG/recording .
scp pi@cm5.local:.ros/transforms/left_right.yaml .
cp $SCRIPT_DIR/../openMVS/make_laptop.sh ./make_rough.sh
chmod 755 make_rough.sh
$SCRIPT_DIR/make_model.py -d .
