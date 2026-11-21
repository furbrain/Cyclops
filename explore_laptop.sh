#!/usr/bin/env bash
docker run -it --gpus all -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics \
     --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
     --volume=/tmp/.docker.xauth:/tmp/.docker.xauth:rw --env=XAUTHORITY=/tmp/.docker.xauth \
     --volume=/home/phil/footage:/footage \
     --env=DISPLAY=unix:1 \
     --ipc=host \
     --shm-size=4gb \
     -it openmvs-ubuntu-cuda
