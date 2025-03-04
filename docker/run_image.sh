#!/usr/bin/env bash
echo "Finding camera numbers"
~/Cyclops/ROS/link_cameras.py
echo "Starting TOF camera"
source ~/Cyclops/.venv/bin/activate
~/Cyclops/ROS/tof_reader.py &
echo "Starting docker instance"
docker run --network host --privileged --ipc="host" -v /home/pi/:/home/cyclops/ -v /var/run/dbus:/var/run/dbus -v /var/run/avahi-daemon/socket:/var/run/avahi-daemon/socket test:latest
