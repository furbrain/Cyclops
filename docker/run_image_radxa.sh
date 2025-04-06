#!/usr/bin/env bash
echo "Starting docker instance"
docker run --network host --privileged --ipc="host" -v /home/radxa:/home/cyclops -v /home/radxa/bags:/bags -v /var/run/dbus:/var/run/dbus -v /var/run/avahi-daemon/socket:/var/run/avahi-daemon/socket test:latest
