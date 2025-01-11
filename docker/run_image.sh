#!/bin/sh
docker run --network host --name rosie -it --privileged --ipc="host" -v /home/pi/:/home/cyclops/ test:latest
