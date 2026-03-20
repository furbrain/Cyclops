#!/usr/bin/env bash
set -e
source /home/pi/ros2_ws/install/setup.bash
exec /usr/bin/gunicorn -b 127.0.0.1:5000 app:app --reload
