#!/usr/bin/env bash
set -e
exec "$(dirname "$0")/ros_env.sh" /usr/bin/gunicorn -b 127.0.0.1:5000 app:app --reload --worker-class gevent
