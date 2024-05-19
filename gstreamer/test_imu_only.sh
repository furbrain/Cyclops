#!/usr/bin/env bash
export GST_PLUGIN_PATH=$GST_PLUGIN_PATH:$PWD
export GST_DEBUG=python:6
gst-launch-1.0 \
    py_imu_data num_buffers=800 ! video/x-raw,framerate=100/1 ! videorate ! matroskamux name=mux ! filesink location="imu_only.mkv"
