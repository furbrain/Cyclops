#!/usr/bin/env bash
source /home/pi/Cyclops/.venv/bin/activate
export GST_PLUGIN_PATH=$GST_PLUGIN_PATH:$PWD
export GST_DEBUG=python:6
# this is currently set to lossless encoding  - qp values are zero
gst-launch-1.0 \
    py_TOF_Cam ! video/x-raw,width=960,height=240,framerate=30/1,format=I420 ! matroskamux ! filesink location="/home/pi/tmp.mkv"

#    v4l2h264enc extra-controls="controls,video_bitrate_mode=0,h264_minimum_qp_value=0,h264_maximum_qp_value=0,h264_i_frame_period=30,h264_profile=4,h264_level=14;" !\
#    video/x-h264,level="(string)5" ! h264parse !\
#    matroskamux ! filesink location="/home/pi/tmp.mkv"
