#!/usr/bin/env bash
source /home/pi/Cyclops/.venv/bin/activate
export GST_PLUGIN_PATH=$GST_PLUGIN_PATH:$PWD
export GST_DEBUG=python:6
MUXER="matroskamux name=mux ! filesink location=\"/home/pi/cam.mkv\""
AUDIO_STREAM="alsasrc device=hw:0 ! queue ! audioconvert ! avenc_aac ! queue ! mux."
VIDEO_STREAM="v4l2src device=\"/dev/video3\" ! video/x-h264,width=1280,height=720 ! h264parse ! queue ! mux."
gst-launch-1.0 $AUDIO_STREAM $VIDEO_STREAM $MUXER
