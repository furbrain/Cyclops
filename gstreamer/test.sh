#!/usr/bin/env bash
source /home/pi/Cyclops/.venv/bin/activate
export GST_PLUGIN_PATH=$GST_PLUGIN_PATH:$PWD
export GST_DEBUG=python:6
IMU_STREAM="py_imu_data  ! video/x-raw,framerate=30/1 ! queue ! mux."
TOF_STREAM="py_TOF_Cam ! video/x-raw,width=960,height=240,framerate=30/1,format=I420 ! queue ! mux."
AUDIO_STREAM="alsasrc device=hw:0 ! queue ! audioconvert ! vorbisenc ! queue ! mux."
VIDEO_STREAM="v4l2src device=\"/dev/video3\" ! video/x-h264,width=1280,height=720 ! h264parse ! queue ! mux."
MUXER="matroskamux name=mux ! filesink location=\"/home/pi/combined.mkv\""
gst-launch-1.0 $IMU_STREAM $TOF_STREAM $AUDIO_STREAM $VIDEO_STREAM $MUXER
