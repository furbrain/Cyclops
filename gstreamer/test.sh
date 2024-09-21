#!/usr/bin/env bash
source /home/pi/Cyclops/.venv/bin/activate
export GST_PLUGIN_PATH=$GST_PLUGIN_PATH:$PWD
export GST_DEBUG=GST_TRACER:7
export GST_TRACERS="cpuusage;proctime;queuelevel;buffer"
unset GST_SHARK_LOCATION
export GST_SHARK_FILE_BUFFERING=0
#get cameras
read TOF_DEVICE USB_DEVICE < <(./find_cameras.py)

IMU_STREAM="py_imu_data  ! video/x-raw,framerate=30/1 ! queue name=q_imu ! mux."
TOF_STREAM="py_TOF_Cam device=$TOF_DEVICE ! video/x-raw,width=480,height=180,framerate=30/1,format=GRAY8 ! queue name=q_tof ! mux."
AUDIO_STREAM="alsasrc device=default:CARD=Camera ! queue ! audioconvert ! speexenc ! queue name=q_audio ! mux."
VIDEO_STREAM="v4l2src device=/dev/video$USB_DEVICE ! video/x-h264,width=1280,height=720 ! h264parse ! queue name=q_vid ! mux."
MUXER="matroskamux name=mux ! filesink location=\"/home/pi/combined.mkv\""
gst-launch-1.0 $IMU_STREAM $TOF_STREAM $AUDIO_STREAM $VIDEO_STREAM $MUXER
