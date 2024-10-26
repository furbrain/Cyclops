#!/usr/bin/env bash
source /home/pi/Cyclops/.venv/bin/activate
export GST_PLUGIN_PATH=$GST_PLUGIN_PATH:$PWD
#export GST_DEBUG=GST_TRACER:7
#export GST_DEBUG=python:4
#export GST_TRACERS="cpuusage;proctime;queuelevel;buffer"
unset GST_SHARK_LOCATION
#export GST_SHARK_FILE_BUFFERING=0
#get cameras
read TOF_DEVICE USB_DEVICE < <(./find_cameras.py)
#QUEUE="queue"
#FILE_QUEUE="queue !"
#IMU_STREAM="py_imu_data  ! video/x-raw,framerate=30/1 ! $QUEUE name=q_imu ! mux."
#TOF_STREAM="py_TOF_Cam device=$TOF_DEVICE absolute=true ! video/x-raw,width=480,height=180,framerate=15/1,format=GRAY8 ! $QUEUE name=q_tof ! mux."
#AUDIO_STREAM="alsasrc device=default:CARD=Camera ! audio/x-raw,format=F32LE,rate=8000 ! queue !  avenc_aac ! $QUEUE name=q_audio ! mux."
#VIDEO_STREAM="v4l2src device=/dev/video$USB_DEVICE ! video/x-h264,width=1280,height=720 ! h264parse ! $QUEUE name=q_vid ! mux."
#MUXER="matroskamux name=mux ! $FILE_QUEUE filesink location=\"/home/pi/cal.mkv\""
gst-launch-1.0 py_TOFD_Cam device=$TOF_DEVICE ! video/x-raw,width=240,format=GRAY8 ! pngenc snapshot=true ! filesink location="snapshot.png"
