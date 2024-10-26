#!/usr/bin/env bash
source /home/pi/Cyclops/.venv/bin/activate
export GST_PLUGIN_PATH=$GST_PLUGIN_PATH:$PWD
#get cameras
read TOF_DEVICE USB_DEVICE < <(./find_cameras.py)

QUEUE="queue2"
DQUEUE="queue max-size-buffers=1 leaky=downstream"
RTP_RAW="videoconvert ! video/x-raw,format=RGB ! rtpvrawpay !"
RTP_H264="rtph264pay !"

TOF_STREAM="py_TOF_Cam device=$TOF_DEVICE absolute=true ! video/x-raw,width=480,height=180,framerate=15/1,format=GRAY8 ! $DQUEUE ! $RTP_RAW mux."
VIDEO_STREAM="v4l2src device=/dev/video$USB_DEVICE ! video/x-h264,width=1280,height=720 ! h264parse config-interval=1 ! $QUEUE ! $RTP_H264 mux."

MUXER="udpsink host=192.168.1.124 name=mux sync=false"
gst-launch-1.0 -v $TOF_STREAM $MUXER port=8000 &
gst-launch-1.0 -v $VIDEO_STREAM $MUXER port=8001


