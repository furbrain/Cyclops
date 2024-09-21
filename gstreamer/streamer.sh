#!/usr/bin/env bash
gst-launch-1.0 v4l2src device="/dev/video3" ! video/x-h264,width=1280 ! h264parse !  rtph264pay config-interval=1 pt=96 ! gdppay ! tcpserversink host=192.168.1.119 port=5000
