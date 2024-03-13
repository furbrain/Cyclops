#!/usr/bin/env bash
export GST_PLUGIN_PATH=$GST_PLUGIN_PATH:$PWD
export GST_DEBUG=python:6
gst-launch-1.0 \
    libcamerasrc ! video/x-raw,width=640,height=480,framerate=10/1,format=YUY2,interlace-mode=progressive,colorimetry=bt709 ! \
    v4l2h264enc extra-controls="controls,video_bitrate_mode=0,h264_minimum_qp_value=35,h264_maximum_qp_value=35,h264_i_frame_period=30,h264_profile=0,h264_level=11;" ! video/x-h264,level="(string)4" ! h264parse ! queue ! matroskamux name=mux ! filesink location="combined.mkv" \
    py_imu_data  ! video/x-raw,framerate=50/1 ! queue ! mux.
