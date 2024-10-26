#!/usr/bin/sh

#DEMUXER="tcpclientsrc host=cyclops.local port=8888 ! queue ! matroskademux name=demux"
DEMUXER="udpsrc name=demux"
RAW_CAPS="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)RAW, sampling=(string)RGB, depth=(string)8, width=(string)480, height=(string)180"
QUEUE="queue max-size-buffers=1 leaky=downstream"
#DEMUXER="udpsrc  port=8888 caps=\"$CAPS\" name=demux"
#DEMUXER="tcpclientsrc host=cyclops.local port=8888 ! $CAPS ! rtpstreamdepay name=demux"
CAM="demux. ! application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96 ! rtph264depay ! avdec_h264 ! $QUEUE ! autovideosink"
#TOF="demux. ! rtpvrawdepay ! video/x-raw,width=480,height=180,framerate=15/1,format=RGB ! videoconvert ! queue ! autovideosink"
TOF="demux. ! rtpvrawdepay ! videoconvert ! $QUEUE ! autovideosink"
gst-launch-1.0 -v $DEMUXER caps=\"$RAW_CAPS\" port=8000 $TOF &
gst-launch-1.0 -v $DEMUXER port=8001 $CAM 

