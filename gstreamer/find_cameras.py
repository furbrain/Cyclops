#!/usr/bin/env python3

##find cameras
import sys

from linuxpy.video.device import iter_video_capture_devices, PixelFormat

tof_device = None
usb_device = None

for dev in iter_video_capture_devices():
    with dev:
        if dev.info.driver == "unicam":
            tof_device = dev.index
        elif dev.info.driver == "uvcvideo":
            for fmt in dev.info.formats:
                if fmt.pixel_format == PixelFormat.H264:
                    usb_device = dev.index
if tof_device is None:
    print("No TOF camera found!",file=sys.stderr)
elif usb_device is None:
    print("No USB camera found!",file=sys.stderr)
else:
    print(f"{tof_device} {usb_device}")
    
