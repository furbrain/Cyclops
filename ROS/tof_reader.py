#!/usr/bin/env python3
#TOF SERVER
from multiprocessing import shared_memory
from ArducamDepthCamera import (ArducamCamera, TOFConnect, TOFDeviceType, 
                                TOFOutput, TOFControl, DepthData, ArducamInfo)
import numpy as np
from numpy_shares import NumpyShareManager
from find_camera2 import find_cams


    

def get_arducam():
    tof = ArducamCamera()
    ret = tof.open(TOFConnect.CSI, find_cams("csi")[0])
    if not ret:
        print("Failed to open camera. Error code:", ret)
        return
    ret = tof.start(TOFOutput.DEPTH)
    if ret != 0:
        print("Failed to start camera. Error code:", ret)
        tof.close()
        return
    info = tof.getCameraInfo()
    if info.device_type == TOFDeviceType.HQVGA:
        tof.setControl(TOFControl.RANGE, 4)
    return tof, info

with NumpyShareManager() as nsm:
    cam, info = get_arducam()
    print("pointcloud publisher start")
    try:
        temp_frame = np.zeros((info.height,info.width),dtype=np.float32)
        shared_depth, depth_lock = nsm.create_numpy_share(temp_frame,"depth")
        shared_amplitude, amp_lock = nsm.create_numpy_share(temp_frame,"amplitude")
        print(info.width, info.height)

        while True:
            frame = cam.requestFrame(200)
            if frame is not None and isinstance(frame, DepthData):
                depth_buf = frame.getDepthData()
                confidence_buf = frame.getConfidenceData()
                with amp_lock:
                    shared_amplitude[:] = frame.getAmplitudeData()                
                depth_buf[confidence_buf < 30] = 0
    
                # Convert depth values ​​from millimeters to meters
                z = depth_buf / 1000.0
                z[z <= 0] = np.nan  # Handling invalid depth values
                with depth_lock:
                    shared_depth[:] = z            
                cam.releaseFrame(frame)
    finally:
        cam.stop()
        cam.close()

