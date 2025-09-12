import time
from typing import List, Tuple, Optional

import cv2
import numpy as np

import rclpy
from camera_info_manager import CameraInfoManager
import picamera2
import cv_bridge
from sensor_msgs.msg import Image, CameraInfo

from .utils import SmartNode

class CameraNode(SmartNode):
    def __init__(self):
        super().__init__("Camera")
        self.info_mgr = CameraInfoManager(self,"rgb")
        self.info_mgr.loadCameraInfo()
        self.pub = self.create_publisher(Image,"image_raw", 2)
        self.ci_pub = self.create_publisher(CameraInfo,"camera_info", 2)
        crop_x = self.get_initial_param("crop_x",0)
        crop_y = self.get_initial_param("crop_y", 0)
        width = self.get_initial_param("width", 0)
        height = self.get_initial_param("height", 0)
        self.resize_width = self.get_initial_param("resize_width", 0)


        cam_list = picamera2.Picamera2.global_camera_info()
        for cam in cam_list:
            self.cam: picamera2.Picamera2 = picamera2.Picamera2(cam['Num'])
            if self.cam.sensor_modes[0]['unpacked'].startswith("SRG"):
                break
        else:
            print("No RGB camera found, aborting!")
            exit()
        config = self.cam.create_still_configuration(main={"format": 'XRGB8888'})
        self.cam.configure(config)
        self.cam.start()
        # setting up cropping area as a slice
        if not all(x==0 for x in (crop_x, crop_y, width, height)):
            self.crop = slice
            if width<=0:
                x_end = None
            else:
                x_end = crop_x + width
            if height<=0:
                y_end = None
            else:
                y_end = crop_y + height
            self.crop: Optional[Tuple[slice]] = np.s_[crop_y:y_end, crop_x:x_end]
        else:
            self.crop = None
        self.bridge = cv_bridge.CvBridge()

    def run(self):
        while True:
            rclpy.spin_once(self, timeout_sec=0.01)
            with self.cam.captured_request() as req:
                im = req.make_array("main")
                if self.crop:
                    im = im[self.crop]
                if self.resize_width:
                    resize_height = (im.shape[0]*self.resize_width)//im.shape[1]
                    im = cv2.resize(im, (self.resize_width, resize_height),interpolation=cv2.INTER_NEAREST)
                msg = self.bridge.cv2_to_imgmsg(im[:,:,:3], encoding="bgr8")
                msg.header.stamp = self.time_from_ns(req.get_metadata()['SensorTimestamp']).to_msg()
            self.pub.publish(msg)
            self.ci_pub.publish(self.info_mgr.getCameraInfo())

def main():
    rclpy.init()
    node = CameraNode()
    node.run()

if __name__=="__main__":
    main()