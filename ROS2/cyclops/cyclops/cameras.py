import time
from typing import List

import rclpy
from camera_info_manager import CameraInfoManager
import picamera2
import cv_bridge
from sensor_msgs.msg import Image, CameraInfo


class CameraNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("Camera")
        self.info_mgr = CameraInfoManager(self,"rgb")
        self.info_mgr.loadCameraInfo()
        self.pub = self.create_publisher(Image,"image_raw", 2)
        self.ci_pub = self.create_publisher(CameraInfo,"camera_info", 2)
        cam_list = picamera2.Picamera2.global_camera_info()
        self.cam: picamera2.Picamera2 = None
        for cam in cam_list:
            self.cam = picamera2.Picamera2(cam['Num'])
            if self.cam.sensor_modes[0]['unpacked'].startswith("SRG"):
                break
        else:
            print("No RGB camera found, aborting!")
            exit()
        config = self.cam.create_still_configuration(main={"format": 'XRGB8888'})
        self.cam.configure(config)
        self.cam.start()
        self.cvbridge = cv_bridge.CvBridge()

    def run(self):
        while True:
            rclpy.spin_once(self, timeout_sec=0.01)
            im = self.cam.capture_array()
            shape = im.shape[:2]
            tl = [x//4 for x in shape]
            br = [3*x for x in tl]
            cropped = im[320:860, 480:1440]
            msg = self.cvbridge.cv2_to_imgmsg(cropped[:,:,:3], encoding="bgr8")
            self.pub.publish(msg)
            self.ci_pub.publish(self.info_mgr.getCameraInfo())

def main():
    rclpy.init()
    node = CameraNode()
    node.run()

if __name__=="__main__":
    main()