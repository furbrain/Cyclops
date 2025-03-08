#!/usr/bin/env python3
import argparse
import numpy as np
from sensor_msgs.msg import CameraInfo, Image
from gst_camera.cam_info_mgr import CameraInfoManager
import rospy
import rosbag
from cv_bridge import CvBridge
import cv2
import yaml

class AddCameraInfo:
    def __init__(self):
        self.sub = rospy.Subscriber("image_raw", Image, self.callback, queue_size=5)
        cam_name = rospy.get_param("~name")
        self.cam_info = CameraInfoManager(cam_name)
        self.cam_info.loadCameraInfo()
        
    def callback(self, imgmsg: Image):
        header = imgmsg.header
        self.cam_info.publish(header)

rospy.init_node("caminfo_adder")
AddCameraInfo()
rospy.spin()
