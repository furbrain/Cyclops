#!/usr/bin/env python3
import argparse
import numpy as np
from sensor_msgs.msg import CameraInfo, Image
import message_filters
import rospy
from cv_bridge import CvBridge
import cv2

class FillDepthNode:
    def __init__(self):
        self.sub = rospy.Subscriber("in", Image, self.callback, queue_size=5)
        self.pub = rospy.Publisher("out", Image, queue_size=5)
        self.bridge = CvBridge()
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))


    def callback(self, img_msg: Image):
        img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
        if img.dtype != np.float32:
            img = img.astype(np.float32) / 1000.0
        else:
            img = img.copy()
            img[np.isnan(img)] = 0
        img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, self.kernel)
        header = img_msg.header
        msg = self.bridge.cv2_to_imgmsg(img, encoding='32FC1')
        msg.header = header
        self.pub.publish(msg)
        
if __name__=="__main__":
    rospy.init_node("radial_node")
    FillDepthNode()
    rospy.spin()
