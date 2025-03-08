#!/usr/bin/env python3
import argparse
import numpy as np
from sensor_msgs.msg import CameraInfo, Image
import message_filters
import rospy
from cv_bridge import CvBridge
import cv2

class MyResize:
    def __init__(self):
        self.sub = rospy.Subscriber("in", Image, self.callback, queue_size=5)
        self.pub = rospy.Publisher("out", Image, queue_size=5)
        self.bridge = CvBridge()

    def callback(self, imgmsg: Image):
        img = self.bridge.imgmsg_to_cv2(imgmsg, desired_encoding='passthrough')
        img = img[:, 159:1119]
        img = cv2.resize(img, (640,480))
        pubmsg = self.bridge.cv2_to_imgmsg(img, encoding="rgb8")
        pubmsg.header = imgmsg.header
        self.pub.publish(pubmsg)
        
if __name__=="__main__":
    rospy.init_node("my_resize")
    MyResize()
    rospy.spin()
