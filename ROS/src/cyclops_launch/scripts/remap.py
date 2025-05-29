#!/usr/bin/env python3
import argparse
import numpy as np
from sensor_msgs.msg import CameraInfo, Image
import message_filters
import rospy
from cv_bridge import CvBridge
import cv2

class RadialToDepthNode:
    def __init__(self):
        self.paired_sub = message_filters.ApproximateTimeSynchronizer(
            (message_filters.Subscriber("depth/radial", Image),
            message_filters.Subscriber("camera_info", CameraInfo)),
            queue_size=10, slop=0.01)
        self.paired_sub.registerCallback(self.paired_callback)
        self.pub = rospy.Publisher("depth/image_raw", Image, queue_size=5)
        self.last_K = None
        self.last_D = None
        self.mapping = None
        self.bridge = CvBridge()

    def update_mapping(self, ci: CameraInfo):
        self.last_K = ci.K
        self.last_D = ci.D
        u = np.arange(ci.width,dtype=np.float64)
        v = np.arange(ci.height,dtype=np.float64)
        K = np.array(ci.K).reshape((3,3))
        D = np.array(ci.D)
        u,v = np.meshgrid(u,v)
        stack = np.dstack((u,v))
        output = cv2.undistortPoints(stack.reshape((-1,2)), K, D)
        output = output.reshape((ci.height, ci.width, 2))
        stack = np.dstack((output, np.ones_like(u)))
        norms = np.linalg.norm(stack, axis=2)
        self.mapping = 1/ norms
        
    def paired_callback(self, img_msg: Image, cam: CameraInfo):
        if self.mapping is None or cam.K != self.last_K or cam.D != self.last_D:
            self.update_mapping(cam)
        img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
        img  = img * self.mapping
        header = img_msg.header
        msg = self.bridge.cv2_to_imgmsg(img.astype(np.float32), encoding='32FC1')
        msg.header = header
        self.pub.publish(msg)
        
if __name__=="__main__":
    rospy.init_node("radial_node")
    RadialToDepthNode()
    rospy.spin()
