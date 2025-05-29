#!/usr/bin/env python3
import argparse
import numpy as np
from sensor_msgs.msg import CameraInfo, Image
import rospy
import rosbag
from cv_bridge import CvBridge
import cv2

parser = argparse.ArgumentParser(description="A program to add CameraInfo structures to a rosbag")
parser.add_argument('-o', '--output_file', help="File for the new bag data to be stored within", required=True)
parser.add_argument('-b', '--bag', help="Bag file to search for images", required=True)
parser.add_argument('-c', '--camera_info', help="topic with camera_info", required=True)
parser.add_argument('-t', '--topic', help="topic with image_data", required=True)
opts = parser.parse_args()

def get_mapping(ci: CameraInfo):
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
    return 1/ norms
    

with rosbag.Bag(opts.bag,"r") as in_bag:
    with rosbag.Bag(opts.output_file, "w") as out_bag:
        caminfo_reader = in_bag.read_messages(topics=[opts.camera_info])
        for _, msg, _ in caminfo_reader:
            mapping = get_mapping(msg)
            break
        general_reader = in_bag.read_messages()
        bridge = CvBridge()
        for topic, msg, t in general_reader:
            if topic==opts.topic:
                img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                img  = img * mapping
                header = msg.header
                msg = bridge.cv2_to_imgmsg(img, encoding='passthrough')
            out_bag.write(topic,msg,t)
