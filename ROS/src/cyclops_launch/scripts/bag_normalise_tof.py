#!/usr/bin/env python3
import os
import argparse
from pathlib import Path
import re
import numpy as np
np.float = np.float64 #workaround for bug in ros_numpy
import rospy
import rosbag
from cv_bridge import CvBridge
import cv2
import numpy as np 
from math import tan, pi



# parameters: bag file
# keyframes file

TOPIC = "/lidar/point_cloud"
WIDTH = 240
HEIGHT = 180
Fx = WIDTH / (2 * tan(0.5 * pi * 64.3 / 180))
Fy = HEIGHT / (2 * tan(0.5 * pi * 50.4 / 180))

parser = argparse.ArgumentParser(description="A program to add equalised images structures to a rosbag")
parser.add_argument('-o', '--output_file', help="File for the new bag data to be stored within", required=True)
parser.add_argument('-b', '--bag', help="Bag file to search for images", required=True)
opts = parser.parse_args()



def equalise_16(img:np.ndarray):
    flat = img.flatten()
    histogram,_ = np.histogram(img,65536,(0,65535))
    cs = np.cumsum(histogram)
    nj = (cs - cs.min()) * 65535
    N = cs.max() - cs.min()

    # re-normalize the cdf
    cs = nj / N
    cs = cs.astype('uint16') // 257
    img_new = cs[flat].reshape(img.shape)
    return img_new.astype('uint8')

bridge = CvBridge()
with rosbag.Bag(opts.bag,"r") as in_bag:
    with rosbag.Bag(opts.output_file, "w") as out_bag:
        message_reader = in_bag.read_messages()
        bridge = CvBridge()
        for topic, msg, t in message_reader:
            if topic == "/tof/image_raw":
                img = bridge.imgmsg_to_cv2(msg,desired_encoding="passthrough")
                new_img = equalise_16(img)
                new_msg = bridge.cv2_to_imgmsg(new_img,"mono8")
                new_msg.header = msg.header
                out_bag.write("/tof/image_norm_raw",new_msg, t)
            out_bag.write(topic, msg, t)

