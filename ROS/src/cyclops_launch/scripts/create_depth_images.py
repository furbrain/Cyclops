#!/usr/bin/env python3
import os
import argparse
from pathlib import Path
import re
import numpy as np
np.float = np.float64 #workaround for bug in ros_numpy
from ros_numpy import numpify, msgify
from sensor_msgs.msg import PointCloud2
import rospy
import rosbag
from cv_bridge import CvBridge
import cv2
import numpy as np 
import yaml
from math import tan, pi
import time

yaml.add_multi_constructor('tag:', lambda loader, suffix, node: None, Loader=yaml.SafeLoader)


# parameters: bag file
# keyframes file

TOPIC = "/lidar/point_cloud"
WIDTH = 240
HEIGHT = 180
Fx = WIDTH / (2 * tan(0.5 * pi * 64.3 / 180))
Fy = HEIGHT / (2 * tan(0.5 * pi * 50.4 / 180))

parser = argparse.ArgumentParser(description="A program to convert pointclouds back to depthimages")
parser.add_argument('-o', '--output_file', help="File for the new bag data to be stored within", required=True)
parser.add_argument('-b', '--bag', help="Bag file to search for images", required=True)
opts = parser.parse_args()

    
with rosbag.Bag(opts.bag,"r") as in_bag:
    with rosbag.Bag(opts.output_file, "w") as out_bag:
        message_reader = in_bag.read_messages()
        bridge = CvBridge()
        for topic, msg, t in message_reader:
            if topic==TOPIC:
                dct = {x: getattr(msg,x) for x in msg.__slots__}
                msg2 = PointCloud2(**dct)
                data = numpify(msg2)
                u =data['x']*Fx/data['z'] + WIDTH/2
                v =data['y']*Fy/data['z'] + HEIGHT/2
                u = np.round(u).astype(np.uint16)
                v = np.round(v).astype(np.uint16)
                img = np.full((180,240),np.nan,dtype=np.float32)
                img[v,u] = data['z']
                out_msg = bridge.cv2_to_imgmsg(img)
                out_msg.header = msg.header
                out_bag.write('/lidar/depth_raw',out_msg, t)
            out_bag.write(topic, msg, t)

