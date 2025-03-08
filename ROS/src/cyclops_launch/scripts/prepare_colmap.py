#!/usr/bin/env python3
import os
import argparse
from pathlib import Path
import re

import rospy
import rosbag
from cv_bridge import CvBridge
import cv2
import numpy as np 
import yaml

yaml.add_multi_constructor('tag:', lambda loader, suffix, node: None, Loader=yaml.SafeLoader)


# parameters: bag file
# keyframes file

BAG_FILE = "/home/phil/bags/couch.bag"
COLMAP_DIR = Path("/home/phil/footage/couch2/")
TOPIC = "/camera/image_raw"
parser = argparse.ArgumentParser(description="A program to populate the images directory of a COLMAP folder from a bagfile")
parser.add_argument('-o', '--output_dir', help="Directory for the COLMAP data to be stored within", required=True)
parser.add_argument('-b', '--bag', help="Bag file to search for images", required=True)
parser.add_argument('-t', '--topic', help="Topic that publishes the images on", default="/camera/image_raw")
parser.add_argument('-c', '--compressed', action='store_true', help="whether to get images from TOPIC/compressed")
parser.add_argument('-i', '--cam_info', help="yaml file to load cam calibration from")
opts = parser.parse_args()
COLMAP_DIR = Path(opts.output_dir)


for directory in "images", "sparse":
    os.makedirs(COLMAP_DIR / directory, exist_ok=True)
    
if opts.cam_info:
    with open(opts.cam_info) as cal_file:
        cam_info = yaml.safe_load(cal_file)
        mtx = {x: cam_info["Camera1."+x] for x in ("fx","fy","cx","cy")}
        cam_matrix = np.array([[mtx['fx'], 0,         mtx['cx']],
                               [0,         mtx['fy'], mtx['cy']],
                               [0,         0,         1        ]])
        dist_coeffs = np.array([cam_info["Camera1."+x] for x in ("k1","k2","p1","p2")])
        cam_size =  (cam_info['Camera.width'], cam_info['Camera.height'])
        undistort_maps = cv2.initUndistortRectifyMap(cam_matrix, dist_coeffs, None, cam_matrix, cam_size, cv2.CV_32FC1)
else:
    cam_info = None

with open(COLMAP_DIR / "sparse" / "images.txt", "r") as images_file, \
        rosbag.Bag(opts.bag,"r") as bag:
    if opts.compressed:
        topic = opts.topic + "/compressed"
    else:
        topic = opts.topic
    message_reader = bag.read_messages(topics=topic)
    bridge = CvBridge()
    timestamps = []
    for line in images_file:
        match = re.search(r"(\d+).png", line)
        if match:
            ts = int(match.group(1))
            timestamps.append(ts)
    timestamps = sorted(timestamps)
    for ts in timestamps:
        ts_time = rospy.Time(ts/1e9)
        for _, msg, _ in message_reader:
            if msg.header.stamp==ts_time:
                if opts.compressed:
                    cv_image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
                else:
                    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                if cam_info:
                    #undistort the image
                    cv_image = cv2.remap(cv_image, *undistort_maps, cv2.INTER_NEAREST)
                out_fname = f"{int(ts)}.png"
                print("matched: ", ts_time)
                cv2.imwrite(str(COLMAP_DIR / "images" / out_fname), cv_image)
                break

