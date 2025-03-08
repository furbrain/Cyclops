#!/usr/bin/env python3
import os
from collections import defaultdict
import rospy
import rosbag
from cv_bridge import CvBridge
from pathlib import Path
import cv2
import re


# parameters: bag file
# keyframes file

BAG_FILE = "/home/phil/bags/couch.bag"
COLMAP_DIR = Path("/home/phil/footage/couch2/")
TOPIC = "/camera/image_raw"


def grouper(iterable, n):
    "Collect data into non-overlapping fixed-length chunks or blocks."
    iterators = [iter(iterable)] * n
    return zip(*iterators)

for directory in "images", "sparse":
    os.makedirs(COLMAP_DIR / directory, exist_ok=True)

with open(COLMAP_DIR / "sparse" / "images.txt", "r") as images_file, \
        open(COLMAP_DIR / "sparse" / "points3D.txt", "r") as points_file:
    matches = defaultdict(list)
    image_id = -1
    for line in images_file:
        match = re.search(r"(\d+).png", line)
        if match:
            image_id = int(line.split()[0])
            print(f"Reading image: {image_id}")
        else:
            for x, y, idx in grouper(line.split(),3):
                matches[image_id].append(int(idx))
    for line in points_file:
        items = line.split()
        point_id = int(items[0])
        print(f"Matching point: {point_id}")
        
        for image_id,idx in grouper(items[8:], 2):
            idx = int(idx)
            image_id = int(image_id)
            if idx < 0:
                print(f"Negative idx!: {image_id}, {idx}")
            if idx > len(matches[image_id]):
                print(f"IDX out of range: {image_id}, {idx}, {len(matches[image_id])}")
            if matches[image_id][idx] != point_id:
                print(f"point mismatch: {point_id}, {image_id}, {idx}, {matches[image_id][idx]}")
