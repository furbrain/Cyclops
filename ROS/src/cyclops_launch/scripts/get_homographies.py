#!/usr/bin/env python3
import argparse
from collections import defaultdict
from typing import Dict, Tuple, List

import numpy as np
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Time, Header
import rospy
import rosbag
from cv_bridge import CvBridge
import cv2

parser = argparse.ArgumentParser(description="A program to calculate best homography transforms between cameras")
parser.add_argument('-b', '--bag', help="Bag file to search for images", default="/home/phil/bags/sipeed_cal.bag")
parser.add_argument('-c', '--cam_topic', help="topic with image_data", default="/rgb/image_raw")
parser.add_argument('-t', '--tof_topic', help="topic with image_data", default="/tof/image_raw")
opts = parser.parse_args()

def read_points_from_bag():
    rgb_frames: Dict[Time, np.ndarray] = {}
    tof_frames: Dict[Time, np.ndarray] = {}
    bridge = CvBridge()
    with rosbag.Bag(opts.bag,"r") as in_bag:
        cam_reader = in_bag.read_messages(topics=[opts.cam_topic])
        tof_reader = in_bag.read_messages(topics=[opts.tof_topic])
        i=0
        j=0
        for topic, msg, _ in tof_reader:
            img = bridge.imgmsg_to_cv2(msg,desired_encoding="16UC1")
            img = cv2.normalize(img,None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8UC1)
            retval, points = cv2.findChessboardCorners(img, (9,6))
            i+=1
            if retval:
                tof_frames[msg.header.stamp] = points
                j+=1
            if i % 20 == 0:
                print(f"{i}: {j}")
        i=0
        j=0
        for topic, msg, _ in cam_reader:
            img = bridge.imgmsg_to_cv2(msg,desired_encoding="rgb8")
            retval, points = cv2.findChessboardCorners(img, (9,6))
            i += 1
            if retval:
                rgb_frames[msg.header.stamp] = points
                j += 1
            if i % 20 == 0:
                print(f"{i}: {j}")
        print(len(rgb_frames))
        print(len(tof_frames))
    tof_idxs = set(tof_frames.keys())
    rgb_idxs = set(rgb_frames.keys())
    joint_idxs = list(tof_idxs.intersection(rgb_idxs))
    rgb_points = np.array([rgb_frames[x] for x in joint_idxs])
    tof_points = np.array([tof_frames[x] for x in joint_idxs])
    np.savez("cal_points.npz",rgb=rgb_points, tof=tof_points)
    return rgb_points, tof_points

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

def read_orb_points_from_bag():
    rgb_frames: Dict[Time, np.ndarray] = {}
    tof_frames: Dict[Time, np.ndarray] = {}
    bridge = CvBridge()
    orb = cv2.ORB.create()
    with rosbag.Bag(opts.bag,"r") as in_bag:
        cam_reader = in_bag.read_messages(topics=[opts.cam_topic])
        tof_reader = in_bag.read_messages(topics=[opts.tof_topic])
        i = 0
        tof_points: Dict[Time, Tuple[List, np.ndarray]] = {}
        rgb_points: Dict[Time, Tuple[List, np.ndarray]] = {}
        tof_imgs: Dict[Time, np.ndarray] = {}
        rgb_imgs: Dict[Time, np.ndarray] = {}
        for topic, msg, _ in tof_reader:
            i+=1
            if i % 10 ==0 and i > 70:
                img = bridge.imgmsg_to_cv2(msg,desired_encoding="16UC1")
                img = equalise_16(img)
                pts, descs = orb.detectAndCompute(img, None)
                tof_points[msg.header.stamp] = (pts, descs)
                tof_imgs[msg.header.stamp] = img
        for _, msg, _ in cam_reader:
            times = tuple(tof_points.keys())
            if msg.header.stamp in times:
                img = bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
                img = img[:,:,0]
                pts, descs = orb.detectAndCompute(img, None)
                rgb_points[msg.header.stamp] = (pts, descs)
                rgb_imgs[msg.header.stamp] = img

        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        for stamp, (tof_pts, tof_desc) in tof_points.items():
            rgb_pts, rgb_desc = rgb_points[stamp]
            matches = bf.match(tof_desc, rgb_desc)
            matches = sorted(matches, key = lambda x:x.distance)
            img3 = cv2.drawMatches(tof_imgs[stamp], tof_pts,
                                   rgb_imgs[stamp], rgb_pts,
                                   matches[:10],None,
                                   flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
            cv2.imshow("main",img3)
            cv2.waitKey(0)



def load_points():
    d = np.load("cal_points.npz")
    return d['rgb'], d['tof']
#read_orb_points_from_bag()
#exit()
#rgb, tof = read_points_from_bag()
rgb, tof = load_points()
rgb = rgb.reshape((-1,2))
tof = tof.reshape((-1,2))
grid = defaultdict(list)
for tof_pt, rgb_pt in zip(tof,rgb):
    ix = int(tof_pt[0]*5)//320
    iy = int(tof_pt[1]*5)//240
    grid[(ix,iy)].append([tof_pt, rgb_pt])
min_count = min(len(x) for x in grid.values())
print(f"smallest cell: {min_count}")
new_grid = {}
for key,val in grid.items():
    idxs = np.random.choice(len(val),min_count,replace=False)
    new_grid[key] = np.array(val)[idxs]
all_points = np.vstack(tuple(new_grid.values()))
print(all_points.shape)
Tx = all_points[:,0,0:1]
Ty = all_points[:,0,1:2]
X = all_points[:,1,0:1]
Y = all_points[:,1,1:2]
XY = X*Y
XX = X*X
YY = Y*Y
ones = np.ones_like(X)
M = np.hstack((ones,X,Y,XY,XX,YY))
print(M.shape)
out = np.hstack((Tx, Ty))
x_parms = np.linalg.lstsq(M,out)
#res = cv2.findHomography(rgb.reshape(-1,1,2), tof.reshape(-1,1,2), method=cv2.RANSAC)
np.save("fancy_homography.npy", x_parms[0])