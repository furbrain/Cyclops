#!/usr/bin/env python3
import os
import argparse
from pathlib import Path
import re
import numpy as np
np.float = np.float64 #workaround for bug in ros_numpy
from ros_numpy import numpify, msgify
from sensor_msgs.msg import CameraInfo
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

parser = argparse.ArgumentParser(description="A program to add CameraInfo structures to a rosbag")
parser.add_argument('-o', '--output_file', help="File for the new bag data to be stored within", required=True)
parser.add_argument('-b', '--bag', help="Bag file to search for images", required=True)
parser.add_argument('-c', '--config', help="COnfig file to serch for camerainfo data", required=True)
parser.add_argument('conversions', nargs="*")
opts = parser.parse_args()

conversions = [y.split(":=") for y in opts.conversions]
conversions = {x[0]:x[1] for x in conversions}

def get_caminfos(opts):
    with open(opts.config,"r") as config_file:
        cams_config = yaml.safe_load(config_file)
    caminfos = {}
    for cam in cams_config.values():
        topic = cam['rostopic']
        if topic in conversions:
            topic = conversions[topic]
        caminfos[topic] = get_caminfo_from_yaml(cam)
    return caminfos


def get_caminfo_from_yaml(dct):
    ci = CameraInfo()
    ci.height = dct['resolution'][1]
    ci.width = dct['resolution'][0]
    ci.distortion_model="plumb_bob"
    ci.D = dct['distortion_coeffs']+[0]
    fx, fy, cx, cy = dct['intrinsics']
    ci.K = [fx, 0, cx,
            0, fy, cy,
            0,  0, 1]
    ci.R = [1, 0, 0,
            0, 1, 0,
            0, 0, 1]
    ci.P = [fx, 0, cx, 0,
            0, fy, cy, 0,
            0,  0, 1,  0]
    return ci

def get_ci_topic(topic: str):
    last_slash = topic.rfind("/")
    ci_topic = topic[:last_slash+1]+'camera_info'
    if ci_topic in conversions:
        return conversions[ci_topic]
    else:
        return ci_topic
    
            
caminfos = get_caminfos(opts)    
with rosbag.Bag(opts.bag,"r") as in_bag:
    with rosbag.Bag(opts.output_file, "w") as out_bag:
        message_reader = in_bag.read_messages()
        bridge = CvBridge()
        for topic, msg, t in message_reader:
            if topic in caminfos:
                ci = caminfos[topic]
                ci.header = msg.header
                out_bag.write(get_ci_topic(topic), ci, t)
            out_bag.write(topic, msg, t)

