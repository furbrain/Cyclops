#!/usr/bin/env python3
from typing import List, Dict

import argparse
import gtsam
import numpy as np
import scipy.spatial.transform as sst

import rclpy
import rclpy.node
from std_msgs.msg import Header, ColorRGBA
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseArray, Pose, Point
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import MarkerArray, Marker

import orb_slam3_py
from orb_slam3_py import Leg
from survey import SVXParser


def _get_point32_from_value(values, key):
    pos = Point()
    pos.x, pos.y, pos.z = values.atPoint3(key)
    return pos


class CaveViewer(rclpy.node.Node):
    def __init__(self, fname: str, all_maps = False, align=True):
        super().__init__('cave_viewer')
        self.point_pub = self.create_publisher(PointCloud2, '/points', 10)
        self.pose_pub = self.create_publisher(PoseArray, '/pose', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/marker', 10)
        self.timer = self.create_timer(1, self.callback)
        self.markers = MarkerArray()
        atlas = orb_slam3_py.load_atlas(str(fname), binary=False)
        if align:
            orb_slam3_py.align_atlas(atlas)
        if all_maps:
            kfs = atlas.get_all_keyframes()
            mps = atlas.get_all_map_points()
        else:
            m0: orb_slam3_py.Map
            m0 = list(sorted(atlas.get_all_maps(), key = lambda m: m.keyframes_in_map()))[-1]
            kfs = m0.get_all_keyframes()
            mps = m0.get_all_map_points()
        hdr = Header()
        hdr.frame_id = "map"
        hdr.stamp = self.get_clock().now().to_msg()
        points: List[np.ndarray] = [m.get_world_pos() for m in mps]
        poses: List[gtsam.Pose3] = [kf.get_pose().inv() for kf in kfs]
        points = np.array(points)
        print(points.shape)
        self.points = point_cloud2.create_cloud_xyz32(hdr, points.astype(np.float32))
        self.poses = PoseArray()
        self.poses.header = hdr
        pose: sst.RigidTransform
        for pose in poses:
            p = Pose()
            p.position.x, p.position.y, p.position.z = pose.translation
            q = pose.rotation.as_quat()
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
            self.poses.poses.append(p)

    def callback(self):
        self.point_pub.publish(self.points)
        self.pose_pub.publish(self.poses)
        # self.marker_pub.publish(self.markers)

parser = argparse.ArgumentParser(description="Create a colmap from a bag (ROS1 or ROS2)")
parser.add_argument('-a', '--atlas', help="provide an atlas message file to use", required=True)
parser.add_argument('--maps', '-m', help="show all maps", action="store_true")
opts = parser.parse_args()

rclpy.init()
viewer = CaveViewer(opts.atlas, opts.maps)
rclpy.spin(viewer)