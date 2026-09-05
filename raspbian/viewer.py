#!/usr/bin/env python3
from typing import List, Dict

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

from orb_slam3_py import Leg
from survey import SVXParser
from atlas_tools import get_station_symbol


def _get_point32_from_value(values, key):
    pos = Point()
    pos.x, pos.y, pos.z = values.atPoint3(key)
    return pos


class CaveViewer(rclpy.node.Node):
    def __init__(self, fname: str):
        super().__init__('cave_viewer')
        self.point_pub = self.create_publisher(PointCloud2, '/points', 10)
        self.pose_pub = self.create_publisher(PoseArray, '/pose', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/marker', 10)
        self.timer = self.create_timer(1, self.callback)
        self.markers = MarkerArray()
        hdr = Header()
        hdr.frame_id = "map"
        hdr.stamp = self.get_clock().now().to_msg()
        with open(fname) as f:
            values = gtsam.Values()
            values.deserialize(f.read())
        points: List[np.ndarray] = []
        kfs: List[gtsam.Pose3] = []
        survey_points: Dict[int, Point] = {}
        aruco_points: Dict[int, Point] = {}
        for key in values.keys():
            symbol = gtsam.Symbol(key)
            if symbol.string().startswith("m"):
                points.append(values.atPoint3(key))
            elif symbol.string().startswith("k"):
                kfs.append(values.atPose3(key))
            elif symbol.string().startswith("s"):
                survey_points[key] = _get_point32_from_value(values, key)
            elif symbol.string().startswith("a"):
                aruco_points[key] = _get_point32_from_value(values, key)
        all_survey_points = survey_points | aruco_points
        points = np.array(points)
        print(points.shape)
        self.points = point_cloud2.create_cloud_xyz32(hdr, points.astype(np.float32))
        self.poses = PoseArray()
        self.poses.header = hdr
        for kf in kfs:
            p = Pose()
            p.position.x, p.position.y, p.position.z = kf.translation()
            q = kf.rotation().toQuaternion() # scipy order: x,y,z,w
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q.x(), q.y(), q.z(), q.w()
            self.poses.poses.append(p)
        lines = Marker()
        lines.header = hdr
        lines.id = 1
        lines.scale.x = 0.1
        lines.type = Marker.LINE_LIST
        for leg in SVXParser().parse_file("/footage/storrs_aruco_4/survey.svx"):
             lines.points.append(all_survey_points[get_station_symbol(leg.from_station_id)])
             lines.points.append(all_survey_points[get_station_symbol(leg.to_station_id)])
        lines.color = ColorRGBA(g=1.0, a=1.0)
        self.markers.markers.append(lines)

        survey_spheres = Marker()
        survey_spheres.header = hdr
        survey_spheres.id = 2
        survey_spheres.scale.x = 0.2
        survey_spheres.scale.y = 0.2
        survey_spheres.scale.z = 0.2
        survey_spheres.type = Marker.SPHERE_LIST
        survey_spheres.color = ColorRGBA(b=1.0, a=1.0)
        survey_spheres.points.extend(list(survey_points.values()))
        self.markers.markers.append(survey_spheres)

        aruco_spheres = Marker()
        aruco_spheres.header = hdr
        aruco_spheres.id = 3
        aruco_spheres.scale.x = 0.2
        aruco_spheres.scale.y = 0.2
        aruco_spheres.scale.z = 0.2
        aruco_spheres.type = Marker.SPHERE_LIST
        aruco_spheres.color = ColorRGBA(r=1.0, g=1.0, a=1.0)
        aruco_spheres.points.extend(list(aruco_points.values()))
        self.markers.markers.append(aruco_spheres)

        print("starting")

    def callback(self):
        self.point_pub.publish(self.points)
        self.pose_pub.publish(self.poses)
        self.marker_pub.publish(self.markers)


rclpy.init()
viewer = CaveViewer("/data/values.gtsam")
rclpy.spin(viewer)