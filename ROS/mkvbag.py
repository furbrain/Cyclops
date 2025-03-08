#!/usr/bin/env python3
import time
from typing import Sequence, List

import numpy as np

import rospy
from geometry_msgs.msg import Vector3
from reader.gstreader import VidReader, IMUReader, TOFReader, GstReader
from sensor_msgs.msg import Image, Imu, MagneticField
from cv_bridge import CvBridge
from std_msgs.msg import Header

bridge = CvBridge()

def vectorify(data: Sequence):
    return Vector3(*data)

class ROSReader:
    def __init__(self, reader: GstReader, frame_id: str, topic: str=None):
        if topic is None:
            topic = f"/{frame_id}/image_raw"
        self.reader = reader
        self.frame_id = frame_id
        self.last_tm = None
        self.last_frame = None
        self.set_pub(topic)

    def read(self):
        self.last_tm, self.last_frame = self.reader.get_frame()

    def set_pub(self, topic: str):
        self.pub = rospy.Publisher(topic, Image, queue_size=5)

    def publish(self):
        if self.reader.channels==3:
            encoding="rgb8"
        else:
            encoding="mono8"
        msg: Image = bridge.cv2_to_imgmsg(self.last_frame, encoding=encoding)
        msg.header.stamp = rospy.Time(self.last_tm)
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)

class ROSTOFReader(ROSReader):
    def publish(self):
        msg: Image = bridge.cv2_to_imgmsg(self.last_frame.astype(np.float32), encoding="32FC1")
        msg.header.stamp = rospy.Time(self.last_tm)
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)


class ROSIMUReader(ROSReader):
    def set_pub(self, topic: str):
        self.imu_pub = rospy.Publisher(f"/{self.frame_id}/imu_raw", Imu, queue_size=5)
        self.mag_pub = rospy.Publisher(f"/{self.frame_id}/magnetic_raw", MagneticField, queue_size=5)

    def publish(self):
        header = Header(stamp=rospy.Time(self.last_tm), frame_id=self.frame_id)
        mag = vectorify([x*1e-6 for x in self.last_frame[0:3]])
        accel = vectorify(self.last_frame[3:6])
        gyro = vectorify(self.last_frame[6:9])
        imu_msg = Imu(header=header, linear_acceleration=accel, angular_velocity=gyro)
        mag_msg = MagneticField(header=header, magnetic_field=mag)
        self.imu_pub.publish(imu_msg)
        self.mag_pub.publish(mag_msg)

class MKVBag:
    def __init__(self, fname: str):
        # setup readers
        self.cam_reader = ROSReader(VidReader.from_filename(fname), "camera", "camera_mkv/image_raw")
        self.tof_reader = ROSTOFReader(TOFReader.from_filename(fname), "lidar", "lidar/depth_raw")
        self.imu_reader = ROSIMUReader(IMUReader.from_filename(fname), "imu")
        self.readers: List[ROSReader] = [
            self.cam_reader,
            self.tof_reader,
            self.imu_reader
        ]

    def run(self):
        first_offset = None
        lidar_alternator = True
        cam_trigger = False
        for r in self.readers:
            r.read()
        while True:
            if any(x.last_tm is None for x in self.readers):
                break
            min_reader = min(self.readers,key= lambda x: x.last_tm)
            offset = time.time() - min_reader.last_tm
            if first_offset is None:
                first_offset = offset
            elif first_offset > offset:
              time.sleep(first_offset-offset)
            if min_reader is self.tof_reader:
                #lidar_alternator = not lidar_alternator
                if lidar_alternator:
                    cam_trigger = True
                else:
                    min_reader.read()
                    continue
            elif min_reader is self.cam_reader:
                if cam_trigger:
                    cam_trigger = False
                else:
                    min_reader.read()
                    continue
            min_reader.publish()
            min_reader.read()

if __name__=="__main__":
    rospy.init_node("MKVBag")
    bag = MKVBag("/home/phil/footage/storrs/vid.mkv")
    bag.run()
