#!/usr/bin/env python3

from math import tan, pi
import rospy
from sensor_msgs.msg import PointCloud2, PointField, Image
from std_msgs.msg import Float32MultiArray, Header
from gst_camera.cam_info_mgr import CameraInfoManager
import numpy as np
import struct
from threading import Thread
from numpy_shares import NumpyShareManager
import ctypes
import cv2
import time


#copied from sensor_msgs_py (which is only in ROS2)

_DATATYPES = {}
_DATATYPES[PointField.INT8]    = ('b', 1)
_DATATYPES[PointField.UINT8]   = ('B', 1)
_DATATYPES[PointField.INT16]   = ('h', 2)
_DATATYPES[PointField.UINT16]  = ('H', 2)
_DATATYPES[PointField.INT32]   = ('i', 4)
_DATATYPES[PointField.UINT32]  = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)

def _get_struct_fmt(is_bigendian, fields, field_names=None):
    fmt = '>' if is_bigendian else '<'
    offset = 0
    for field in (f for f in sorted(fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
        if offset < field.offset:
             fmt += 'x' * (field.offset - offset)
             offset = field.offset
        if field.datatype not in _DATATYPES:
             print('Skipping unknown PointField datatype [%d]' % field.datatype, file=sys.stderr)
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt    += field.count * datatype_fmt
            offset += field.count * datatype_length
    return fmt
    
def create_cloud(header, fields, points):
     """
     Create a L{sensor_msgs.msg.PointCloud2} message.
  
     @param header: The point cloud header.
     @type  header: L{std_msgs.msg.Header}
     @param fields: The point cloud fields.
     @type  fields: iterable of L{sensor_msgs.msg.PointField}
     @param points: The point cloud points.
     @type  points: list of iterables, i.e. one iterable for each point, with the
                    elements of each iterable being the values of the fields for 
                    that point (in the same order as the fields parameter)
     @return: The point cloud.
     @rtype:  L{sensor_msgs.msg.PointCloud2}
     """
  
     cloud_struct = struct.Struct(_get_struct_fmt(False, fields))
  
     #buff = ctypes.create_string_buffer(cloud_struct.size * len(points))
  
     #point_step, pack_into = cloud_struct.size, cloud_struct.pack_into
     #offset = 0
     #for p in points:
     #    pack_into(buff, offset, *p)
     #    offset += point_step
  
     return PointCloud2(header=header,
                        height=1,
                        width=len(points),
                        is_dense=False,
                        is_bigendian=False,
                        fields=fields,
                        point_step=cloud_struct.size,
                        row_step=cloud_struct.size * len(points),
                        data=points.astype(np.float32).tobytes())
  
def create_cloud_xyz32(header, points):
     """
     Create a L{sensor_msgs.msg.PointCloud2} message with 3 float32 fields (x, y, z).
  
     @param header: The point cloud header.
     @type  header: L{std_msgs.msg.Header}
     @param points: The point cloud points.
     @type  points: iterable
     @return: The point cloud.
     @rtype:  L{sensor_msgs.msg.PointCloud2}
     """
     fields = [PointField('x', 0, PointField.FLOAT32, 1),
               PointField('y', 4, PointField.FLOAT32, 1),
               PointField('z', 8, PointField.FLOAT32, 1)]
     return create_cloud(header, fields, points)


class TOFPublisher:

    def __init__(self, nsm: NumpyShareManager):
        self.camera_info = CameraInfoManager(rospy.get_name())
        self.camera_info.loadCameraInfo()
        self.frame_id = self.camera_info.frame
        self.depth_data, self.depth_lock = nsm.get_numpy_share("depth")
        self.amp_data, self.amp_lock = nsm.get_numpy_share("amplitude")
        self.width = self.depth_data.shape[1]
        self.height = self.depth_data.shape[0]
        self.pointsize_ = self.width * self.height
        self.timer_ = rospy.Rate(10)
        self.publisher_ = rospy.Publisher("~point_cloud",PointCloud2, queue_size=10)
        self.publisher_image = rospy.Publisher("~image_raw",Image, queue_size=10)
        #may need to fix this bit later...
        self.fx = self.width / (2 * tan(0.5 * pi * 64.3 / 180))
        self.fy = self.height / (2 * tan(0.5 * pi * 50.4 / 180))
        self.header = Header()
        self.header.frame_id = self.frame_id
        self.points = None
        
    def run(self):
        while not rospy.is_shutdown():
            start = time.time()
            with self.depth_lock:
                z = self.depth_data.copy()
            # Calculate x and y coordinates
            u = np.arange(self.width)
            v = np.arange(self.height)
            u, v = np.meshgrid(u, v)

            # Calculate point cloud coordinates
            x = (u - self.width / 2) * z / self.fx
            y = (v - self.height / 2) * z / self.fy

            # Combined point cloud
            points = np.stack((x, y, z), axis=-1)
            self.points = points[~np.isnan(points).any(axis=-1)]  # Filter invalid points
            self.header.stamp = rospy.Time.now()
            print("starting cloud", time.time()-start)
            pc2_msg_ = create_cloud_xyz32(self.header, self.points)
            print("finished cloud", time.time()-start)
            
            image_msg = Image(header=self.header, encoding="mono8",width=self.width, height=self.height, step=self.width*2)
            with self.amp_lock:
                normed = cv2.normalize(self.amp_data, None, 0, 256, cv2.NORM_MINMAX, dtype=cv2.CV_8U)                
            image_msg.data = cv2.equalizeHist(normed).tobytes()
            print("equalised", time.time()-start)

            self.publisher_.publish(pc2_msg_)
            self.publisher_image.publish(image_msg)
            print("published", time.time()-start)
            self.timer_.sleep()


def main(args = None):
    with NumpyShareManager() as nsm:
        rospy.init_node("TOF")
        print("node created")
        print("pointcloud publisher start")
        tof_publisher = TOFPublisher(nsm)
        tof_publisher.run()

if __name__ == "__main__":
    main()
