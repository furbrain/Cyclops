from typing import Optional

import cv2
import numpy as np
import scipy.spatial.transform as sst

import rclpy
import rclpy.time
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Transform
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from .utils import SmartNode

class RegisterNode(SmartNode):
    def __init__(self):
        super().__init__('register')
        self.pub = self.create_publisher(Image, "depth_registered", 10)
        self.im_sub = self.create_subscription(Image, "depth_raw", self.im_callback, 10)
        self.ci_sub = self.create_subscription(CameraInfo, "camera_info", self.ci_callback, 10)
        self.ci_rgb_sub = self.create_subscription(CameraInfo, "rgb_camera_info", self.rgb_callback, 10)
        self.radial = self.get_initial_param("radial", False)
        self.ci: Optional[CameraInfo] = None
        self.rgb_ci: Optional[CameraInfo] = None
        self.transform: Optional[Transform] = None
        self.depth_projection: Optional[np.ndarray] = None
        self.cv_bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def get_transform(self):
        if self.transform:
            return self.transform
        if self.ci and self.rgb_ci:
            try:
                t = self.tf_buffer.lookup_transform(
                    self.rgb_ci.header.frame_id,
                    self.ci.header.frame_id,
                    rclpy.time.Time())
            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform {self.rgb_ci.header.frame_id,} to {self.ci.header.frame_id}: {ex}')
                return None
            self.get_logger().info(f"Transform found: {t}")
            self.transform = t.transform
            return self.transform
        return None


    def ci_equal(self, a: CameraInfo, b: CameraInfo) -> bool:
        """Compare two CameraInfo and return whether they are effectively identical. Ignores header"""
        if a is None or b is None:
            return False
        for attr in "height", "width", "distortion_model", "d":
            if getattr(a, attr) != getattr(b, attr):
                return False
        for attr in ("k", ):
            if any(getattr(a, attr) != getattr(b, attr)):
                return False
        return True

    def get_depth_perspective(self):
        k = np.array(self.ci.k).reshape((3,3))
        d = np.array(self.ci.d)
        grid = np.mgrid[0:self.ci.height, 0:self.ci.width]
        points = np.dstack((grid[1],grid[0])).reshape((-1,1,2)).astype(np.float32)
        coords = cv2.undistortPoints(points, k, d)
        z = np.ones_like(coords[:, :, 0])
        self.depth_projection = np.dstack((coords, z)).reshape((-1,3))
        if self.radial:
            norm = np.linalg.norm(self.depth_projection,axis=1)
            self.depth_projection = self.depth_projection / norm.reshape((-1,1))
        self.get_logger().info(str(self.depth_projection.shape))


    def ci_callback(self, ci:CameraInfo):
        if not self.ci_equal(self.ci,ci):
            self.ci = ci
            self.get_transform()
            self.get_depth_perspective()

    def rgb_callback(self, ci: CameraInfo):
        if self.rgb_ci != ci:
            self.rgb_ci = ci
            self.get_transform()


    def im_callback(self, im: Image):
        t = self.get_transform()
        if t is None:
            return
        k = np.array(self.rgb_ci.k).reshape((3,3))
        d = np.array(self.rgb_ci.d)
        depths = self.cv_bridge.imgmsg_to_cv2(im).reshape((-1,1))
        xyz = (self.depth_projection * depths).reshape((-1,3)).astype(np.float32)
        quat = sst.Rotation.from_quat([self.transform.rotation.x,
                                       self.transform.rotation.y,
                                       self.transform.rotation.z,
                                       self.transform.rotation.w])
        translation = np.array([self.transform.translation.x,self.transform.translation.y,self.transform.translation.z]).astype(np.float32)
        mat = quat.inv().as_matrix()
        rod, _ = cv2.Rodrigues(mat)
        #xyz_rgb = (mat.astype(np.float32) @ xyz.T).T + translation
        xyz_rgb, jac = cv2.projectPoints(xyz,rod,translation,k,None, jacobian=None)
        xys = np.rint(xyz_rgb).reshape((-1,2))
        zs = xyz[...,2]
        img = np.zeros((self.rgb_ci.height, self.rgb_ci.width)).astype(np.float32)
        valid_xys = (0 <= xys[:,1]) & (xys[:,1] < self.rgb_ci.width) & (0 <= xys[:,0]) & (xys[:,0] < self.rgb_ci.height)
        xs = xys[valid_xys,1].astype("int64")
        ys = xys[valid_xys,0].astype("int64")
        zs = zs[valid_xys]
        img[ys, xs] = zs
        msg = self.cv_bridge.cv2_to_imgmsg(img,encoding="32FC1")
        msg.header = im.header
        msg.header.frame_id = self.rgb_ci.header.frame_id
        self.pub.publish(msg)
        #self.get_logger().info(f"{xys}")

def main(args=None):
    rclpy.init(args=args)
    reg = RegisterNode()
    rclpy.spin(reg)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
