from typing import Optional

import yaml
from scipy.spatial.transform import Rotation, RigidTransform
import numpy as np
import rclpy
import rclpy.time
from geometry_msgs.msg import Transform
from sensor_msgs.msg import CameraInfo
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import ament_index_python as aip

from .utils import SmartNode

class OrbSetter(SmartNode):
    def __init__(self):
        super().__init__("OrbSetter")
        self.create_subscription(CameraInfo,"/left/camera_info", self.set_left_caminfo, 2)
        self.create_subscription(CameraInfo,"/right/camera_info", self.set_right_caminfo, 2)
        self.left_cam_frame_id = self.get_initial_param("left_camera_frame_id", "left")
        self.right_cam_frame_id = self.get_initial_param("right_camera_frame_id", "right")
        self.imu_frame_id = self.get_initial_param("imu_frame_id", "imu")
        self.imu_noise_url = self.get_url_from_param("imu_noise_url", "cam_imu/imu.yaml")
        self.settings_url = self.get_url_from_param("settings_url", "orb_slam3.yaml")
        self.tf_buffer = Buffer(cache_time=rclpy.time.Duration(seconds=3.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.left_ci: Optional[CameraInfo] = None
        self.right_ci: Optional[CameraInfo] = None

    def set_left_caminfo(self, ci: CameraInfo):
        self.left_ci = ci

    def set_right_caminfo(self, ci: CameraInfo):
        self.right_ci = ci


    def get_transform(self, t):
        translation = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
        q = t.transform.rotation
        rotation = Rotation.from_quat([q.x, q.y, q.z, q.w])
        transform = RigidTransform.from_components(translation, rotation)
        return transform

    def run(self):
        while self.left_ci is None or self.right_ci is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        # Also wait for TF frames to be available before looking up
        while not self.tf_buffer.can_transform(
                self.imu_frame_id, self.left_cam_frame_id, rclpy.time.Time()):
            rclpy.spin_once(self, timeout_sec=0.1)

        while not self.tf_buffer.can_transform(
                self.right_cam_frame_id, self.left_cam_frame_id, rclpy.time.Time()):
            rclpy.spin_once(self, timeout_sec=0.1)

        imu_left_t = self.tf_buffer.lookup_transform(self.imu_frame_id,
                                            self.left_cam_frame_id,
                                            rclpy.time.Time())
        imu_left = self.get_transform(imu_left_t)

        right_left_t = self.tf_buffer.lookup_transform(self.right_cam_frame_id,
                                            self.left_cam_frame_id,
                                            rclpy.time.Time())
        right_left = self.get_transform(right_left_t)


        with open(self.imu_noise_url) as f:
            imu_noise_data = yaml.safe_load(f)
        self.get_logger().info(f"Writing ORB_SLAM3 settings to {self.settings_url}")
        config_file = aip.get_package_share_path("cyclops") / "config" / "orb_slam3.yaml.tpl"
        with open(config_file) as f:
            template = f.read()
        left_ci = self.left_ci
        right_ci = self.right_ci
        baseline = np.linalg.norm(right_left.translation)
        output = eval(f'f"""{template}"""') # nested so text within template treated as f-string
        with open(self.settings_url,"w") as f:
            f.write(output)

def main(args=None):
    rclpy.init(args=args)
    node = OrbSetter()
    node.run()
