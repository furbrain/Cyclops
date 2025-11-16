from typing import Optional

import scipy.spatial.transform
import yaml
from scipy.spatial.transform import Rotation, RigidTransform
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
        self.create_subscription(CameraInfo,"/rgb/camera_info", self.set_caminfo, 2)
        self.cam_frame_id = self.get_initial_param("camera_frame_id", "rgb")
        self.imu_frame_id = self.get_initial_param("imu_frame_id", "imu")
        self.imu_noise_url = self.get_url_from_param("imu_noise_url", "cam_imu/imu.yaml")
        self.settings_url = self.get_url_from_param("settings_url", "orb_slam3.yaml")
        self.tf_buffer = Buffer(cache_time=rclpy.time.Duration(seconds=3.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.ci: Optional[CameraInfo] = None

    def set_caminfo(self, ci: CameraInfo):
        self.ci = ci



    def run(self):
        while self.ci is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        t = self.tf_buffer.lookup_transform(self.imu_frame_id,
                                            self.cam_frame_id,
                                            rclpy.time.Time(),
                                            rclpy.time.Duration(seconds=5))
        # write settings file here...
        translation = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
        q = t.transform.rotation
        rotation = Rotation.from_quat([q.x, q.y, q.z, q.w])
        transform = RigidTransform.from_components(translation, rotation)

        with open(self.imu_noise_url) as f:
            imu_noise_data = yaml.safe_load(f)
        self.get_logger().info(f"Writing ORB_SLAM3 settings to {self.settings_url}")
        config_file = aip.get_package_share_path("cyclops") / "config" / "orb_slam3.yaml.tpl"
        with open(config_file) as f:
            template = f.read()
        rgb_ci = self.ci

        output = eval(f'f"""{template}"""') # nested so text within template treated as f-string
        with open(self.settings_url,"w") as f:
            f.write(output)

def main(args=None):
    rclpy.init(args=args)
    node = OrbSetter()
    node.run()
