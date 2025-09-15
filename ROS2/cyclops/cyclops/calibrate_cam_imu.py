import os
import signal
import subprocess
from pathlib import Path
from typing import Optional, Dict

import cv2
import scipy.spatial.transform as sst

import rclpy
from geometry_msgs.msg import Transform, Vector3, Quaternion
from sensor_msgs.msg import Image, CameraInfo
from cyclops_interfaces.srv import SetTransform
import time
from message_filters import Subscriber, ApproximateTimeSynchronizer

from .utils import CalNode, camera_info_to_cv2, vector3_from_array, quat_from_sst

class CalibratorCamIMU(CalNode):

    def __init__(self):
        super().__init__('calibrate')
        self.ns_cam = self.get_initial_param("ns_cam","", description="Namespace for the camera")
        self.ns_imu = self.get_initial_param("ns_imu","", description="Namespace for the IMU")
        self.cam_topic = self.get_initial_param("img_topic", "image_raw", description="image topic")
        self.caminfo_topic = self.get_initial_param("camerainfo_topic", "camera_info", description="camera info topic")
        self.imu_topic = self.get_initial_param("imu_topic", "imu_rect", description="IMU topic")
        bag_name: str = self.get_initial_param("bag_name", "cam_imu", description="Name to store bag as")
        if bag_name.startswith("/"):
            self.url = Path(bag_name)
        else:
            if "ROS_HOME" in os.environ:
                ros_home = Path(os.environ.get("ROS_HOME"))
            else:
                ros_home = Path(os.environ.get("HOME")) / ".ros"
            self.url = Path(ros_home) / "bags" / bag_name

        self.dynamic_tf2_client = self.create_client(SetTransform, "set_transform")
        self.process: Optional[subprocess.Popen] = None

    def on_start_calibration(self):
        if not self.url.parent.exists():
            self.url.parent.mkdir(parents=True, exist_ok=True)
        topics = f"{self.ns_cam}/{self.cam_topic} {self.ns_cam}/{self.caminfo_topic} {self.ns_imu}/{self.imu_topic}"
        self.process = subprocess.Popen(f"exec ros2 bag record -o {self.url}_r2.bag {topics}",
                                        shell=True)

    def on_finish_calibration(self, result):
        self.process.send_signal(signal.SIGINT)
        try:
            self.process.wait(5.0)
        except TimeoutError:
            result.success = False
            result.message = "Bag recording did not stop in good time"
            return
        subprocess.run(f"rosbags-convert --src {self.url}_r2.bag --dst {self.url}_r1.bag")
        # convert bag to rosbag1
        # run kalibr in docker...
        return # return early for now
        req = SetTransform.Request(transform=transform)
        future = self.dynamic_tf2_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        tf_result: SetTransform.Response = future.result()
        result.success = tf_result.success
        if tf_result.success:
            result.message = "Calibration complete"
        else:
            result_message = "Error storing transform"



def main(args=None):
    rclpy.init(args=args)
    cal = CalibratorCamIMU()
    rclpy.spin(cal)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
