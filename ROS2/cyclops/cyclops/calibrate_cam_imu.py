import shutil
import signal
import subprocess
from inspect import cleandoc
from pathlib import Path
from typing import Optional

import rclpy
from sensor_msgs.msg import CameraInfo
from cyclops_interfaces.srv import SetTransform

from .utils import CalNode, get_ros_home

class CalibratorCamIMU(CalNode):

    def __init__(self):
        super().__init__('calibrate')

        # set up parameters
        ns_cam = self.get_initial_param("ns_cam","", description="Namespace for the camera")
        ns_imu = self.get_initial_param("ns_imu","", description="Namespace for the IMU")
        self.cam_topic = self.get_initial_param("cam_topic", "image_raw", description="image topic")
        self.caminfo_topic = self.get_initial_param("camerainfo_topic", "camera_info", description="camera info topic")
        self.imu_topic = self.get_initial_param("imu_topic", "imu_rect", description="IMU topic")
        imu_noise_url = self.get_initial_param("imu_noise_url", "imu_noise.yaml",
                                                    description="YAML file that describes the imu noise density. "
                                                                "Defaults to .ros/imu/{imu_topic}.yaml")
        cal_dir: str = self.get_initial_param("cal_dir", "cal", description="Name to store bag as")

        # set up topics
        if not self.imu_topic.startswith("/"):
            self.imu_topic = ns_imu + '/' + self.imu_topic
        if not self.cam_topic.startswith("/"):
            self.cam_topic = ns_cam + '/' + self.cam_topic
        if not self.caminfo_topic.startswith("/"):
            self.caminfo_topic = ns_cam + '/' + self.caminfo_topic
        self.create_subscription(CameraInfo,self.caminfo_topic, self.caminfo_received, 1)

        #set up directories
        ros_home = get_ros_home()
        if cal_dir.startswith("/"):
            self.cal_dir = Path(cal_dir)
        else:
            self.cal_dir = ros_home / cal_dir
        if imu_noise_url=="":
            self.imu_noise_url = ros_home / "imu" / (self.imu_topic.strip('/') + ".yaml")
        elif imu_noise_url.startswith("/"):
            self.imu_noise_url = Path(imu_noise_url)
        else:
            self.imu_noise_url = self.cal_dir / imu_noise_url
        self.get_logger().info(f"imu_noise_url: {self.imu_noise_url}")
        self.dynamic_tf2_client = self.create_client(SetTransform, "set_transform")
        self.process: Optional[subprocess.Popen] = None
        self.camera_info: Optional[CameraInfo] = None
        self.kalibr_temp_dir: Optional[Path] = None

    def caminfo_received(self, camera_info: CameraInfo):
        self.camera_info = camera_info

    def on_start_calibration(self):

        topics = f"{self.cam_topic} {self.caminfo_topic} {self.imu_topic}"
        self.process = subprocess.Popen(f"exec ros2 bag record -o {self.cal_dir / 'ros2.bag'} {topics}",
                                        shell=True)

    def on_finish_calibration(self, result):
        self.process.send_signal(signal.SIGINT)
        try:
            self.process.wait(5.0)
        except TimeoutError:
            result.success = False
            result.message = "Bag recording did not stop in good time"
            return

        #convert ros2 bag to ros1 bag
        #subprocess.run(['rosbags-convert',
        #                '--src',self.cal_dir / 'ros2.bag',
        #                '--dst',self.cal_dir / 'ros1.bag'])

        #create various yaml files:
        with open(self.cal_dir / "target.yaml", "w") as f:
            if self.target_type=="circle":
                f.write(cleandoc(f"""
                    target_type: circlegrid  # gridtype
                    targetCols: {self.target_shape[0]}  # number of circles (cols)
                    targetRows: {self.target_shape[1]}  # number of circles (rows)
                    spacingMeters: {self.target_size}  # distance between circles [m]
                    asymmetricGrid: True  # use asymmetric grid (opencv) [bool]
                """))
        with open(self.cal_dir / "cam.yaml", "w") as f:
            ci = self.camera_info
            models  = {'plumb bob': 'radtan', 'equidistant': 'equi'}
            f.write(cleandoc(f"""
                cam0:
                  camera_model: pinhole
                  intrinsics: [{ci.k[0]}, {ci.k[4]}, {ci.k[2]}, {ci.k[5]}]
                  distortion_model: {models[ci.distortion_model]}
                  distortion_coeffs: {list(ci.d[:4])}
                  rostopic: {self.cam_topic}
                  resolution: [{ci.width}, {ci.height}]
            """))
        shutil.copy(self.imu_noise_url, self.cal_dir / "imu.yaml") # copy the imu params

        # run kalibr in docker
        subprocess.run(['docker', 'run', '-v', f"{self.cal_dir}:/data", 'furbrain/kalibr:latest',
                        'rosrun', 'kalibr', 'kalibr_calibrate_imu_camera',
                        '--bag', '/data/ros1.bag',
                        '--target', '/data/target.yaml',
                        '--cam', '/data/cam.yaml',
                        '--imu', '/data/imu.yaml',
                        '--max-iter', '10',
                        '--dont-show-report',
                        ])

        # convert bag to rosbag1
        # run kalibr in docker...
        result.success = True
        result.message = "interim results done"
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
