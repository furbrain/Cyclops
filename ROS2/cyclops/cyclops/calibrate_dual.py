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

class CalibratorDual(CalNode):

    def __init__(self):
        super().__init__('calibrate')
        self.ns_a = self.get_initial_param("ns_a","a")
        self.ns_b = self.get_initial_param("ns_b","b")
        self.image_a = Subscriber(self, Image, f"{self.ns_a}/image_raw")
        self.image_b = Subscriber(self, Image, f"{self.ns_b}/image_raw")
        self.ci_a = self.create_subscription(CameraInfo, f"{self.ns_a}/camera_info",
                                             lambda x: self.store_ci("a", x), 1)
        self.ci_b = self.create_subscription(CameraInfo, f"{self.ns_b}/camera_info",
                                             lambda x: self.store_ci("b", x), 1)
        self.synced_imgs = ApproximateTimeSynchronizer([self.image_a, self.image_b], 10, 0.03)
        self.synced_imgs.registerCallback(self.images_in)
        self.dynamic_tf2_client = self.create_client(SetTransform, "set_transform")
        self.corners_a = []
        self.corners_b = []
        # For circles grid → apply spacing correction
        self.ts = time.time()
        self.camera_infos: Dict[str, Optional[CameraInfo]]= {"a":None, "b": None}

    def store_ci(self,channel: str, msg:CameraInfo):
        self.camera_infos[channel] = msg

    def on_start_calibration(self):
        pass

    def on_finish_calibration(self, result):
        dist_a, k_a, p_a = camera_info_to_cv2(self.camera_infos['a'])
        dist_b, k_b, p_b = camera_info_to_cv2(self.camera_infos['b'])
        objects = [self.get_object_corners()] * len(self.corners_a)
        self.get_logger().warn(f"{len(self.corners_a)}")
        cal_res = cv2.stereoCalibrate(objects, self.corners_a, self.corners_b,
                                      k_a, dist_a,
                                      k_b, dist_b,
                                      (0, 0), flags=cv2.CALIB_FIX_INTRINSIC)
        rotation = sst.Rotation.from_matrix(cal_res[5])
        translation = cal_res[6][:,0]
        transform = Transform(translation=vector3_from_array(translation),
                              rotation=quat_from_sst(rotation))
        self.get_logger().info(f"{transform}")
        self.get_logger().info(f"{rotation.as_quat()}")
        self.get_logger().info(f"{translation}, {cal_res[5]}")
        req = SetTransform.Request(transform=transform)
        future = self.dynamic_tf2_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        tf_result: SetTransform.Response = future.result()
        result.success = tf_result.success
        if tf_result.success:
            result.message = "Calibration complete"
        else:
            result_message = "Error storing transform"

    def images_in(self, msg_a: Image, msg_b: Image):
        if self.calibrating:
            now = time.time()
            if now > self.ts + 0.5:
                img_a = self.bridge.imgmsg_to_cv2(msg_a, "mono8")
                img_b = self.bridge.imgmsg_to_cv2(msg_b, "mono8")
                ret_a, corners_a = self.find_corners(img_a)
                if not ret_a:
                    return
                ret_b, corners_b = self.find_corners(img_b)
                if not ret_b:
                    return
                self.ts = now
                self.corners_a.append(corners_a)
                self.corners_b.append(corners_b)
                self.publish_progress(len(self.corners_a))


def main(args=None):
    rclpy.init(args=args)
    cal = CalibratorDual()
    rclpy.spin(cal)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
