import time

import cv2
import numpy as np

import rclpy
from cyclops_interfaces.action import Calibrate
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.srv import SetCameraInfo
from .utils import CalNode


class Calibrator(CalNode):

    def __init__(self):
        super().__init__('calibrate')
        self.pub = self.create_publisher(Image, "markers", 10)
        self.sub = self.create_subscription(Image, "image_raw", self.callback, 10)
        self.ci_client = self.create_client(SetCameraInfo, "set_camera_info")
        self.corners = []
        self.shape = None
        self.ts = 0

    def on_start_calibration(self):
        self.corners = []

    def on_finish_calibration(self, result: Calibrate.Result):
        if (len(self.corners)) < 8:
            result.success = False
            result.message = f"Not enough images ({len(self.corners)}), need at least 8"
            return
        objpoints = [self.get_object_corners()] * len(self.corners)
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, self.corners, self.shape, None, None)
        if ret:
            p, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, self.shape, 0.5)
            p_3x4 = np.zeros((3,4))
            p_3x4[:3,:3] = p #convert to 3x4 matrix
            ci = CameraInfo(
                width = self.shape[0],
                height = self.shape[1],
                distortion_model = "plumb_bob",
                d = dist[0],
                k = mtx.flatten(),
                r = np.eye(3).flatten(),
                p = p_3x4.flatten()
            )
            req = SetCameraInfo.Request(camera_info=ci)
            future = self.ci_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            ci_result: SetCameraInfo.Response = future.result()
            if ci_result.success:
                result.success = True
                result.message = "Calibration Completed"
            else:
                result.success = False
                result.message = "SetCameraInfo failed with: " + ci_result.status_message
        else:
            result.success = False
            result.message = "Calibration Failed"

    def callback(self, msg:Image):
        if self.pub.get_subscription_count()>0:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
            self.shape = img.shape[::-1]
            ret, corners = self.find_corners(img)
            if ret:
                out_img = self.draw_corners(img, corners, ret)
                if self.calibrating and time.time() > (self.ts+0.5): #add corners every 0.5 seconds
                    self.ts = time.time()
                    self.corners.append(corners)
                    self.publish_progress(len(self.corners))
            else:
                out_img = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
            img = self.bridge.cv2_to_imgmsg(out_img, header=msg.header, encoding="rgb8")
            self.pub.publish(img)

def main(args=None):
    rclpy.init(args=args)
    cal = Calibrator()
    rclpy.spin(cal)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
