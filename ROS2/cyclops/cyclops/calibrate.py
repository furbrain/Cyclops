import cv2
import numpy as np

import cv_bridge

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from sensor_msgs.srv import SetCameraInfo
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image, CameraInfo
from cyclops_interfaces.action import Calibrate
import time

class Calibrator(Node):

    def __init__(self):
        super().__init__('calibrate')
        self.pub = self.create_publisher(Image, "markers", 10)
        self.sub = self.create_subscription(Image, "image_raw", self.callback, 10)
        self.cal_action = ActionServer(self, Calibrate, "complete_calibration", self.complete_calibration)
        self.srv = self.create_service(Trigger, 'start_calibration', self.start_calibration)
        self.ci_client = self.create_client(SetCameraInfo, "set_camera_info")
        self.bridge = cv_bridge.CvBridge()
        self.declare_parameter("target_size","4x11")
        self.declare_parameter("target_type","circle")
        self.calibrating = False
        self.corners = []
        self.shape = None
        # For circles grid → apply spacing correction
        self.objp = np.zeros((4 * 11, 1, 3), np.float32)
        for i in range(11):
            for j in range(4):
                self.objp[i * 4 + j, 0, 0] = j * 1.0 + 0.5 * (i % 2)
                self.objp[i * 4 + j, 0, 1] = i * (np.sqrt(3) / 2)
        self.ts = 0

    def start_calibration(self, request: Trigger.Request, response: Trigger.Response):
        response.success = True
        response.message = "Calibration Started"
        self.calibrating = True
        return response

    def complete_calibration(self, goal_handle: rclpy.action.server.ServerGoalHandle):
        print("starting calibration")
        result = Calibrate.Result()
        if not self.calibrating:
            goal_handle.abort()
            result.message = "Calibration not yet started"
            return result
        if (len(self.corners)) < 8:
            goal_handle.abort()
            result.message = f"Not enough images ({len(self.corners)}), need at least 8"
            return result
        goal_handle.publish_feedback(Calibrate.Feedback(interim_message="Calibrating..."))
        objpoints = [self.objp] * len(self.corners)
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, self.corners, self.shape, None, None)
        if ret:
            p, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, self.shape, 0.5)
            p_3x4 = np.zeros((3,4))
            p_3x4[:3,:3] = p #convert to 3x4 matrix
            ci = CameraInfo(
                height = self.shape[0],
                width = self.shape[1],
                distortion_model = "plumb bob",
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
                goal_handle.succeed()
                result.success = True
                result.message = "Calibration Completed"
            else:
                goal_handle.abort()
                result.success = False
                result.message = ci_result.status_message
        else:
            goal_handle.abort()
            result.message = "Calibration Failed"
        self.calibrating = False
        self.corners = []
        return result

    def callback(self, msg:Image):
        if self.pub.get_subscription_count()>0:
            size = [int(x) for x in self.get_parameter("target_size").get_parameter_value().string_value.split("x")]
            target_type = self.get_parameter("target_type").get_parameter_value().string_value
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
            self.shape = img.shape[::-1]
            if target_type=="circle":
                ret, corners = cv2.findCirclesGrid(img, size, flags=cv2.CALIB_CB_ASYMMETRIC_GRID + cv2.CALIB_CB_CLUSTERING)
            else:
                raise ValueError("Type must be circle")
            out_img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            if ret:
                out_img = cv2.drawChessboardCorners(out_img, size, corners, ret)
                if self.calibrating and time.time() > (self.ts+0.5): #add corners every 0.5 seconds
                    self.ts = time.time()
                    self.corners.append(corners)
            img = self.bridge.cv2_to_imgmsg(out_img, header=msg.header, encoding="rgb8")
            self.pub.publish(img)

def main(args=None):
    rclpy.init(args=args)
    cal = Calibrator()
    rclpy.spin(cal)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
