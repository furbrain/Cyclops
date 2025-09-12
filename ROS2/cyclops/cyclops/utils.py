from typing import Any, Optional, Tuple
import numpy as np
import time
import cv2
import scipy.spatial.transform

import builtin_interfaces.msg
import rclpy
import rclpy.time
from cv_bridge import CvBridge
from cyclops_interfaces.action import Calibrate
from geometry_msgs.msg import Vector3, Quaternion
from rcl_interfaces.msg import ParameterDescriptor
from rclpy import Parameter, Future
from rclpy.action.server import ServerGoalHandle, ActionServer
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from std_srvs.srv import Trigger

def vector3_from_array(arr):
    return Vector3(x=arr[0], y=arr[1], z=arr[2])

def quat_from_sst(rot: scipy.spatial.transform.Rotation):
    quat = rot.as_quat()
    return Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

def camera_info_to_cv2(ci: CameraInfo) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    dist = np.array(ci.d)
    k = ci.k.reshape((3,3))
    p = ci.p.reshape((3,4))
    return dist, k, p


class SmartNode(Node):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.get_time_offset()

    def get_param_value(self, name: str):
        param: Parameter = self.get_parameter(name)
        return param.value

    def get_initial_param(self, name: str, default: Any = None, description: str = None, **kwargs):
        """Same args as Node.declare_parameter, but returns the actual parameter result"""
        if description is not None:
            description = ParameterDescriptor(description=description)
        elif "descriptor" in kwargs:
            description = kwargs.pop("descriptor")
        self.declare_parameter(name, default, descriptor=description, **kwargs)
        return self.get_param_value(name)

    def time_from_ns(self, ns: int) -> rclpy.time.Time:
        """
        Convert a time from `time.monotonic_ns()` to a ROS2 time in the correct time basis
        :param int ns: Time to convert in nanoseconds
          (can get this from time.monotonic_ns() or a picmaera2 request)
        :return: ROS2 Time instance
        """
        return self._time_from_ns_int(ns) + self.offset

    def get_time_offset(self):
        """
        Update current time ROS->system offset as used by `time_from_ns`
        :return:
        """
        now_ns1 = time.monotonic_ns()
        now_ros = self.get_clock().now()
        now_ns2 = time.monotonic_ns()
        now_ns = (now_ns1 + now_ns2) // 2
        ns_time = self._time_from_ns_int(now_ns)
        self.offset = now_ros - ns_time

    def _time_from_ns_int(self, now_ns: int) -> rclpy.time.Time:
        time_msg = builtin_interfaces.msg.Time(sec=now_ns // 1e9, nanosec=now_ns % 1e9)
        return rclpy.time.Time.from_msg(time_msg)


class CalNode(SmartNode):
    """
    This is a SmartNode that understands calibration grids, and creates a do_calibration action,
    and a finalise calibration service
    """
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._calibration_complete: Optional[Future] = None
        self.calibrating = False # this is true when we are running a calibration
        self.gh: Optional[ServerGoalHandle] = None
        shape = self.get_initial_param("target_shape", "4x11", description="Size of target pattern as e,g 4x11")
        self.target_shape = [int(x) for x in shape.split("x")]
        self.target_type = self.get_initial_param("target_type", "circle",
                                                  description="Target type, currently only circle available")
        self.target_size = self.get_initial_param("target_size", 0.036,
                                                  description="Target size in metres")
        self._cal_action = ActionServer(self, Calibrate, "do_calibration", self._do_calibration)
        self._srv = self.create_service(Trigger, 'finalise_calibration', self._finalise_calibration)

        self.bridge = CvBridge()

    def get_object_corners(self) -> np.ndarray:
        """
        Get the Mx1x3 array that represents the corners of the calibration pattern
        :return:
        """
        max_x, max_y = self.target_shape
        if self.target_type=="circle":
            corners = np.zeros((max_x*max_y, 1, 3),dtype=np.float32)
            for j in range(max_y):
                for i in range(max_x):
                    corners[j*max_x + i, 0, 0] = (i + (j%2)/2)
                    corners[j*max_x + i, 0, 1] = j/2
            corners *= self.target_size
        else:
            raise ValueError("Only circle target type currently supported")
        return corners

    def find_corners(self, image: np.ndarray) -> Tuple[bool, np.ndarray]:
        """
        Find the Mx1x2 array of camera points that represent the found pattern
        :param image:
        :return:
        """
        if self.target_type=="circle":
            ret, corners = cv2.findCirclesGrid(image, self.target_shape,
                                               flags=cv2.CALIB_CB_ASYMMETRIC_GRID + cv2.CALIB_CB_CLUSTERING)
            return ret, corners
        else:
            raise ValueError("Only circle target type currently supported")

    def draw_corners(self, image: np.ndarray, corners: np.ndarray, pattern_found=True):
        """
        Draw the found corners on the image
        :param np.ndarray image: BGR or mono original image in cv2 format
        :param np.ndarray corners: Mx1x2 numpy array. M=number of object points
        :param bool pattern_found: whether pattern was found
        :return:
        """
        if len(image.shape)==2 or image.shape[2]==1:
            new_img = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        else:
            new_img = image.copy()
        if self.target_type=="circle":
            cv2.drawChessboardCorners(new_img, self.target_shape, corners, pattern_found)
        return new_img

    def on_start_calibration(self):
        """
        Override this function with any code that is needed when the calibration starts
        """

    def on_finish_calibration(self, result: Calibrate.Result):
        """
        Override this function with any code that is needed when the calibration finishes
        Alter the fields of result to denote outcome
        """

    def _do_calibration(self, gh: ServerGoalHandle):
        self.gh = gh
        self.calibrating = True
        self._calibration_complete = Future()
        self.on_start_calibration()
        #wait until finalise calibration called
        rclpy.spin_until_future_complete(self, self._calibration_complete)
        result = Calibrate.Result()
        self.on_finish_calibration(result)
        self.gh.succeed()
        return result

    def publish_progress(self, progress: int):
        """
        Call this with an int to report progress
        :param progress:
        :return:
        """
        if self.gh:
            self.gh.publish_feedback(Calibrate.Feedback(count=progress))

    def _finalise_calibration(self, req:Trigger.Request, rsp:Trigger.Response):
        if self._calibration_complete:
            self._calibration_complete.set_result(None)
        self.calibrating = False
        rsp.success = True
        return rsp