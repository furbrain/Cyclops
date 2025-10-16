from typing import Tuple, Optional
from enum import Enum, auto
import cv2
import numpy as np

import rclpy
from camera_info_manager import CameraInfoManager
from cyclops_interfaces.msg import Sync
import picamera2
import cv_bridge
from sensor_msgs.msg import Image, CameraInfo, CompressedImage

from .utils import SmartNode

class TimeStampSet:
    def __init__(self, count: int, expected_period: int):
        self.count = count
        self.data = np.zeros(count, dtype=np.int64)
        self.index = 0
        self.expected_period = expected_period

    def store(self, val:int):
        self.data[self.index] = val
        self.index  = (self.index + 1) % self.count

    def ready(self) -> bool:
        return bool(np.all(self.data))

    def period(self) -> Tuple[bool, int]:
        val =( max(self.data) - min(self.data)) // (self.count-1)
        success =  abs(val-self.expected_period) < self.expected_period// (2*self.count)
        return success, val

    def offset(self, period: Optional[int] = None):
        if period is None:
            _, period = self.period()
        # move all data to an offset of approx period//2, assuming x[0] is typical
        # this should be a valid assumption
        k = period//2 - self.data[0]
        y = (self.data+k) % period
        mean_phi_y = int(np.average(y))

        # correct back to average result
        mean_phi_x = mean_phi_y - k
        return mean_phi_x

class State(Enum):
    INIT = auto()
    WATCHING = auto()
    CORRECTING = auto()
    COOLDOWN = auto()


class CameraNode(SmartNode):
    def __init__(self):
        super().__init__("rgb")
        self.info_mgr = CameraInfoManager(self,"rgb")
        self.info_mgr.loadCameraInfo()
        self.pub = self.create_publisher(Image,"image_raw", 2)
        self.pub_compressed = self.create_publisher(CompressedImage, "image_raw/compressed", 2)
        self.ci_pub = self.create_publisher(CameraInfo,"camera_info", 2)
        self.sync_sub = self.create_subscription(Sync, "sync", self.sync_handler, 3)
        crop_x = self.get_initial_param("crop_x",0)
        crop_y = self.get_initial_param("crop_y", 0)
        width = self.get_initial_param("width", 0)
        height = self.get_initial_param("height", 0)
        self.sync_frame_count = self.get_initial_param("sync_frame_count", 10)
        self.resize_width = self.get_initial_param("resize_width", 0)
        self.period_us = 1_000_000 // self.get_initial_param("frame_rate", 10)
        self.frame_id = self.get_initial_param("frame_id", "rgb")
        self.tof_stamps = TimeStampSet(self.sync_frame_count, self.period_us*1000)
        self.rgb_stamps = TimeStampSet(self.sync_frame_count, self.period_us*1000)

        cam_list = picamera2.Picamera2.global_camera_info()
        for cam in cam_list:
            self.cam: picamera2.Picamera2 = picamera2.Picamera2(cam['Num'])
            if self.cam.sensor_modes[0]['unpacked'].startswith("SRG"):
                break
        else:
            print("No RGB camera found, aborting!")
            exit()
        config = self.cam.create_video_configuration(main={"format": 'RGB888', 'size': (1920, 1080)},
                                                     controls={'FrameDurationLimits': (self.period_us,
                                                                                       self.period_us)},
                                                     buffer_count=2,
                                                     display=None,
                                                     lores=None,
                                                     raw=None,
                                                     )
        self.cam.configure(config)
        self.cam.start()
        # setting up cropping area as a slice
        if not all(x==0 for x in (crop_x, crop_y, width, height)):
            self.crop = slice
            if width<=0:
                x_end = None
            else:
                x_end = crop_x + width
            if height<=0:
                y_end = None
            else:
                y_end = crop_y + height
            self.crop: Optional[Tuple[slice]] = np.s_[crop_y:y_end, crop_x:x_end]
        else:
            self.crop = None
        self.bridge = cv_bridge.CvBridge()
        self.state = State.INIT
        self.frame_count = 0

    def on_loop(self):
        self.publish_image()
        self.run_state_machine()

    def publish_image(self):
        with self.cam.captured_request() as req:
            im = req.make_array("main")
            if self.crop:
                im = im[self.crop]
            if self.resize_width:
                resize_height = (im.shape[0] * self.resize_width) // im.shape[1]
                im = cv2.resize(im, (self.resize_width, resize_height), interpolation=cv2.INTER_NEAREST)
            msg = self.bridge.cv2_to_imgmsg(im, encoding="bgr8")
            comp_msg = self.bridge.cv2_to_compressed_imgmsg(im, dst_format="jpeg")
            ts = req.get_metadata()['SensorTimestamp']
            self.rgb_stamps.store(ts)
            stamp = self.time_from_ns(ts).to_msg()
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id
            comp_msg.header = msg.header
        self.pub.publish(msg)
        self.pub_compressed.publish(comp_msg)

        ci_msg = self.info_mgr.getCameraInfo()
        ci_msg.header = msg.header
        self.ci_pub.publish(ci_msg)

    def run_state_machine(self):
        if self.state == State.INIT:
            if self.rgb_stamps.ready() and self.tof_stamps.ready():
                self.state = State.WATCHING
                self.get_logger().info("Entering Watching mode")
        elif self.state == State.WATCHING:
            valid_period, period = self.tof_stamps.period()
            self.get_logger().debug(f"Watching: {valid_period}, {period}")
            if valid_period:
                tof_offset = self.tof_stamps.offset(period)
                rgb_offset = self.rgb_stamps.offset(period)
                self.get_logger().debug(f"Disparity: {tof_offset}, {rgb_offset}")
                correction = (tof_offset-rgb_offset) % period
                if correction > (3*period)//4:
                    correction -= period
                if abs(correction) > 5e6: # more than 5ms out
                    self.period_us = period // 1000
                    new_duration = (period + correction) // 1000 #convert to us for picamera!
                    self.cam.set_controls({'FrameDurationLimits':(new_duration, new_duration)})
                    self.state = State.CORRECTING
                    self.get_logger().warn(f"Starting correction. Correction: {new_duration}")
        elif self.state == State.CORRECTING:
            self.cam.set_controls({'FrameDurationLimits': (self.period_us, self.period_us)})
            self.state = State.COOLDOWN
            self.frame_count = 0
            self.get_logger().info(f"Correction finished, new_period: {self.period_us} ")
        elif self.state == State.COOLDOWN:
            self.frame_count += 1
            if self.frame_count >= self.sync_frame_count*2:
                self.state = State.WATCHING
                self.get_logger().info("Returning to Watching mode")

    def sync_handler(self, msg: Sync):
        self.tof_stamps.store(msg.nanoseconds)

def main():
    rclpy.init()
    node = CameraNode()
    node.run()

if __name__=="__main__":
    main()