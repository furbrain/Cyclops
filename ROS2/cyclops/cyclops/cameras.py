import multiprocessing
import queue
from typing import List, Tuple, Optional

import cv2
import numpy as np

import rclpy
import rclpy.time
from camera_info_manager import CameraInfoManager
import cv_bridge
from rclpy.logging import get_logger
from sensor_msgs.msg import Image, CameraInfo

from .utils import SmartNode
from .synced_cams import SyncCams, FrameSet


class RGBCam:
    def __init__(self, node: SmartNode):
        self.info_mgr = CameraInfoManager(node, "rgb")
        self.info_mgr.loadCameraInfo()
        self.pub = node.create_publisher(Image, "rgb/image_raw", 5)
        self.ci_pub = node.create_publisher(CameraInfo, "rgb/camera_info", 5)
        crop_x = node.get_initial_param("crop_x",0)
        crop_y = node.get_initial_param("crop_y", 0)
        width = node.get_initial_param("width", 0)
        height = node.get_initial_param("height", 0)
        self.resize_width = node.get_initial_param("resize_width", 0)
        if not all(x==0 for x in (crop_x, crop_y, width, height)):
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

    def process_frame(self, frame:np.ndarray, ts: rclpy.time.Time):
        if self.crop:
            frame = frame[self.crop]
        if self.resize_width:
            resize_height = (frame.shape[0] * self.resize_width) // frame.shape[1]
            frame = cv2.resize(frame, (self.resize_width, resize_height), interpolation=cv2.INTER_NEAREST)
        msg = self.bridge.cv2_to_imgmsg(frame[:, :, :3], encoding="bgr8")
        msg.header.stamp = ts.to_msg()
        msg.header.frame_id = "rgb"
        self.pub.publish(msg)
        self.ci_pub.publish(self.info_mgr.getCameraInfo())

class TOFCam:
    def __init__(self, node:SmartNode):
        self.info_mgr = CameraInfoManager(node, "rgb")
        self.info_mgr.loadCameraInfo()
        self.img_pub = node.create_publisher(Image, "tof/image_raw", 5)
        self.depth_pub = node.create_publisher(Image, "tof/depth_raw", 5)
        self.ci_pub = node.create_publisher(CameraInfo, "tof/camera_info", 5)
        self.bridge = cv_bridge.CvBridge()

    def process_frame(self, frames: List[np.ndarray], ts:rclpy.time.Time):
        arrs = [np.frombuffer(x.data, dtype=np.uint16).reshape((180,256)) for x in frames]
        arrs = [(x << 1) + 0x8000 for x in arrs]
        Q = arrs[1] + arrs[3]
        I = arrs[2] + arrs[0]
        #bad_pixels = np.logical_and(Q==0,I==0)
        #print(np.count_nonzero(bad_pixels))
        distance = np.arctan2(Q, I).astype(np.float32) % (np.pi * 2)
        #confidence = np.sqrt(Q ** 2 + I ** 2)
        distance *= 2/np.pi
        #img_msg = self.bridge.cv2_to_imgmsg(distance, encoding="32FC1")
        img_msg = self.bridge.cv2_to_imgmsg(np.vstack(arrs), encoding="16UC1")
        img_msg.header.stamp = ts.to_msg()
        img_msg.header.frame_id = "tof"
        self.img_pub.publish(img_msg)
        #distance[confidence < 30] = 0
        # distance = cv2.normalize(distance, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        # distance = cv2.medianBlur(distance,3)
        # #confidence= cv2.normalize(confidence, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        # colored = cv2.applyColorMap(distance,cv2.COLORMAP_JET)
        # cv2.imshow("main", colored)
        # cv2.imshow("raw", frame)
        # #cv2.imshow("confidence", confidence)

class SyncProcess(multiprocessing.Process):
    def __init__(self, queue):
        super().__init__()
        self.queue = queue
        self.sync = None
        self.event = multiprocessing.Event()

    def run(self):
        self.sync = SyncCams(self.queue)
        try:
            self.sync.start()
            self.event.wait()
        finally:
            self.sync.stop()

    def stop(self):
        rclpy.logging.get_logger("SyncProcess").warn("stopping")
        self.event.set()
        self.join(timeout=2.0)


class CameraNode(SmartNode):
    def __init__(self):
        super().__init__("Camera")
        self.rgb_cam = RGBCam(self)
        self.tof_cam = TOFCam(self)
        self.img_queue = multiprocessing.Queue()
        self.sync = SyncProcess(self.img_queue)

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    def start(self):
        self.sync.start()

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.001)
            try:
                frames: FrameSet = self.img_queue.get(timeout=0.01)
            except queue.Empty:
                continue
            ts = self.time_from_ns(frames.rgb.ts)
            #self.rgb_cam.process_frame(frames.rgb.data, ts)
            self.tof_cam.process_frame([x for x in frames.tof], ts)

    def stop(self):
        self.sync.stop()

def main():
    rclpy.init()
    with CameraNode() as node:
        node.run()

if __name__=="__main__":
    main()