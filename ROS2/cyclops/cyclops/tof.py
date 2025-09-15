import copy
import time

import rclpy
import rclpy.node
import rclpy.time
from .utils import SmartNode
from camera_info_manager import CameraInfoManager
from cyclops_interfaces.msg import Sync
from rclpy.qos import qos_profile_sensor_data
import ArducamDepthCamera as ac
import numpy as np
import cv2
from sensor_msgs.msg import Image, CameraInfo

RANGE = 4000

class Tof(SmartNode):
    def __init__(self):
        super().__init__("tof")
        self.pub = self.create_publisher(Image,"depth_raw", 3)
        self.pub_img = self.create_publisher(Image,"image_raw", 3)
        self.info_mgr = CameraInfoManager(self,"tof")
        self.info_mgr.loadCameraInfo()
        self.ci_pub = self.create_publisher(CameraInfo,"camera_info", 2)
        self.sync_pub = self.create_publisher(Sync,"sync", 2)

        self.cam = ac.ArducamCamera()
        if self.cam.open(ac.Connection.CSI, 0) != 0 :
            print("initialization failed")
        if self.cam.start(ac.FrameType.DEPTH) != 0 :
            print("Failed to start camera")
        self.cam.setControl(ac.Control.RANGE, RANGE)
        self.cam.setControl(ac.Control.SKIP_FRAME, 2) # skip 2 out of 3 frames - frequency -> 10Hz
        self.cam.setControl(ac.Control.SKIP_FRAME_LOOP, 3)


    def run(self):
        msg = Image()
        msg.height = 180
        msg.width = 240
        msg.is_bigendian = False
        msg.encoding = "32FC1"
        msg.header.frame_id = "tof"
        msg.step = msg.width*4
        img_msg = copy.deepcopy(msg)
        img_msg.encoding = "mono8"
        img_msg.step = msg.width
        last_ts = self.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            frame = self.cam.requestFrame(200)
            ts_ns = time.monotonic_ns() - 7_700_000
            ts = self.time_from_ns(ts_ns) # this point is approx 7.7ms after frame timestamp
            if frame is not None and isinstance(frame, ac.DepthData):
                print(f"Dt: {ts-last_ts}")
                last_ts = ts
                msg.header.stamp = ts.to_msg()
                img_msg.header.stamp = msg.header.stamp
                depth_buf = frame.depth_data
                bad_pixels = frame.confidence_data < 30
                amp_buf = np.sqrt(frame.amplitude_data)
                amp_buf = 255 * amp_buf / np.max(amp_buf)
                depth_float = depth_buf.astype(np.float32) / 1000
                depth_float[bad_pixels] = np.nan
                depth_img = depth_float.reshape((msg.height,msg.width))
                depth_img = cv2.rotate(depth_img,cv2.ROTATE_180)

                amp_img = amp_buf.astype("uint8").reshape((msg.height,msg.width))
                amp_img = cv2.rotate(amp_img, cv2.ROTATE_180)

                msg.data = depth_img.tobytes()
                img_msg.data = amp_img.tobytes()

                ci_msg = self.info_mgr.getCameraInfo()
                ci_msg.header = msg.header
                self.pub.publish(msg)
                self.pub_img.publish(img_msg)
                self.ci_pub.publish(self.info_mgr.getCameraInfo())
                self.sync_pub.publish(Sync(nanoseconds = ts_ns))
                self.cam.releaseFrame(frame)            
                
    def shutdown(self):
        self.cam.stop()

def main(args=None):
    rclpy.init(args=args)
    node = Tof()
    node.run()
    node.shutdown()

if __name__ == '__main__':
    main()
