import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data
import ArducamDepthCamera as ac
import numpy as np
import cv2
import copy
import time

from sensor_msgs.msg import Image

RANGE = 4000

class Tof(rclpy.node.Node):
    def __init__(self):
        super().__init__("tof")
        self.pub = self.create_publisher(Image,"depth_raw", 3)
        self.pub_img = self.create_publisher(Image,"image_raw", 3)
        self.cam = ac.ArducamCamera()
        if self.cam.open(ac.Connection.CSI, 0) != 0 :
            print("initialization failed")
        if self.cam.start(ac.FrameType.DEPTH) != 0 :
            print("Failed to start camera")
        self.cam.setControl(ac.Control.RANGE, RANGE)
        print(self.get_clock().now().nanoseconds)
        print(time.monotonic_ns())
        

    def run(self):
        msg = Image()
        msg.height = 180
        msg.width = 240
        msg.is_bigendian = False
        msg.encoding = "16UC1"
        msg.header.frame_id = "tof"
        msg.step = msg.width*2
        img_msg = copy.deepcopy(msg)
        img_msg.encoding = "mono8"
        img_msg.step = msg.width
        
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            frame = self.cam.requestFrame(200)
            if frame is not None and isinstance(frame, ac.DepthData):
                msg.header.stamp = self.get_clock().now().to_msg()
                img_msg.header.stamp = self.get_clock().now().to_msg()
                depth_buf = frame.depth_data
                bad_pixels = frame.confidence_data < 30
                amp_buf = np.sqrt(frame.amplitude_data)
                amp_buf = 255 * amp_buf / np.max(amp_buf)

                depth_buf[bad_pixels] = 0
                depth_img = depth_buf.astype("int16")
                depth_img.resize((msg.height,msg.width))
                depth_img = cv2.rotate(depth_img,cv2.ROTATE_180)

                amp_img = amp_buf.astype("uint8")
                amp_img.resize((msg.height,msg.width))
                amp_img = cv2.rotate(amp_img, cv2.ROTATE_180)

                msg.data = depth_img.tobytes()
                img_msg.data = amp_img.tobytes()

                self.pub.publish(msg)
                self.pub_img.publish(img_msg)
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
