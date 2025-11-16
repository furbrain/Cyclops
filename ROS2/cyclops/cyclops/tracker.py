import numpy as np

import rclpy
from sensor_msgs.msg import Imu
from autonode import Node, subscription


class TrackerNode(Node):
    def __init__(self):
        super().__init__()
        self.msg_count = 0
        self.cumulative = np.zeros((3,))
        self.last_time = 0

    @subscription(Imu)
    def imu(self, msg: Imu):
        tm = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.msg_count > 0:
            angular = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
            dt = tm - self.last_time
            self.cumulative += angular * dt
        self.msg_count += 1
        self.last_time = tm
        if self.msg_count % 20 ==0:
            print(self.cumulative, np.sqrt(msg.linear_acceleration.x**2 + msg.linear_acceleration.y**2 +
                                        msg.linear_acceleration.z**2))

def main():
    rclpy.init()
    node = TrackerNode()
    node.run()

if __name__=="__main__":
    main()
