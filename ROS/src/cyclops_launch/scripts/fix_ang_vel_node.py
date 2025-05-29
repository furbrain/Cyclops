#!/usr/bin/env python3
import argparse
import math as maths

import numpy as np
from sensor_msgs.msg import Imu
import rospy



RADS_PER_DEG = maths.pi / 180.0


class FixAngVel:
    def __init__(self):
        self.sub = rospy.Subscriber("~in", Imu, self.callback, queue_size=5)
        self.pub = rospy.Publisher("~out", Imu, queue_size=5)
        
    def callback(self, msg):
        new_msg = Imu(linear_acceleration=msg.linear_acceleration)
        new_msg.angular_velocity.x = msg.angular_velocity.x * RADS_PER_DEG
        new_msg.angular_velocity.y = msg.angular_velocity.y * RADS_PER_DEG
        new_msg.angular_velocity.z = msg.angular_velocity.z * RADS_PER_DEG
        new_msg.header = msg.header
        self.pub.publish(new_msg)
        
if __name__=="__main__":
    rospy.init_node("fix_ang_vel")
    FixAngVel()
    rospy.spin()
