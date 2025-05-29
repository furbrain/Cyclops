#!/usr/bin/env python3
import argparse
import numpy as np
from sensor_msgs.msg import Imu
from sensor_msgs_ext.msg import accelerometer
import rospy

class AddAccelerometer:
    def __init__(self):
        self.sub = rospy.Subscriber("~in", Imu, self.callback, queue_size=5)
        self.pub = rospy.Publisher("~out", accelerometer, queue_size=5)
        
    def callback(self, msg: Imu):
        out_msg = accelerometer(x=msg.linear_acceleration.x, y=msg.linear_acceleration.y, z=msg.linear_acceleration.z)
        self.pub.publish(out_msg)

rospy.init_node("get_accel")
AddAccelerometer()
rospy.spin()
