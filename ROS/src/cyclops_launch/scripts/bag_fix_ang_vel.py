#!/usr/bin/env python3

import math as maths

from bag_adjust import Bagger
from sensor_msgs.msg import Imu

RADS_PER_DEG = maths.pi / 180.0

class FixAngVel(Bagger):
    FROM_TOPIC="/imu/imu_rect"

    def process(self, msg):
        new_msg = Imu(linear_acceleration=msg.linear_acceleration)
        new_msg.angular_velocity.x = msg.angular_velocity.x * RADS_PER_DEG
        new_msg.angular_velocity.y = msg.angular_velocity.y * RADS_PER_DEG
        new_msg.angular_velocity.z = msg.angular_velocity.z * RADS_PER_DEG
        return new_msg
        
FixAngVel().run()
