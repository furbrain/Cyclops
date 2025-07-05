#!/usr/bin/env python3
from sensor_msgs.msg import Imu
import rospy

class AdjustTimeStamp:
    def __init__(self):
        self.sub = rospy.Subscriber("~in", Imu, self.callback, queue_size=5)
        self.pub = rospy.Publisher("~out", Imu, queue_size=5)
        self.offset = rospy.Duration.from_sec(rospy.get_param("~offset"))
        
    def callback(self, imumsg: Imu):
        imumsg.header.stamp += self.offset
        self.pub.publish(imumsg)

rospy.init_node("adjust_timestamp")
AdjustTimeStamp()
rospy.spin()
