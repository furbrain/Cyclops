#!/usr/bin/env python3
import sys
from typing import List, Dict

import rospy
import rostopic
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse


class PublicationCheck:
    def __init__(self, topics: List[str]):
        self.service = rospy.Service("wait_for_pubs", Trigger, self.callback)
        self.topics = topics

    def callback(self, req:TriggerRequest):
        for topic in self.topics:
            print(f"Waiting for {topic}")
            rospy.wait_for_message(topic, rospy.AnyMsg)
            print(f"{topic} has published")
        return TriggerResponse(True,f"{self.topics} all publishing")

if __name__=="__main__":
    rospy.init_node("waiter")
    PublicationCheck(rospy.myargv()[1:])
    rospy.spin()
