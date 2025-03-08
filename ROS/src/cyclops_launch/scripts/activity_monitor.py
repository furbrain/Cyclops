from typing import List, Dict
from functools import partial

import rospy
import rostopic
from std_srvs.srv import Trigger, TriggerRequest


class PublicationCheck:
    def __init__(self, topics: List[str]):
        self.service = rospy.Service("wait_for_pubs", Trigger, self.callback())
        self.topics = topics

    def callback(self, req:TriggerRequest):
        for topic in self.topics:
            print(f"Waiting for {topic}")
            _class = rostopic.get_topic_class(topic)
            print(f"Class is {_class}")
            rospy.wait_for_message(topic,_class)

if __name__=="__main__":
    rospy.init_node("waiter")
    PublicationCheck()