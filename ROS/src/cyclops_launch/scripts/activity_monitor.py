import sys
from typing import List, Dict

import rospy
import rostopic
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse


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
            print(f"{topic} has published")
        return TriggerResponse(True,f"{self.topics} all publishing")

if __name__=="__main__":
    rospy.init_node("waiter")
    PublicationCheck(sys.argv[1:])