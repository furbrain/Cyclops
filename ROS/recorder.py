#!/usr/bin/env python3
from functools import partial
from typing import Sequence, Dict, Any

import board
import digitalio
import time
import os
from pathlib import Path
import signal
import shutil
from xmlrpc.client import  ServerProxy

import roslaunch
import roslaunch.rlutil
import roslaunch.parent
import rospkg
import rospy
import rostopic
from sensor_msgs.msg import CompressedImage, Imu, Image, MagneticField

LAUNCH_FILE = "local_start_sensors.launch"

LAUNCH_PKG = "cyclops_launch"

HAPPY = (("C6", 0.2), ("E6", 0.2), ("G6", 0.2), ("C7", 0.2))
SAD = tuple(reversed(HAPPY))
SHORT = (("C7", 0.1),)
LONG = (("C7", 0.5),)
BUTTON_PIN = board.D20
TOPICS_REQUIRED = ['/imu/imu_raw',
                   '/lidar/image_raw',
                   '/camera/image_raw/compressed']


def init_button() -> digitalio.DigitalInOut:
    button = digitalio.DigitalInOut(BUTTON_PIN)
    button.pull = digitalio.Pull.UP
    return button

def wait_for_button_press(button: digitalio.DigitalInOut, launcher: roslaunch.parent.ROSLaunchParent = None) -> bool:
    while button.value:
        rospy.sleep(0.1)
        if launcher and launcher.server.is_shutdown:
            return False
    start = time.time()
    while not button.value:
        rospy.sleep(0.1)
    return (time.time()-start) > 1.0

class PublicationCheck:
    def __init__(self, topics: Dict[str,type]):
        self.found = {topic: False for topic in topics}
        rospy.init_node("waiter")
        self.subs: Dict[str, rospy.Subscriber] = {}
        for topic, class_ in topics.items():
            self.subs[topic] = rospy.Subscriber(topic,
                                                class_,
                                                partial(self.callback,topic))
        print(self.subs)


    def callback(self, topic: str, message: Any):
        print(f"{topic} has published")
        self.found[topic] = True
        self.subs[topic].unregister()

    def wait(self):
        while not any(self.found.values()):
            rospy.sleep(0.1)
        rospy.signal_shutdown("kill waiter")

def main():
    button  = init_button()
    beeper = ServerProxy("http://localhost:8000", allow_none=True)
    beeper.beep(HAPPY)
    launch_script = roslaunch.rlutil.resolve_launch_arguments((LAUNCH_PKG, LAUNCH_FILE))[0]
    print(launch_script)
    ros_uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    while True:
        result = wait_for_button_press(button)
        if result:
             beeper.beep(SAD)
             break
        beeper.beep(SHORT)
        print("Recording")
        launch = roslaunch.parent.ROSLaunchParent(ros_uuid, [launch_script], is_core=True, force_required=True)
        launch.start()
        print("launched")
        #waiter = PublicationCheck(TOPICS_REQUIRED)
        #waiter.wait()
        for topic in TOPICS_REQUIRED:
            print(f"Waiting for {topic}") 
            os.system(f"rostopic echo --noarr -n 1 {topic}")
        print("all running")
        beeper.beep(HAPPY)
        wait_for_button_press(button)
        beeper.beep(SHORT)
        launch.shutdown()
        time.sleep(1)
        beeper.beep(LONG)
        print("Finished recording")

if __name__=="__main__":
    main()
