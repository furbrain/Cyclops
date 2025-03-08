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
from std_srvs.srv import Trigger

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
        rospy.sleep(1)
        rospy.wait_for_service("wait_for_pubs")
        waiter = rospy.ServiceProxy('wait_for_pubs', Trigger)
        result = waiter()
        print(result)
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
