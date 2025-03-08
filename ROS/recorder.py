#!/usr/bin/env python3
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

LAUNCH_FILE = "start_sensors.launch"

LAUNCH_PKG = "cyclops_launch"

HAPPY = (("C6", 0.2), ("E6", 0.2), ("G6", 0.2), ("C7", 0.2))
SAD = tuple(reversed(HAPPY))
SHORT = (("C7", 0.1),)
LONG = (("C7", 0.5),)
BUTTON_PIN = board.D20

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
    launch_dir = rospkg.RosPack().get_path(LAUNCH_PKG)
    launch_script = Path(launch_dir) / "launch" / LAUNCH_FILE
    ros_uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    while True:
        result = wait_for_button_press(button)
        if result:
             break
        beeper.beep(SHORT)
        print("Recording")
        launch = roslaunch.parent.ROSLaunchParent(ros_uuid, [launch_script], is_core=True, force_required=True)
        launch.start()
        wait_for_button_press()
        beeper.beep(LONG)
        time.sleep(1)
        print("Finished recording")

if __name__=="__main__":
    main()