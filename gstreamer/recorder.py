#!/usr/bin/env python3
import board
import digitalio
import pwmio
import time
import sys
import subprocess
import os
import signal
import shutil

pwm = pwmio.PWMOut(board.D18, frequency=2000)
pwm.duty_cycle = 0

button = digitalio.DigitalInOut(board.D20)
button.pull = digitalio.Pull.UP
vid_count = 0

def wait_for_button_press():
    while button.value:
        time.sleep(0.1)
    while not button.value:
        time.sleep(0.1)        

def beep(seconds):
    pwm.duty_cycle = 2 ** 15
    time.sleep(seconds)
    pwm.duty_cycle = 0
    
while True:
    wait_for_button_press()
    beep(0.1)
    print("Recording")
    cmd = "./test.sh"
    pro = subprocess.Popen(cmd, stdout=subprocess.PIPE, 
                       shell=True, preexec_fn=os.setsid) 
    wait_for_button_press()
    print("Terminating")
    os.killpg(os.getpgid(pro.pid), signal.SIGTERM)
    beep(0.5)
    shutil.move("combined.mkv", f"vid{vid_count}.mkv")
    vid_count += 1
    print("Finished recording")
