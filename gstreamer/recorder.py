#!/usr/bin/env python3
import board
import digitalio
import time
import sys
import subprocess
import os
from pathlib import Path
import signal
import shutil
from xmlrpc.client import  ServerProxy

HAPPY = (("C6", 0.2), ("E6", 0.2), ("G6", 0.2), ("C7", 0.2))
SAD = tuple(reversed(HAPPY))
SHORT = (("C7", 0.1),)
LONG = (("C7", 0.5),)
THIS_DIR = Path(__file__).absolute().parent

button = digitalio.DigitalInOut(board.D20)
button.pull = digitalio.Pull.UP
vid_count = 0

def wait_for_button_press() -> bool:
    while button.value:
        time.sleep(0.1)
    start = time.time()
    while not button.value:
        time.sleep(0.1)        
    return (time.time()-start) > 1.0


cmd = [sys.executable, THIS_DIR / "beep_server.py"] 
beeper_process = subprocess.Popen(cmd, stdout=subprocess.PIPE, 
                                  shell=False, preexec_fn=os.setsid) 
time.sleep(1)
beeper = ServerProxy("http://localhost:8000", allow_none=True)



beeper.beep(HAPPY)
while True:
    result = wait_for_button_press()
    if result:
         print("Shutting down")
         beeper.beep(SAD)
         os.killpg(os.getpgid(beeper_process.pid), signal.SIGTERM)
         subprocess.Popen("sudo shutdown -h now", shell=True)
         time.sleep(60)
    beeper.beep(SHORT)
    print("Recording")
    cmd = THIS_DIR / "test.sh"
    pro = subprocess.Popen(str(cmd), stdout=subprocess.PIPE, 
                       shell=True, preexec_fn=os.setsid, cwd = str(THIS_DIR)) 
    wait_for_button_press()
    print("Terminating")
    os.killpg(os.getpgid(pro.pid), signal.SIGTERM)
    beeper.beep(LONG)
    time.sleep(1)
    shutil.move("/home/pi/combined.mkv", f"/home/pi/vid{vid_count}.mkv")
    vid_count += 1
    print("Finished recording")

