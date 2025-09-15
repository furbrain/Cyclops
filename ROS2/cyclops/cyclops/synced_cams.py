import multiprocessing
import queue
import threading
import time
from threading import Thread
from typing import Optional, NamedTuple, List, Tuple, Sequence

import numpy as np

import picamera2
import rclpy.logging

TARGET_DURATION = 100_000
OFFSET_SCALE = 2

class Frame(NamedTuple):
    ts: int
    name: str
    shape: Sequence[int]
    data: bytes

class FrameSet(NamedTuple):
    rgb: Frame
    tof: List[Frame]

def get_rgb_cam() -> picamera2.Picamera2:
    cam = picamera2.Picamera2(0)
    config = cam.create_video_configuration(main={"format": 'RGB888', 'size': (1920,1080)},
                                            controls={'FrameDurationLimits': (TARGET_DURATION, TARGET_DURATION)},
                                            buffer_count=2,
                                            display=None,
                                            )
    cam.configure(config)
    cam.start()
    return cam

def get_tof_cam() -> picamera2.Picamera2:
    cam = picamera2.Picamera2(1)
    config = cam.create_video_configuration(raw={"format": 'R12', 'size': (240, 180)},
                                            #controls={'FrameRate': 120.0},
                                            display=None,
                                            buffer_count=8
                                            )
    cam.configure(config)
    cam.start()
    return cam

class CamThread:
    def __init__(self, cam: picamera2.Picamera2, name: str, stream: str, q: queue.Queue,):
        self.cam = cam
        self.name = name
        self.stream = stream
        self.q = q
        self._running = True
        self._thread = Thread(target=self.runner, daemon=True)
        self._thread.start()

    def runner(self):
        req: picamera2.CompletedRequest
        while self._running:
            with self.cam.captured_request() as req:
                ts = req.get_metadata()["SensorTimestamp"]
                arr = req.make_array(self.stream)
                frame = Frame(ts, self.name, arr.shape, arr.tobytes())
                self.q.put(frame)

    def stop(self):
        self._running = False
        self._thread.join()
        self.cam.stop()


class SyncCams:
    def __init__(self, output_queue: multiprocessing.Queue):
        self.output_queue = output_queue
        self.rgb_cam = get_rgb_cam()
        self.internal_queue = queue.Queue()
        self.rgb_thread = CamThread(self.rgb_cam, "rgb", "main", self.internal_queue)
        self.tof_cam = get_tof_cam()
        self.tof_thread = CamThread(self.tof_cam, "tof", "raw", self.internal_queue)
        self._running = True
        self._thread = threading.Thread(target=self.run)

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    def start(self):
        self._thread.start()

    @staticmethod
    def get_offset(D: int) -> int:
        if D > 10:
            offset = 1000
        elif D > 5:
            offset = 500
        elif D > 2:
            offset = 250
        elif D > -2:
            offset = 0
        elif D > -5:
            offset = -250
        elif D > -10:
            offset = -500
        else:
            offset = -1000
        return offset*OFFSET_SCALE

    def run(self):
        tof0: List[Frame] = []
        tof1: List[Frame] = []
        last_rgb: Optional[Frame] = None
        last_tof: Optional[Frame] = None
        recent_main = False
        last_duration = 0
        while self._running:
            frame = self.internal_queue.get()
            if frame.name=="rgb":
                if len(tof0) > 0:
                    recent_main = True
                last_rgb = frame
            elif frame.name=="tof":
                if last_tof is None or (frame.ts - last_tof.ts) > 10_000_000:
                    #first frame of new set
                    if len(tof0) < 4:
                        tof0 = [frame]
                    else:
                        if len(tof1)>=4:
                            tof0 = tof1
                        tof1 = [frame]
                else:
                    if len(tof0) >= 4:
                        tof1.append(frame)
                    else:
                        tof0.append(frame)
                last_tof=frame
            if len(tof1) >= 4 and recent_main:
                T0 = sum(x.ts for x in tof0)//4
                T1 = sum(x.ts for x in tof1)//4
                R = last_rgb.ts
                D0 = T0-R
                D1 = T1-R
                if abs(D0) < abs(D1):
                    out_tof = tof0
                    D = D0
                else:
                    out_tof = tof1
                    D = D1
                D  = D / 1_000_000
                offset = self.get_offset(D)
                new_duration = TARGET_DURATION+offset
                if new_duration != last_duration:
                    rclpy.logging.get_logger("SyncCam").warn(f"adjusting timing now {new_duration} was {last_duration}")
                    self.rgb_cam.set_controls({'FrameDurationLimits':(new_duration, new_duration)})
                    rclpy.logging.get_logger("SyncCam").warn(f"adjusting timing now {new_duration} was {last_duration}")
                    last_duration = new_duration
                if self.output_queue.full():
                    try:
                        self.output_queue.get(block=False)
                    except queue.Empty:
                        pass
                self.output_queue.put(FrameSet(last_rgb,out_tof))
                recent_main = False

    def stop(self):
        self._running = False
        self._thread.join(1)
        self.rgb_thread.stop()
        self.tof_thread.stop()

if __name__=="__main__":
    with SyncCams() as cams:
        print("Starting")
        time.sleep(10)
        print("Stopping")
