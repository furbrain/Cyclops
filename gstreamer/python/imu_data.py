'''
Element that generates video data based on the IMU output

Example pipeline:

gst-launch-1.0 py_imu_data ! video/x-raw,framerate=50 ! videoconvert ! autovideosink
'''
import time
from xmlrpc.client import  ServerProxy
from threading import Thread
from queue import Queue
import numpy as np

import gi

gi.require_version('Gst', '1.0')
gi.require_version('GstBase', '1.0')
gi.require_version('GstVideo', '1.0')

from gi.repository import Gst, GLib, GObject, GstBase, GstVideo
import struct
import json

DUMMY = False

if DUMMY:
    try:
        import numpy as np
    except ImportError:
        Gst.error('imu_data requires numpy')
        raise
else:
    import board
    import icm
    import digitalio
    import busio

OCAPS = Gst.Caps.from_string ('video/x-raw, format=GRAY8, width=16, height=16, framerate=[10/1,100/1]')
HAPPY = (("C6", 0.1), ("E6", 0.1), ("G6", 0.1), ("C7", 0.1))
BIP = (("C7",0.1),)

class Beeper(Thread):
    def __init__(self, queue: Queue):
        super().__init__()
        self._queue = queue
        self._beeper = ServerProxy("http://localhost:8000", allow_none=True)
        
    def run(self):
        while True:
            seq = self._queue.get()
            try:
                self._beeper.beep(seq)
            except ConnectionError:
                pass

class MovementDetector:
    NUM_READINGS = 15
    MAX_READINGS = np.array([0.78868562, 0.76723892, 0.8220638,  0.03392057, 0.04504494, 0.03118985, 0.00269039, 0.00254637, 0.00263502]) * 10
    def __init__(self):
        self.index = 0
        self.matrix = np.zeros((self.NUM_READINGS, 9))
        self.full = False
    
    def add(self, values):
        self.matrix[self.index] = values
        self.index += 1
        std = np.std(self.matrix, axis=0)
        if self.index >= self.NUM_READINGS:
            self.full = True
            self.index = 0
        return (std > self.MAX_READINGS).any()
   

class IMUDataSrc(GstBase.PushSrc):
    __gstmetadata__ = ('IMU Data','Src', \
                      'IMU data packaging source', 'Phil Underwood')

    __gproperties__ = {}

    __gsttemplates__ = Gst.PadTemplate.new("src",
                                           Gst.PadDirection.SRC,
                                           Gst.PadPresence.ALWAYS,
                                           OCAPS)

    def __init__(self):
        if not DUMMY:
            self.en_pin = digitalio.DigitalInOut(board.D4)
            self.en_pin.switch_to_output(False)
            self.i2c_bus = busio.I2C(board.SCL, board.SDA, frequency=400_000)
            self.device = icm.ICM20948(self.i2c_bus)
        GstBase.PushSrc.__init__(self)
        self.info = GstVideo.VideoInfo()
        self.set_live(True)
        self.set_format(Gst.Format.BYTES)
        self.accumulator = 0
        self.beep_queue = Queue()
        self.beep_task = Beeper(self.beep_queue)
        self.beep_task.start()
        self.start_beep_done = False
        self.next_beep_due = 0
        self.movement = MovementDetector()


    def do_set_caps(self, caps):
        self.info.new_from_caps(caps)
        self.set_blocksize(self.info.size)
        self.set_do_timestamp(True)
        self.framerate = self.info.fps_n // self.info.fps_d
        if self.framerate==0:
            self.framerate = 30
        Gst.info(f"Framerate: {self.framerate} , {self.info.fps_n}, {self.info.fps_d}")
        if not DUMMY:
            self.device.gyro_data_rate = self.framerate
            accel_rate = (1125-self.framerate) // self.framerate
            self.device.accelerometer_data_rate = accel_rate
            if self.framerate > 50:
                mag_rate = icm.MagDataRate.RATE_100HZ
            elif self.framerate > 20:
                mag_rate = icm.MagDataRate.RATE_50HZ
            elif self.framerate > 10:
                mag_rate = icm.MagDataRate.RATE_20HZ
            else:
                mag_rate = icm.MagDataRate.RATE_10HZ
            self.device.magnetometer_data_rate = mag_rate
        self.next_frame_due = None
        return True


    def do_get_property(self, prop):
        Gst.info(f"Not Getting prop: {prop.name}")
        raise AttributeError('unknown property %s' % prop.name)

    def do_set_property(self, prop, value):
        Gst.info(f"Not Setting prop: {prop.name}")
        raise AttributeError('unknown property %s' % prop.name)

    def do_start(self):
        return True

    def do_gst_base_src_query(self, query):
        if query.type == Gst.QueryType.LATENCY:
            latency = 0
            query.set_latency(self.is_live, latency, Gst.CLOCK_TIME_NONE)
            res = True
        else:
            res = GstBase.PushSrc.do_query(self, query)
        return res

    def do_get_times(self, buf):
        end = 0
        start = 0
        if self.is_live:
            ts = buf.pts
            if ts != Gst.CLOCK_TIME_NONE:
                duration = buf.duration
                if duration != Gst.CLOCK_TIME_NONE:
                    end = ts + duration
                start = ts
        else:
            start = Gst.CLOCK_TIME_NONE
            end = Gst.CLOCK_TIME_NONE
        return start, end

    def do_is_seekable(self):
        return False

    def do_gst_base_src_create(self, offset, length, buf):
        if not self.start_beep_done:
            self.start_beep_done = True
            self.beep_queue.put(HAPPY)
            self.next_beep_due = time.time() + 3
        if self.next_frame_due is None:
            self.next_frame_due = time.time() + 1 / self.framerate
        else:
            now = time.time()
            if now < self.next_frame_due:
                time.sleep(self.next_frame_due-now)
            else:
                self.next_frame_due = now
            self.next_frame_due += 1 / self.framerate
        if DUMMY:
            randoms = np.random.random(9)
            data = struct.pack("<9d",*randoms)
        else:
            mag = self.device.magnetic
            accel = self.device.acceleration
            gyro = self.device.gyro
            movement = self.movement.add((*mag, *accel, *gyro))
            if not movement and time.time()> self.next_beep_due:
                self.beep_queue.put(BIP)
                self.next_beep_due = time.time() + 1
            data = bytearray(256)
            struct.pack_into("<9d?", data, 0, *mag, *accel, *gyro, movement)
        buf = Gst.Buffer.new_wrapped(data)

        return Gst.FlowReturn.OK, buf



__gstelementfactory__ = ("py_imu_data", Gst.Rank.NONE, IMUDataSrc)
