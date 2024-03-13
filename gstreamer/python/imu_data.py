'''
Element that generates video data based on the IMU output

Example pipeline:

gst-launch-1.0 py_imu_data ! video/x-raw,framerate=50 ! videoconvert ! autovideosink
'''
import time

import gi

gi.require_version('Gst', '1.0')
gi.require_version('GstBase', '1.0')
gi.require_version('GstVideo', '1.0')

from gi.repository import Gst, GLib, GObject, GstBase, GstVideo
import struct

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

    def do_set_caps(self, caps):
        self.info.from_caps(caps)
        self.set_blocksize(self.info.size)
        self.set_do_timestamp(True)
        self.framerate = self.info.fps_n // self.info.fps_d
        if not DUMMY and False:
            self.device.gyro_data_rate = self.framerate
            accel_rate = (1125-self.framerate) // self.framerate
            self.device.accelerometer_data_rate = accel_rate
            if self.framerate > 50:
                mag_rate = adafruit_icm20x.MagDataRate.RATE_100HZ
            elif self.framerate > 20:
                mag_rate = adafruit_icm20x.MagDataRate.RATE_50HZ
            elif self.framerate > 10:
                mag_rate = adafruit_icm20x.MagDataRate.RATE_20HZ
            else:
                mag_rate = adafruit_icm20x.MagDataRate.RATE_10HZ
            self.device.magnetometer_data_rate = mag_rate
        self.next_frame_due = None
        return True


    def do_get_property(self, prop):
        raise AttributeError('unknown property %s' % prop.name)

    def do_set_property(self, prop, value):
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

    def do_gst_push_src_fill(self, buf):
        # work out what to do with pts...
        if self.next_frame_due is None:
            self.next_frame_due = time.time() + 1 / self.framerate
        else:
            now = time.time()
            if now < self.next_frame_due:
                time.sleep(self.next_frame_due-now)
            else:
                self.next_frame_due = now
            self.next_frame_due += 1 / self.framerate
        try:
            if DUMMY:
                randoms = np.random.random(9)
                data = struct.pack("<9d",*randoms)
            else:
                mag = self.device.magnetic
                accel = self.device.acceleration
                gyro = self.device.gyro
                data = struct.pack("<9d", *mag, *accel, *gyro)
            buf.fill(0, data)
        except Exception as e:
            Gst.error("Mapping error: %s" % e)
            return Gst.FlowReturn.ERROR

        return Gst.FlowReturn.OK, buf


__gstelementfactory__ = ("py_imu_data", Gst.Rank.NONE, IMUDataSrc)
