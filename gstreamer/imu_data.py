'''
Element that generates video data based on the IMU output

Requires numpy

Example pipeline:

gst-launch-1.0 py_audiotestsrc ! autoaudiosink
'''

import gi

gi.require_version('Gst', '1.0')
gi.require_version('GstBase', '1.0')
gi.require_version('GstVideo', '1.0')

from gi.repository import Gst, GLib, GObject, GstBase, GstVideo

try:
    import numpy as np
except ImportError:
    Gst.error('py_audiotestsrc requires numpy')
    raise

OCAPS = Gst.Caps.from_string ('video/x-raw, format=GRAY8, width=16, height=16, framerate=[30/1,60/1]')

DEFAULT_FRAMERATE = 50
DEFAULT_IS_LIVE = True
SAMPLES_PER_BUFFER = 1

class IMUDataSrc(GstBase.PushSrc):
    __gstmetadata__ = ('IMU Data','Src', \
                      'IMU data packaging source', 'Phil Underwood')

    __gproperties__ = {}

    __gsttemplates__ = Gst.PadTemplate.new("src",
                                           Gst.PadDirection.SRC,
                                           Gst.PadPresence.ALWAYS,
                                           OCAPS)

    def __init__(self):
        GstBase.PushSrc.__init__(self)
        self.info = GstVideo.VideoInfo()
        self.set_live(True)
        self.set_format(Gst.Format.BYTES)
        self.accumulator = 0

    def do_set_caps(self, caps):
        self.info.from_caps(caps)
        self.set_blocksize(self.info.size)
        self.set_do_timestamp(True)
        return True

    # def do_get_caps(self, caps):
    #     print(caps)
    #     print("getting caps")
    #     return super().do_get_caps(self,caps)

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
        try:
            my_buf = bytearray(256)
            my_buf[self.accumulator*17] = 255
            self.accumulator = (self.accumulator + 1) % self.info.width
            buf.fill(0, my_buf)
        except Exception as e:
            Gst.error("Mapping error: %s" % e)
            return Gst.FlowReturn.ERROR

        return Gst.FlowReturn.OK, buf


__gstelementfactory__ = ("py_imu_data", Gst.Rank.NONE, IMUDataSrc)