'''
Element that generates video data based on the IMU output

Example pipeline:

gst-launch-1.0 py_imu_data ! video/x-raw,framerate=50 ! videoconvert ! autovideosink
'''
import time
import numpy as np
import ArducamDepthCamera as ac
import gi

gi.require_version('Gst', '1.0')
gi.require_version('GstBase', '1.0')
gi.require_version('GstVideo', '1.0')

from gi.repository import Gst, GLib, GObject, GstBase, GstVideo


OCAPS = Gst.Caps.from_string('video/x-raw,format=GRAY8,width=240,height=180,framerate=[15/1,30/1]')
DEFAULT_DEV = 0
DEFAULT_CONFIDENCE=30

class TOFDCamSrc(GstBase.PushSrc):
    __gstmetadata__ = ('TOFD Cam','Src', \
                      'Arducam TOF Camera Depth source', 'Phil Underwood')

    __gproperties__ = {
        "device": (int,
                 "Device",
                 "Index of v4l2 TOF device",
                 0,
                 GLib.MAXINT,
                 DEFAULT_DEV,
                 GObject.ParamFlags.READWRITE
                ),
        "confidence": (int,
                 "Confidence",
                 "Cut off to mark pixels as no reading",
                 0,
                 GLib.MAXINT,
                 DEFAULT_CONFIDENCE,
                 GObject.ParamFlags.READWRITE
                ),
    }

    __gsttemplates__ = Gst.PadTemplate.new("src",
                                           Gst.PadDirection.SRC,
                                           Gst.PadPresence.ALWAYS,
                                           OCAPS)

    def __init__(self):
        self.cam = ac.ArducamCamera()
        GstBase.PushSrc.__init__(self)
        self.info = GstVideo.VideoInfo()
        self.set_live(True)
        self.set_format(Gst.Format.BYTES)
        self.accumulator = 0
        self.device = DEFAULT_DEV
        self.confidence = DEFAULT_CONFIDENCE


    def do_set_caps(self, caps):
        self.info = GstVideo.VideoInfo.new_from_caps(caps)
        self.set_blocksize(self.info.size)
        self.set_do_timestamp(True)
        self.framerate = self.info.fps_n // self.info.fps_d
        if self.framerate==0:
            self.framerate = 30
        Gst.info(f"Framerate: {self.framerate} , Supplied: {self.info.fps_n} / {self.info.fps_d}")
        self.next_frame_due = None
        ret = self.cam.open(ac.TOFConnect.CSI, self.device)
        if ret != 0:
            print("TOF initialization failed. Error code:", ret)
            return
        ret = self.cam.start(ac.TOFOutput.DEPTH)
        if ret != 0:
            Gst.error("TOF start failed. Error code:", ret)
        return True


    def do_get_property(self, prop):
        if prop.name == 'device':
            return self.device
        elif prop.name == 'confidence':
            return self.confidence
        else:
            raise AttributeError('unknown property %s' % prop.name)

    def do_set_property(self, prop, value):
        if prop.name == 'device':
            self.device = value
        elif prop.name == 'confidence':
            self.confidence = value
        else:
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
        if self.next_frame_due is None:
            self.next_frame_due = time.time() + 1 / self.framerate
        else:
            now = time.time()
            if now < self.next_frame_due:
                time.sleep(self.next_frame_due-now)
            else:
                self.next_frame_due = now
            self.next_frame_due += 1 / self.framerate
        frame = self.cam.requestFrame(2000)
        if frame is not None and isinstance(frame, ac.DepthData):
            data = frame.getDepthData().copy()
            if self.confidence:
                confidence = frame.getConfidenceData().copy()
            self.cam.releaseFrame(frame)
            data = np.reshape(data, (180,240))
            data = data * 255/4000.0
            data = data.astype("uint8")
            if self.confidence:
                data[confidence < self.confidence] = 0
        else:
            Gst.error("Failed to get frame")
        buf = Gst.Buffer.new_wrapped(bytes(data))

        return Gst.FlowReturn.OK, buf



__gstelementfactory__ = ("py_TOFD_Cam", Gst.Rank.NONE, TOFDCamSrc)
