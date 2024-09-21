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


OCAPS = Gst.Caps.from_string('video/x-raw,format=GRAY8,width=480,height=180,framerate=[30/1,31/1]')
DEFAULT_DEV = 0

def get_upper_byte_rounded(d: np.array):
    return ((d+128) // 256).astype("int8")


def mungulator(d: np.ndarray):
    d = d.astype("int32")
    phase = [d[:,i*240:(i+1)*240] for i in range(4)]
    Q = phase[1] - phase[3]
    I = phase[2] - phase[0]
    magnitude = Q**2 + I**2
    bad_pixels = magnitude < 300
    stack = np.dstack((Q,I))
    mx = np.maximum(np.abs(Q), np.abs(I))
    mx = np.maximum(mx, 1)
    multiplier = (32700 / mx).astype("int16")
    multiplier = np.minimum(multiplier, 255)
    Q = get_upper_byte_rounded(Q * multiplier)
    I = get_upper_byte_rounded(Q * multiplier)
    Q[bad_pixels] = 0
    I[bad_pixels] = 0
    output = np.hstack((Q,I))
    return output



def raw2I420(data: np.ndarray):
    #data is a numpy array of int16
    #first convert to uint16 nicely
    height = data.shape[0]
    data  = (data+0x8000).astype("uint16")
    #data  = data.astype("uint16")
    #left shift
    data >>= 4
    y = data.astype("uint8")
    nibbles = (data >> 4).astype("uint8") & 0xf0
#    uv = nibbles[:height//2] + (nibbles[height//2:] << 4)
#    out = np.vstack((y,uv))
    out = np.vstack((y,nibbles))
    return out
    
def I4202raw(data: np.ndarray, visualise: bool):
    third_height = data.shape[0]//3
    sixth_height = third_height//2
    y = data[:third_height*2]
    uv = data[third_height*2:]
    nibs_lower = uv >> 4
    nibs_upper = uv & 0xf
    nibbles = np.vstack((nibs_upper, nibs_lower))
    out = (nibbles.astype("uint16") << 8) + y
    out <<=4
    if not visualise:
        out = (out-0x8000).astype("int16")
    return out.astype("int16")
   

class TOFCamSrc(GstBase.PushSrc):
    __gstmetadata__ = ('TOF Cam','Src', \
                      'Arducam TOF Camera source', 'Phil Underwood')

    __gproperties__ = {
        "device": (int,
                 "Device",
                 "Index of v4l2 TOF device",
                 0,
                 GLib.MAXINT,
                 DEFAULT_DEV,
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


    def do_set_caps(self, caps):
        self.info.new_from_caps(caps)
        self.set_blocksize(self.info.size)
        self.set_do_timestamp(True)
        self.framerate = self.info.fps_n // self.info.fps_d
        if self.framerate==0:
            self.framerate = 30
        Gst.info(f"Framerate: {self.framerate} , {self.info.fps_n}, {self.info.fps_d}")
        self.next_frame_due = None
        ret = self.cam.open(ac.TOFConnect.CSI, self.device)
        if ret != 0:
            print("TOF initialization failed. Error code:", ret)
            return
        ret = self.cam.start(ac.TOFOutput.RAW)
        if ret != 0:
            Gst.error("TOF start failed. Error code:", ret)
        return True


    def do_get_property(self, prop):
        if prop.name == 'device':
            return self.device
        else:
            raise AttributeError('unknown property %s' % prop.name)

    def do_set_property(self, prop, value):
        if prop.name == 'device':
            self.device = value
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
        if frame is not None and isinstance(frame, ac.RawData):
            data = frame.getRawData().copy()
            self.cam.releaseFrame(frame)
            data = np.reshape(data, (180,960))
            data = mungulator(data)
        else:
            Gst.error("Failed to get frame")
        buf = Gst.Buffer.new_wrapped(bytes(data))

        return Gst.FlowReturn.OK, buf



__gstelementfactory__ = ("py_TOF_Cam", Gst.Rank.NONE, TOFCamSrc)
