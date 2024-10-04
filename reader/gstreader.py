"""A simple class that takes a gstreamer input pipeline. Allows reading and seeking"""
import sys
import struct

import cv2
import gi
import numpy as np

TS_PER_S = 1_000_000_000

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject, GLib

gi.require_version('GstVideo', '1.0')
from gi.repository import GstVideo

_ALL_VIDEO_FORMATS = [GstVideo.VideoFormat.from_string(
    f.strip()) for f in GstVideo.VIDEO_FORMATS_ALL.strip('{ }').split(',')]
    

def _get_num_channels(fmt: GstVideo.VideoFormat) -> int:
    """
        -1: means complex format (YUV, ...)
    """
    frmt_info = GstVideo.VideoFormat.get_info(fmt)
    
    # temporal fix
    if fmt == GstVideo.VideoFormat.BGRX:
        return 4
    
    if frmt_info.flags & GstVideo.VideoFormatFlags.ALPHA:
        return 4

    if frmt_info.flags & GstVideo.VideoFormatFlags.RGB:
        return 3

    if frmt_info.flags & GstVideo.VideoFormatFlags.GRAY:
        return 1

    return -1


_ALL_VIDEO_FORMAT_CHANNELS = {fmt: _get_num_channels(fmt) for fmt in _ALL_VIDEO_FORMATS}


def get_num_channels(fmt: GstVideo.VideoFormat):
    return _ALL_VIDEO_FORMAT_CHANNELS[fmt]

_DTYPES = {
    16: np.int16,
}


def get_np_dtype(fmt: GstVideo.VideoFormat) -> np.number:
    format_info = GstVideo.VideoFormat.get_info(fmt)
    return _DTYPES.get(format_info.bits, np.uint8)




class GstReader:
    SPEC = "" # this needs to be overriden in children
    def __init__(self, spec: str, autostart: bool=True):
        Gst.init([sys.argv[0]])
        self.pipeline = Gst.parse_launch(spec)
        self.bus = self.pipeline.get_bus()
        try:
            self.sink = list(self.pipeline.iterate_sinks()).pop(0)
        except TypeError:
            _, self.sink = self.pipeline.iterate_sinks().next()
        self.caps = None
        self.eos = False
        self.channels = None
        self.dtype = None
        if autostart:
            self.start()

    def __enter__(self):
        return self

    def __exit__(self, type, value, cb):
        self.close()

    def __iter__(self):
        while True:
            tm, frame = self.get_frame()
            if tm is None:
                break
            yield tm, frame

    def start(self):
        self.pipeline.set_state(Gst.State.PLAYING)
    
    def close(self):
        self.pipeline.set_state(Gst.State.NULL)

    def get_caps(self):
        caps_format = self.caps.get_structure(0)
        frmt_str = caps_format.get_value('format')
        video_format = GstVideo.VideoFormat.from_string(frmt_str)
        self.size = caps_format.get_value('width'), caps_format.get_value('height')
        self.channels = max(get_num_channels(video_format),1)
        self.dtype = get_np_dtype(video_format)

    
    def converter(self, data):
        self.get_caps()
        data = data[:]
        array = np.ndarray(shape=(self.size[1], self.size[0], self.channels), buffer=data, dtype=self.dtype)
        return np.squeeze(array)
        
    def get_frame(self):
        if self.eos:
            return None, None
        sample = self.sink.emit("pull-sample")
        if sample is not None:
            if self.caps is None:
                self.caps = sample.get_caps()
            buf = sample.get_buffer()
            buffer_copy = buf.extract_dup(0, buf.get_size())
            result = self.converter(buffer_copy)
            tm = buf.pts / TS_PER_S
            output = tm, result
        else:
            output = None, None
        message = self.bus.pop_filtered(Gst.MessageType.EOS)
        if message:
            self.eos = True
            self.close()
        return output

    def seek(self, t:float):
        self.get_frame()
        t *= TS_PER_S
        self.sink.seek_simple(Gst.Format.TIME, (Gst.SeekFlags.FLUSH | Gst.SeekFlags.ACCURATE), t)

    @classmethod
    def from_filename(cls, filename, sync=False, gray=False, **kwargs):
        if sync:
            sync_txt = "true"
        else:
            sync_txt = "false"
        if gray:
            color = "GRAY8"
        else:
            color = "BGR"
        return cls(cls.SPEC.format(filename=filename, sync=sync_txt, color=color),**kwargs)
        
class IMUReader(GstReader):
    SPEC = """filesrc location="{filename}" ! matroskademux name=demux !
                            video/x-raw ! appsink sync={sync}"""
    @staticmethod
    def converter(data):
        return struct.unpack('<9d', data[:72])
        
class VidReader(GstReader):
    SPEC = """filesrc location="{filename}" ! matroskademux ! video/x-h264 ! decodebin !
                            videoconvert ! video/x-raw,format={color} ! appsink sync={sync}"""


class TOFReader(GstReader):
    SPEC = '''filesrc location="{filename}" ! matroskademux !
          video/x-raw,width=240,format=GRAY8 ! appsink sync={sync}'''

    def __init__(self, spec: str, autostart: bool=True, as_uint8: bool = False, absolute=False):
        super().__init__(spec, autostart)
        self.as_uint8 = as_uint8
        self.absolute = absolute

    def converter(self, data):
        self.get_caps()
        if self.size[0] == 960:
            data = np.ndarray(shape = (360,960), buffer=data, dtype="uint8")
            if self.as_uint8:
                data = self.I4202raw(data, True)
                output = np.zeros_like(data, dtype="uint8")
                output = cv2.normalize(data, output, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            else:
                output = self.I4202raw(data, False)
                return output
        elif self.size[0] == 480:
            if self.absolute:
                data = np.ndarray(shape = (180,240), buffer=data, dtype="int16")
                return data
            data = np.ndarray(shape = (180,480), buffer=data, dtype="int8")
            Q, I  = [data[:, i * 240:(i + 1) * 240].astype("float64") for i in range(2)]
            # Q = phase[1] - phase[3]
            # I = phase[2] - phase[0]
            bad_pixels = np.logical_and(Q == 0, I == 0)
            # print(np.count_nonzero(bad_pixels))
            distance = np.arctan2(Q, I) % (np.pi * 2)
            confidence = np.sqrt(Q ** 2 + I ** 2)
            distance *= 2 / np.pi
            distance[bad_pixels] = np.nan
            return Q
        elif self.size[0] == 240:
            data = np.ndarray(shape = (180,240), buffer=data, dtype="uint8")
            data  = data.astype("float64") * 4000.0/255
            return data
        else:
            return super().converter(data)
