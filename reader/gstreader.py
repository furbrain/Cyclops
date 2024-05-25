"""A simple class that takes a gstreamer input pipeline. Allows reading and seeking"""
import sys
import struct
import gi
import numpy as np

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
    def __init__(self, spec: str, autostart: bool=True):
        Gst.init([sys.argv[0]])
        self.pipeline = Gst.parse_launch(spec)
        self.bus = self.pipeline.get_bus()
        self.sink = list(self.pipeline.iterate_sinks()).pop(0)
        self.caps = None
        self.eos = False
        if autostart:
            self.start()

    def __enter__(self):
        return self

    def __exit__(self, type, value, cb):
        self.close()

    def start(self):
        self.pipeline.set_state(Gst.State.PLAYING)
    
    def close(self):
        self.pipeline.set_state(Gst.State.NULL)
        
    
    @staticmethod
    def converter(data):
        return data.copy()
        
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
            tm = buf.pts/1_000_000_000
            output = tm, result
        else:
            output = None, None
        message = self.bus.pop_filtered(Gst.MessageType.EOS)
        if message:
            self.eos = True
            self.close()
        return output
        
        
class IMUReader(GstReader):
    @staticmethod
    def converter(data):
        return struct.unpack('<9d', data[:72])
        
class VidReader(GstReader):
    def __init__(self, spec: str, autostart: bool=True):
        super().__init__(spec, autostart)
        self.vid_size = None
        self.channels = None
        self.dtype = None

    def get_caps(self):
        caps_format = self.caps.get_structure(0)
        frmt_str = caps_format.get_value('format') 
        video_format = GstVideo.VideoFormat.from_string(frmt_str)
        self.size = caps_format.get_value('width'), caps_format.get_value('height')
        self.channels = get_num_channels(video_format)
        self.dtype = get_np_dtype(video_format)     

    def converter(self, data):
        if self.vid_size is None:
            self.get_caps()
        data = data[:]
        array = np.ndarray(shape=(self.size[1], self.size[0], self.channels), buffer=data, dtype=self.dtype)
        return np.squeeze(array)
