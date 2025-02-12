import pathlib
import sys

import gi

import rospy
from gst_camera.cam_info_mgr import CameraInfoManager
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Header

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject, GLib

gi.require_version('GstVideo', '1.0')
from gi.repository import GstVideo

_ALL_VIDEO_FORMATS = [GstVideo.VideoFormat.from_string(
    f.strip()) for f in GstVideo.VIDEO_FORMATS_ALL.strip('{ }').split(',')]


class Camera:
    def __init__(self, name: str, spec: str, jpeg: bool = True):
        self.jpeg = jpeg
        self.cam_info = CameraInfoManager(name)
        self.cam_info.loadCameraInfo()
        if jpeg:
            fmt = 'jpegparse ! image/jpeg'
        else:
            fmt = 'video/x-raw, format=RGB8'
        if jpeg:
            self.publisher = rospy.Publisher("~image_raw/compressed", CompressedImage, queue_size=1)
        else:
            self.publisher = rospy.Publisher("~image_raw", Image, queue_size=1)
        spec += f' ! {fmt} ! queue max-size-buffers=1 leaky=downstream ! appsink max-buffers=1'
        Gst.init([sys.argv[0]])
        self.pipeline = Gst.parse_launch(spec)
        self.bus = self.pipeline.get_bus()
        try:
            self.sink = list(self.pipeline.iterate_sinks()).pop(0)
        except TypeError:
            _, self.sink = self.pipeline.iterate_sinks().next()
        self.caps = None
        self.eos = False
        self.header = Header(frame_id=self.cam_info.frame)
        #self.rate = rospy.Rate(10)
        self.start()

    def __enter__(self):
        return self

    def __exit__(self, type, value, cb):
        self.close()


    def start(self):
        self.pipeline.set_state(Gst.State.PLAYING)

    def close(self):
        self.pipeline.set_state(Gst.State.NULL)

    def get_caps(self):
        caps_format = self.caps.get_structure(0)
        frmt_str = caps_format.get_value('format')
        video_format = GstVideo.VideoFormat.from_string(frmt_str)
        self.size = caps_format.get_value('width'), caps_format.get_value('height')

    def run(self):
        self.rate = rospy.Rate(30)
        print("running")
        while not rospy.is_shutdown():
            if self.eos:
                print("end of stream")
                rospy.signal_shutdown("end of stream")
                break
            sample = self.sink.emit("pull-sample")
            if sample is not None:
                if self.caps is None:
                    self.caps = sample.get_caps()
                buf = sample.get_buffer()
                self.header.stamp = rospy.Time.now()
                tmp_buf = buf.extract_dup(0,buf.get_size())
                if self.jpeg:
                    msg  = CompressedImage(self.header,'jpeg',tmp_buf)
                else:
                    msg = Image(self.header,self.size[1],self.size[0],"rgb8",False,
                                self.size[0]*3,tmp_buf)
                self.publisher.publish(msg)
                #GLib.free(tmp_buf)
            gst_msg = self.bus.pop_filtered(Gst.MessageType.EOS)
            if gst_msg:
                self.eos = True
                self.close()
            self.rate.sleep()

if __name__=="__main__":
    import argparse
    parser = argparse.ArgumentParser(
        prog="ros1sink",
        description="Allows to publish arbitrary gstreamer streams as ROS topics")
    parser.add_argument('spec',help="the gstreamer spec to use")
    parser.add_argument('-j','--jpeg',
                        help='create a compressed stream of jpegs - needs to use a jpeg source in spec',
                        action="store_true")
    print(sys.argv)
    clean_args = rospy.myargv()
    print(clean_args)
    opts = vars(parser.parse_args(clean_args[1:]))
    rospy.init_node('gst_camera')
    with Camera(rospy.get_name(), **opts) as sink:
        sink.run()
