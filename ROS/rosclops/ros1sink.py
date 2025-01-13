import pathlib
import sys

import gi

import rospy
from cam_info_mgr import CameraInfoManager
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Header

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject, GLib

gi.require_version('GstVideo', '1.0')
from gi.repository import GstVideo

_ALL_VIDEO_FORMATS = [GstVideo.VideoFormat.from_string(
    f.strip()) for f in GstVideo.VIDEO_FORMATS_ALL.strip('{ }').split(',')]


class ROS1Sink:
    def __init__(self, topic: str, spec: str, camera_info_url:str = "", jpeg: bool = True, frame:str = "odom"):
        self.jpeg = jpeg
        if camera_info_url:
            camera_info_url = 'file://' + str(pathlib.Path(camera_info_url).absolute())
        self.cam_info = CameraInfoManager(topic, url=camera_info_url)
        if jpeg:
            fmt = 'jpegparse ! image/jpeg'
        else:
            fmt = 'video/x-raw, format=RGB8'
        if jpeg:
            self.publisher = rospy.Publisher("gst_image/compressed", CompressedImage, queue_size=10)
        else:
            self.publisher = rospy.Publisher("gst_image", Image, queue_size=10)
        spec += f'! {fmt} ! appsink max-buffers=1'
        Gst.init([sys.argv[0]])
        self.pipeline = Gst.parse_launch(spec)
        self.bus = self.pipeline.get_bus()
        try:
            self.sink = list(self.pipeline.iterate_sinks()).pop(0)
        except TypeError:
            _, self.sink = self.pipeline.iterate_sinks().next()
        self.caps = None
        self.eos = False
        self.header = Header(frame_id=frame)
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
        while not rospy.is_shutdown():
            if self.eos:
                rospy.signal_shutdown("end of stream")
                break
            sample = self.sink.emit("pull-sample")
            if sample is not None:
                if self.caps is None:
                    self.caps = sample.get_caps()
                buf = sample.get_buffer()
                self.header.stamp = rospy.Time.now()
                if self.jpeg:
                    msg  = CompressedImage(self.header,'jpeg',buf.extract_dup(0, buf.get_size()))
                else:
                    msg = Image(self.header,self.size[1],self.size[0],"rgb8",False,
                                self.size[0]*3,buf.extract_dup(0, buf.get_size()))
                self.publisher.publish(msg)
            gst_msg = self.bus.pop_filtered(Gst.MessageType.EOS)
            if gst_msg:
                self.eos = True
                self.close()

if __name__=="__main__":
    import argparse
    parser = argparse.ArgumentParser(
        prog="ros1sink",
        description="Allows to publish arbitrary gstreamer streams as ROS topics")
    parser.add_argument('topic',help="name of the ROS topic to create")
    parser.add_argument('spec',help="the gstreamer spec to use")
    parser.add_argument('-u','--camera_info_url',
                        help="yaml file to store camera info in",
                        default='')
    parser.add_argument('-j','--jpeg',
                        help='create a compressed stream of jpegs - needs to use a jpeg source in spec',
                        action="store_true")
    parser.add_argument('-f','--frame',
                        help='name of the camera frame to use',
                        default='odom')
    opts = vars(parser.parse_args())
    with ROS1Sink(**opts) as sink:
        rospy.init_node('ros_sink')
        sink.run()
