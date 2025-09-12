import os
from pathlib import Path

import rclpy
from .utils import SmartNode
from geometry_msgs.msg import TransformStamped
from cyclops_interfaces.srv import SetTransform
import yaml
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from rosidl_runtime_py import message_to_yaml, set_message_fields

class TF2Dynamic (SmartNode):
    def __init__(self):
        super().__init__("tf2_dynamic")
        self.frame = self.get_initial_param("frame","map")
        self.child_frame = self.get_initial_param("child_frame","child")
        url = self.get_initial_param("url", "")
        self.set_tf = self.create_service(SetTransform, "set_transform", self.set_transform)
        if url == "":
            if "ROS_HOME" in os.environ:
                ros_home = Path(os.environ.get("ROS_HOME"))
            else:
                ros_home = Path(os.environ.get("HOME")) / ".ros"
            self.url = Path(ros_home) / "transforms" / f'{self.frame}_{self.child_frame}.yaml'
        else:
            self.url = Path(url)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.load_transform()

    def load_transform(self):
        t = self.make_transform()
        try:
            with open(self.url) as f:
                data = yaml.safe_load(f)
                set_message_fields(t.transform, data)
        except IOError:
            self.get_logger().warn(f"No calibration found at {self.url}")
        self.tf_static_broadcaster.sendTransform(t)

    def make_transform(self) -> TransformStamped:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.frame
        t.child_frame_id = self.child_frame
        return t

    def set_transform(self, req: SetTransform.Request, rsp: SetTransform.Response):
        if not self.url.parent.exists():
            self.url.parent.mkdir(parents=True, exist_ok=True)
        try:
            with open(self.url,"w") as f:
                f.write(message_to_yaml(req.transform))
            t = self.make_transform()
            t.transform = req.transform
            self.tf_static_broadcaster.sendTransform(t)
            rsp.success = True
        except (IOError, yaml.YAMLError):
            rsp.success = False
        return rsp

def main():
    rclpy.init()
    node = TF2Dynamic()
    rclpy.spin(node)

if __name__=="__main__":
    main()