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
        self.url = self.get_url_from_param("url", f'transforms/{self.frame}_{self.child_frame}.yaml')
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.transform = self.load_transform()
        self.timer = self.create_timer(0.1, self.publish)
        self.service = self.create_service(SetTransform,
                                           f"{self.frame}_{self.child_frame}/set_transform",
                                           self.set_transform)

    def load_transform(self):
        t = self.make_transform()
        try:
            with open(self.url) as f:
                data = yaml.safe_load(f)
                set_message_fields(t.transform, data)
        except IOError:
            self.get_logger().warn(f"No calibration found at {self.url}")
            return None
        return t

    def publish(self):
        if self.transform is not None:
            self.tf_static_broadcaster.sendTransform(self.transform)

    def make_transform(self) -> TransformStamped:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.frame
        t.child_frame_id = self.child_frame
        return t

    def set_transform(self, req: SetTransform.Request, rsp: SetTransform.Response):
        self.get_logger().info("writing transform")
        if not self.url.parent.exists():
            self.url.parent.mkdir(parents=True, exist_ok=True)
        try:
            with open(self.url,"w") as f:
                f.write(message_to_yaml(req.transform))
            t = self.make_transform()
            t.transform = req.transform
            self.transform = t
            self.tf_static_broadcaster.sendTransform(t)
            self.get_logger().info("writing transform completed")
            rsp.success = True
        except (IOError, yaml.YAMLError) as e:
            self.get_logger().warn(f"Error writing yaml file: {e}")
            rsp.success = False
        return rsp

def main():
    rclpy.init()
    node = TF2Dynamic()
    node.run()

if __name__=="__main__":
    main()