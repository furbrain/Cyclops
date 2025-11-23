import os
import time

import rclpy
from autonode import Node
from cyclops_interfaces.action import MakeModel
from rclpy.action.server import ServerGoalHandle, ActionServer
import subprocess

class Modeller(Node):
    def __init__(self):
        super().__init__()
        self._action = ActionServer(self, MakeModel, "make_model", self._do_action)
        self.process = None

    def _do_action(self, gh: ServerGoalHandle):
        self.gh = gh
        recording: str = gh.request.name
        style: int = gh.request.style
        if style:
            args = ['-r']
        else:
            args = []
        self.get_logger().info(f"Starting model {recording}")
        self.process = subprocess.Popen(['/home/pi/Cyclops/raspbian/make_model.py','-n', recording] + args,
                                        stdout = subprocess.PIPE, stderr=subprocess.STDOUT,
                                        bufsize=1, text=True)
        os.set_blocking(self.process.stdout.fileno(), False)
        while self.process.returncode is None:
            try:
                out = self.process.stdout.readline()
            except subprocess.TimeoutExpired:
                time.sleep(0.02)
                self.get_logger().info("timeout")
                self.process.poll()
                continue
            if out:
                out = out.strip()
                self.get_logger().info(f"Line: {out}")
                self.gh.publish_feedback(MakeModel.Feedback(output=out))
            self.process.poll()
        self.get_logger().info("subprocess complete")
        if self.process.returncode==0:
            result = MakeModel.Result(success=True)
            self.gh.succeed()
        else:
            result = MakeModel.Result(success=False)
            self.gh.succeed()
        return result

    def cleanup(self):
        if self.process:
            self.process.kill()

def main(args=None):
    rclpy.init(args=args)
    b = Modeller()
    rclpy.spin(b)

if __name__ == "__main__":
    main()