import gpiozero
import rclpy
from cyclops_interfaces.srv import SetMode

from autonode import Node

class Button(Node):
    def __init__(self):
        super().__init__()
        self.mode_client = self.create_client(SetMode, "toggle_mode")
        self.button = gpiozero.Button(26, bounce_time=0.05)
        self.button.when_pressed = self.trigger

    def trigger(self):
        req = SetMode.Request()
        req.mode="capture"
        self.mode_client.call_async(req)

    def cleanup(self):
        self.button.close()

def main():
    rclpy.init()
    node = Button()
    node.run()

if __name__=="__main__":
    main()