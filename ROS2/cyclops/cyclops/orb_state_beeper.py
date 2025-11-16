import asyncio
import time

import rclpy
from autonode import Node, subscription
from cyclops_interfaces.srv import BeepPreset, BeepPreset_Request
from orb_slam3.msg import State

class OrbBeeper(Node):
    def __init__(self):
        super().__init__()
        self.beeper = self.create_client(BeepPreset, "preset")
        self.last_noise_tm = 0
        self.last_state = 0

    def elapsed(self, seconds: float):
        now = time.monotonic_ns()
        if self.last_noise_tm + int(seconds*1e9) < now:
            self.last_noise_tm = now
            return True
        return False

    @subscription(State)
    async def state(self, msg: State):
        req = BeepPreset_Request()
        if msg.state==msg.OK:
            if self.elapsed(1.0):
                req.tune = req.BOP
                self.beeper.call_async(req)
        elif msg.state==msg.RECENTLY_LOST:
            if self.elapsed(0.333):
                req.tune = req.BIP
                self.beeper.call_async(req)
        elif msg.state==msg.LOST:
            req.tune = req.SAD
            self.beeper.call_async(req)
        self.last_state = msg.state

def main(args=None):
    rclpy.init(args=args)
    b = OrbBeeper()
    b.run_async()

if __name__ == "__main__":
    main()