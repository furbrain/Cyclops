#!/usr/bin/env python3

import os, mmap, struct
import time
from typing import Optional, Tuple, Dict, Sequence
from xmlrpc.client import ServerProxy

import rclpy
from .utils import SmartNode

from cyclops_interfaces.srv import BeepPreset, BeepPreset_Request, BeepPreset_Response
from cyclops_interfaces.srv import Beep, Beep_Request, Beep_Response

class BeeperNode(SmartNode):
    TUNES = {
        BeepPreset_Request.HAPPY : (("C6", 50.0), ("E6", 50.0), ("G6", 50.0), ("C7", 50.0)),
        BeepPreset_Request.BIP: (("A7", 50),),
        BeepPreset_Request.BOP: (("C7", 50),),
        BeepPreset_Request.SAD: (("G6", 100), ("C6", 200)),
        BeepPreset_Request.FINISH: list(reversed((("C6", 50.0), ("E6", 50.0), ("G6", 50.0), ("C7", 50.0)))),
    }
    def __init__(self, pin_a: int, pin_b: Optional[int] = None):
        super().__init__("Beeper")
        self.b = ServerProxy('http://localhost:8123')
        self.preset_service = self.create_service(BeepPreset, "preset", self.preset)
        self.beep_service = self.create_service(Beep, "beep", self.beep)

    def play_sequence(self, tune: Sequence[Tuple[str, float]]):
        self.b.beep(tune)

    def preset(self, request: BeepPreset_Request, response: BeepPreset_Response):
        if request.tune in self.TUNES:
            self.play_sequence(self.TUNES[request.tune])
        else:
            self.get_logger().warn(f"Unknown tune: {request.tune}")
        return response

    def beep(self, request: Beep_Request, response: Beep_Response):
        tune = list(zip(request.notes, request.durations))
        self.play_sequence(tune)
        return response

def main(args=None):
    rclpy.init(args=args)
    b = BeeperNode(13, 19)
    b.run()

if __name__ == "__main__":
    main()