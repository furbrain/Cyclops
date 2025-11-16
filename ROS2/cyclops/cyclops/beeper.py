#!/usr/bin/env python3

import os, mmap, struct
import time
from typing import Optional, Tuple, Dict, Sequence
from xmlrpc.client import ServerProxy

import rclpy
from autonode import Node, service

from cyclops_interfaces.srv import BeepPreset, BeepPreset_Request, BeepPreset_Response
from cyclops_interfaces.srv import Beep, Beep_Request, Beep_Response

class BeeperNode(Node):
    TUNES = {
        BeepPreset_Request.HAPPY : (("C6", 50.0), ("E6", 50.0), ("G6", 50.0), ("C7", 50.0)),
        BeepPreset_Request.BIP: (("A7", 50),),
        BeepPreset_Request.BOP: (("C7", 50),),
        BeepPreset_Request.SAD: (("G6", 100), ("C6", 200)),
        BeepPreset_Request.FINISH: list(reversed((("C6", 50.0), ("E6", 50.0), ("G6", 50.0), ("C7", 50.0)))),
    }
    def __init__(self):
        super().__init__()
        self.b = ServerProxy('http://localhost:8123')

    def play_sequence(self, tune: Sequence[Tuple[str, float]]):
        self.b.beep(tune)

    @service(BeepPreset)
    def preset(self, request: BeepPreset_Request, response: BeepPreset_Response):
        if request.tune in self.TUNES:
            self.play_sequence(self.TUNES[request.tune])
        else:
            self.get_logger().warn(f"Unknown tune: {request.tune}")
        return response

    @service(Beep)
    def beep(self, request: Beep_Request, response: Beep_Response):
        tune = list(zip(request.notes, request.durations))
        self.play_sequence(tune)
        return response

def main(args=None):
    rclpy.init(args=args)
    b = BeeperNode()
    b.run()

if __name__ == "__main__":
    main()