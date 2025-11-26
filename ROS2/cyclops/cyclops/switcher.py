import asyncio
from typing import Optional
from pathlib import Path
import re
import shutil

import rclpy
from subprocess import Popen
from launch import LaunchService
from launch.launch_description_sources import get_launch_description_from_any_launch_file
from launch.logging import get_logger
from cyclops_interfaces.srv import SetMode

from autonode import Node, service

CURRENT_DIR = Path("/data/current")
TRIPS_DIR = Path("/data/trips")

class SwitcherNode(Node):
    cal_fname: str = ""
    capture_fname: str = ""

    def __init__(self):
        super().__init__()
        self.launch_service: Optional[LaunchService] = None
        self.current_mode: str = "reset"
        self.clean_recording_dir()


    def get_latest_trip_number(self) -> int:
        pattern = re.compile(r"^trip_(\d+)$")

        numbers = []
        for p in TRIPS_DIR.iterdir():
            m = pattern.match(p.name)
            if m:
                numbers.append(int(m.group(1)))
        return max(numbers) if numbers else 1

    def clean_recording_dir(self):
        if (CURRENT_DIR / "recording").exists():
            trip_counter = self.get_latest_trip_number() + 1
            dest = f"trip_{trip_counter:03d}"
            self.get_logger().info(f"Moving current recording to {dest}")
            CURRENT_DIR.rename(TRIPS_DIR / dest)
            self.get_logger().info(f"current dir is {CURRENT_DIR}")
        shutil.rmtree(CURRENT_DIR, ignore_errors=True)
        CURRENT_DIR.mkdir(parents=True)

    async def _reset_mode(self):
        if self.launch_service is not None:
            await self.launch_service.shutdown()
            self.clean_recording_dir()
        self.launch_service = None
        self.current_mode = "reset"

    async def start_mode(self, fname: str):
        if self.current_mode == fname:
            return
        await self._reset_mode()
        descriptor = get_launch_description_from_any_launch_file(fname)
        self.launch_service = LaunchService(debug=False)
        self.launch_service.include_launch_description(descriptor)
        logger = get_logger("launch")
        logger.info(f"starting {fname}")
        asyncio.create_task(self.launch_service.run_async())
        self.current_mode = fname

    @service(SetMode)
    async def set_mode(self, req:SetMode.Request, rsp:SetMode.Response):
        if req.mode == "calibrate":
            await self.start_mode(self.cal_fname)
        elif req.mode == "capture":
            self.clean_recording_dir()
            await self.start_mode(self.capture_fname)
        else:
            await self._reset_mode()
        rsp.success = True
        return rsp

    async def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            await asyncio.sleep(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = SwitcherNode()
    node.run_async()
