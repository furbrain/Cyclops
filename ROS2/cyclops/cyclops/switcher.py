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
from cyclops_interfaces.srv import SetMode, BeepPreset

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
        self.beeper = self.create_client(BeepPreset, "/beeper/preset")
        self.beep(BeepPreset.Request.HAPPY)


    def beep(self, tune):
        req = BeepPreset.Request()
        req.tune = tune
        self.beeper.call_async(req)

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

    async def _wait_for_atlas(self, timeout: float = 5.0):
        """Wait until the named node is no longer visible on the ROS2 graph."""
        deadline = self.get_clock().now() + rclpy.duration.Duration(seconds=timeout)
        atlas_path = CURRENT_DIR / "atlas.osa"
        while self.get_clock().now() < deadline:
            if atlas_path.exists():
                return True
            await asyncio.shield(asyncio.sleep(0.2))
        self.get_logger().warn(f'Timeout waiting for atlas to appear')
        return False

    async def _reset_mode(self):
        if self.launch_service is not None:
            self.beep(BeepPreset.Request.FINISH)
            await self.launch_service.shutdown()
            if self.current_mode=="capture":
                await self._wait_for_atlas()
            self.clean_recording_dir()
            self.beep(BeepPreset.Request.BIP)
        self.launch_service = None
        self.current_mode = "reset"

    async def start_mode(self, fname: str, mode: str):
        if self.current_mode == fname:
            return
        await self._reset_mode()
        descriptor = get_launch_description_from_any_launch_file(fname)
        self.launch_service = LaunchService(debug=False)
        self.launch_service.include_launch_description(descriptor)
        logger = get_logger("launch")
        logger.info(f"starting {fname}")
        asyncio.create_task(self.launch_service.run_async())
        self.current_mode = mode

    @service(SetMode)
    async def set_mode(self, req:SetMode.Request, rsp:SetMode.Response):
        self.get_logger().info(f"Switching mode to {req.mode}")
        if req.mode == "calibrate":
            await self.start_mode(self.cal_fname, req.mode)
            self.beep(BeepPreset.Request.BIP)
        elif req.mode == "capture":
            self.clean_recording_dir()
            await self.start_mode(self.capture_fname, req.mode)
            self.beep(BeepPreset.Request.HAPPY)
        else:
            await self._reset_mode()
        rsp.success = True
        self.get_logger().info(f"Switching mode complete")
        return rsp

    @service(SetMode)
    async def toggle_mode(self, req:SetMode.Request, rsp:SetMode.Response):
        if req.mode == self.current_mode:
            asyncio.ensure_future(self._reset_mode())
            rsp.success = True
            return rsp
        else:
            return await self.set_mode(req, rsp)


    async def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            await asyncio.sleep(0.01)


def main(args=None):
    rclpy.init(args=args)
    node = SwitcherNode()
    node.run_async()
