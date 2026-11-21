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
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from tactical_frog.srv import SetMode, BeepPreset

from autonode import Node, service

CURRENT_DIR = Path("/data/current")
TRIPS_DIR = Path("/data/trips")

class SwitcherNode(Node):
    cal_fname: str = ""
    capture_fname: str = ""

    def __init__(self):
        super().__init__()
        self.launch_service: Optional[LaunchService] = None
        self._current_mode: str = "reset"
        self.mode_lock = asyncio.Lock()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.mode_pub = self.create_publisher(String, "mode", latched)
        # republish periodically too, in case a subscriber doesn't use transient_local
        self.create_timer(2.0, self.publish_mode)
        self.publish_mode()
        self.clean_recording_dir()
        self.beeper = self.create_client(BeepPreset, "/beeper/preset")
        self.beep(BeepPreset.Request.HAPPY)


    @property
    def current_mode(self) -> str:
        return self._current_mode

    @current_mode.setter
    def current_mode(self, mode: str):
        self._current_mode = mode
        self.publish_mode()

    def publish_mode(self):
        self.mode_pub.publish(String(data=self._current_mode))

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
            was_capturing = self.current_mode == "capture"
            self.current_mode = "stopping"
            self.beep(BeepPreset.Request.FINISH)
            await self.launch_service.shutdown()
            if was_capturing:
                await self._wait_for_atlas()
            self.clean_recording_dir()
            self.beep(BeepPreset.Request.BIP)
        self.launch_service = None
        self.current_mode = "reset"

    async def start_mode(self, fname: str, mode: str):
        if self.current_mode == mode:
            return
        await self._reset_mode()
        descriptor = get_launch_description_from_any_launch_file(fname)
        self.launch_service = LaunchService(debug=False)
        self.launch_service.include_launch_description(descriptor)
        logger = get_logger("launch")
        logger.info(f"starting {fname}")
        asyncio.create_task(self.launch_service.run_async())
        self.current_mode = mode

    async def _do_set_mode(self, mode: str):
        # Runs on the asyncio loop: rclpy's executor can't await asyncio futures,
        # so launch shutdown / atlas waiting must not happen inside a service callback.
        async with self.mode_lock:
            if mode == self.current_mode:
                return
            self.get_logger().info(f"Switching mode to {mode}")
            try:
                if mode == "calibrate":
                    await self.start_mode(self.cal_fname, mode)
                    self.beep(BeepPreset.Request.BIP)
                elif mode == "capture":
                    self.clean_recording_dir()
                    await self.start_mode(self.capture_fname, mode)
                    self.beep(BeepPreset.Request.HAPPY)
                else:
                    await self._reset_mode()
            except Exception as e:
                self.get_logger().error(f"Error switching to mode {mode}: {e!r}")
                self.publish_mode()
                return
            self.get_logger().info(f"Switching mode to {mode} complete")

    def _request_mode(self, mode: str) -> bool:
        if self.current_mode == "stopping":
            self.get_logger().warn(f"Mode change to {mode} requested while still stopping; ignoring")
            return False
        asyncio.ensure_future(self._do_set_mode(mode))
        return True

    @service(SetMode)
    def set_mode(self, req:SetMode.Request, rsp:SetMode.Response):
        """Request a mode change. Returns immediately; progress is published on /mode."""
        rsp.success = self._request_mode(req.mode)
        return rsp

    @service(SetMode)
    def toggle_mode(self, req:SetMode.Request, rsp:SetMode.Response):
        mode = "reset" if req.mode == self.current_mode else req.mode
        rsp.success = self._request_mode(mode)
        return rsp

    async def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            await asyncio.sleep(0.01)


def main(args=None):
    rclpy.init(args=args)
    node = SwitcherNode()
    node.run_async()
