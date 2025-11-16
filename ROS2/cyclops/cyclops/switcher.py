import asyncio
from typing import Optional

import rclpy
from subprocess import Popen
from launch import LaunchService
from launch.launch_description_sources import get_launch_description_from_any_launch_file
from launch.logging import get_logger
from std_srvs.srv import Trigger

from autonode import Node, service


class SwitcherNode(Node):
    cal_fname: str = ""
    capture_fname: str = ""

    def __init__(self):
        super().__init__()
        self.launch_service: Optional[LaunchService] = None

    async def _reset_mode(self):
        if self.launch_service is not None:
            await self.launch_service.shutdown()
        self.launch_service = None

    async def start_mode(self, fname: str):
        await self._reset_mode()
        descriptor = get_launch_description_from_any_launch_file(fname)
        self.launch_service = LaunchService(debug=False)
        self.launch_service.include_launch_description(descriptor)
        logger = get_logger("launch")
        logger.info(f"starting {fname}")
        asyncio.create_task(self.launch_service.run_async())

    @service(Trigger)
    async def calibration_mode(self, req:Trigger.Request, rsp:Trigger.Response):
        await self.start_mode(self.cal_fname)
        rsp.success = True
        rsp.message = "Calibration mode started"
        return rsp

    @service(Trigger)
    async def capture_mode(self, req:Trigger.Request, rsp:Trigger.Response):
        await self.start_mode(self.capture_fname)
        rsp.success = True
        rsp.message = "Capture mode started"
        return rsp

    @service(Trigger)
    async def reset_mode(self, req:Trigger.Request, rsp:Trigger.Response):
        await self._reset_mode()
        rsp.success = True
        rsp.message = "Standby mode"
        return rsp

    async def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            await asyncio.sleep(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = SwitcherNode()
    asyncio.run(node.run())
    rclpy.shutdown()
