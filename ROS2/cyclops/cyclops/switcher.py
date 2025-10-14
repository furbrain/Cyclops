import asyncio
from typing import Optional

import rclpy
from subprocess import Popen
from launch import LaunchService
from launch.launch_description_sources import get_launch_description_from_any_launch_file
from launch.logging import get_logger
from std_srvs.srv import Trigger

from .utils import SmartNode

# have one item which is calibrate
# the other is main
# have a service to start either
# and one to stop both

class SwitchNode(SmartNode):
    def __init__(self):
        super().__init__("switcher")
        self.cal_fname = self.get_initial_param("cal_fname", "")
        self.capture_fname = self.get_initial_param("capture_fname", "")
        self.cal_service = self.create_service(Trigger, "calibration_mode", self.cal_mode)
        self.capture_service = self.create_service(Trigger, "capture_mode", self.capture_mode)
        self.reset_service = self.create_service(Trigger, "reset_mode", self.reset_mode)
        self.launch_service: Optional[LaunchService] = None

    async def _reset_mode(self):
        if self.launch_service is not None:
            await self.launch_service.shutdown()



    async def start_mode(self, fname: str):
        await self._reset_mode()
        descriptor = get_launch_description_from_any_launch_file(fname)
        self.launch_service = LaunchService(debug=False)
        self.launch_service.include_launch_description(descriptor)
        logger = get_logger("launch")
        logger.info(f"starting {fname}")
        asyncio.create_task(self.launch_service.run_async())

    async def cal_mode(self, req:Trigger.Request, rsp:Trigger.Response):
        await self.start_mode(self.cal_fname)
        rsp.success = True
        rsp.message = "Calibration mode started"
        return rsp

    async def capture_mode(self, req:Trigger.Request, rsp:Trigger.Response):
        await self.start_mode(self.capture_fname)
        rsp.success = True
        rsp.message = "Capture mode started"
        return rsp

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
    node = SwitchNode()
    asyncio.run(node.run())
    rclpy.shutdown()
