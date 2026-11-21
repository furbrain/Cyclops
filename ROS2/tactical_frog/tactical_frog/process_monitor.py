#!/usr/bin/env python3
"""
ROS2 node that periodically publishes memory usage statistics for a named process.

The process list is only scanned (expensive) when no matching process is currently
known. Once a PID is found it is cached; each tick only reads /proc/<pid>/statm,
which is a single cheap kernel call. If the process dies the cache is cleared and
a scan is triggered on the next tick.

Parameters:
    process_name (str)  : Name (or substring) of the process to monitor (default: 'python3')
    rate_hz      (float): Sampling rate in Hz (default: 1.0)

Published topics:
    ~/memory_usage (diagnostic_msgs/DiagnosticStatus) : Human-readable status with key metrics
    ~/rss_bytes    (std_msgs/Int64)                   : Resident Set Size in bytes
    ~/vms_bytes    (std_msgs/Int64)                   : Virtual Memory Size in bytes
"""
from typing import Optional

import rclpy
from autonode import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, SetParametersResult

from std_msgs.msg import Int64
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue, DiagnosticArray

import psutil


def _scan_for_process(name: str) -> Optional[psutil.Process]:
    """Walk /proc once and return the first process whose name or cmdline contains *name*."""
    for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
        try:
            if name in proc.name():
                return proc
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    return None


class ProcessMemoryMonitorNode(Node):
    rate: float = 1.0
    process_name: str = "orb_slam3"

    def __init__(self):
        super().__init__()

        self._proc: Optional[psutil.Process] = None
        self._pub_status = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self._pub_rss    = self.create_publisher(Int64,            '~/rss_bytes',    10)
        self._pub_vms    = self.create_publisher(Int64,            '~/vms_bytes',    10)

        # ------------------------------------------------------------------ #
        # Timer (recreated when rate_hz changes)
        # ------------------------------------------------------------------ #
        self._timer = self.create_timer(1.0/self.rate, self.poll)
        self.get_logger().info(
            f"Monitoring '{self.process_name}' at {self.rate:.2f} Hz"
        )

    # ---------------------------------------------------------------------- #
    # Timer callback
    # ---------------------------------------------------------------------- #
    def poll(self):
        status_msg             = DiagnosticStatus()
        status_msg.name        = f'process_memory_monitor/{self.process_name}'
        status_msg.hardware_id = self.process_name

        # --- Ensure we have a live process handle -------------------------- #
        if self._proc is None:
            self._proc = _scan_for_process(self.process_name)
            if self._proc is None:
                status_msg.level   = DiagnosticStatus.WARN
                status_msg.message = f"No process found matching '{self.process_name}'"
                self.get_logger().warn(status_msg.message, throttle_duration_sec=5.0)
                return
            self.get_logger().info(
                f"Found '{self.process_name}' — PID {self._proc.pid}"
            )

        # --- Read memory from the cached handle (cheap: /proc/<pid>/statm) - #
        try:
            mem = self._proc.memory_info()
        except psutil.NoSuchProcess:
            self.get_logger().warn(
                f"PID {self._proc.pid} ('{self.process_name}') has died — will rescan"
            )
            self._proc = None
            status_msg.level   = DiagnosticStatus.WARN
            status_msg.message = f"Process '{self.process_name}' died"
            self._pub_status.publish(status_msg)
            return
        except psutil.AccessDenied:
            self.get_logger().error(
                f"Access denied reading memory for PID {self._proc.pid}", throttle_duration_sec=10.0
            )
            return

        # --- Publish -------------------------------------------------------- #
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status_msg.level   = DiagnosticStatus.OK
        status_msg.message = f"Monitoring PID {self._proc.pid}"
        status_msg.values  = [
            KeyValue(key='pid',       value=str(self._proc.pid)),
            KeyValue(key='rss_bytes', value=str(mem.rss)),
            KeyValue(key='vms_bytes', value=str(mem.vms)),
            KeyValue(key='rss_human', value=_fmt_bytes(mem.rss)),
            KeyValue(key='vms_human', value=_fmt_bytes(mem.vms)),
        ]
        array.status =  [status_msg]

        self.get_logger().debug(
            f"[{self.process_name} PID {self._proc.pid}] "
            f"RSS={_fmt_bytes(mem.rss)}  VMS={_fmt_bytes(mem.vms)}"
        )

        self._pub_status.publish(array)
        self._pub_rss.publish(Int64(data=mem.rss))
        self._pub_vms.publish(Int64(data=mem.vms))


def _fmt_bytes(n: int) -> str:
    for unit in ('B', 'KiB', 'MiB', 'GiB'):
        if n < 1024:
            return f'{n:.1f} {unit}'
        n /= 1024
    return f'{n:.1f} TiB'


def main(args=None):
    rclpy.init(args=args)
    node = ProcessMemoryMonitorNode()
    node.run()


if __name__ == '__main__':
    main()