#!/usr/bin/env python3
from dataclasses import dataclass
from typing import Dict
from ctypes import c_int16
import smbus2
import rclpy
from autonode import Node
from sensor_msgs.msg import BatteryState


@dataclass
class RegDetails:
    unit: str
    reg_address: int

CHANNEL = 3

class Battery:
    CONVERSION: Dict[str, float] = {
        'capacity': 0.5 * 1e-3,
        'percentage': 1.0/25600.0,
        'voltage': 78.125 * 1e-6,
        'current': 156.25 * 1e-6,
        'time': 5.625,
        'resistance': 1.0/4096.0,
        'temperature': 1.0/256.0,
    }
    UNITS: Dict[str, str] = {
        'capacity': 'Ah',
        'percentage': '%',
        'voltage': 'V',
        'current': 'A',
        'time': 's',
        'resistance': 'Ohm',
        'temperature': 'C',
    }
    REGISTERS: Dict[str, RegDetails] = {
        'voltage': RegDetails('voltage', 0x09),
        'average_voltage': RegDetails('voltage', 0x19),
        'current': RegDetails('current', 0x0A),
        'average_current': RegDetails('current', 0x0B),
        'temperature': RegDetails('temperature', 0x08),
        'average_temperature': RegDetails('temperature', 0x16),
        'die_temperature': RegDetails('temperature', 0x34),
        'design_capacity': RegDetails('capacity', 0x18),
        'charge': RegDetails('capacity', 0x05),
        'percentage': RegDetails('percentage', 0x06),
        'capacity': RegDetails('capacity', 0x10),
        'time_to_empty': RegDetails('time', 0x11),
        'time_to_full': RegDetails('time', 0x20),
    }

    def __init__(self, channel: int, max17621=0x36, bq25895: int = 0x6a):
        self.channel = channel
        self.max17621 = max17621
        self.bq25895 = bq25895
        self.bus = smbus2.SMBus(channel)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.bus.close()

    def get_register_value(self, register: str) -> float:
        try:
            reg_details = self.REGISTERS.get(register)
        except KeyError:
            raise KeyError(f"Unknown register {register}")
        val = self.bus.read_word_data(self.max17621, reg_details.reg_address)
        if reg_details.unit in ('current', 'temperature'):
            val = c_int16(val).value
        val *= self.CONVERSION[reg_details.unit]
        return val

    def get_register_string(self, register: str) -> str:
        try:
            reg_details = self.REGISTERS.get(register)
        except KeyError:
            raise KeyError(f"Unknown register {register}")
        val = self.get_register_value(register)
        return f"{val:.2f}{self.UNITS[reg_details.unit]}"

    def set_register_value(self, register: str, value: float):
        try:
            reg_details = self.REGISTERS.get(register)
        except KeyError:
            raise KeyError(f"Unknown register {register}")
        val = int(value / self.CONVERSION[reg_details.unit])
        self.bus.write_word_data(self.max17621, reg_details.reg_address, val)

class BatteryNode(Node):
    set_capacity: float = 0.0
    i2c_bus: int = 0
    report_interval: float = 1.0

    def __init__(self):
        super().__init__()
        self.pub = self.create_publisher(BatteryState, "battery", 5)
        self.batt = Battery(self.i2c_bus)
        self.timer = self.create_timer(self.report_interval, self.callback)
        if self.set_capacity > 0:
            self.get_logger().info(f"Setting capacity to {self.set_capacity}Ah")
            self.batt.set_register_value("design_capacity", self.set_capacity)

    def callback(self):
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        fields = msg.get_fields_and_field_types().keys()
        for field in fields:
            if field in self.batt.REGISTERS:
                setattr(msg, field, self.batt.get_register_value(field))
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    b = BatteryNode()
    b.run()

if __name__ == "__main__":
    batt = Battery(3)
    for reg in batt.REGISTERS:
        print(f"{reg}: {batt.get_register_string(reg)}")
