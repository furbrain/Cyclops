#!/usr/bin/env python3
from dataclasses import dataclass, field
from typing import Dict, Tuple, List
from ctypes import c_int16
import smbus2
import rclpy
from autonode import Node, subscription
from sensor_msgs.msg import BatteryState
import time
from orb_slam3.msg import State



@dataclass
class RegDetails:
    unit: str
    reg_address: int

CHANNEL = 3

@dataclass
class Pattern:
    INFINITE_REPEATS = 0xF
    index: int
    bus: smbus2.SMBus
    address: int = 0x2D
    pause_time: Tuple[float,float] = (0,0)
    sloper_time: Tuple[float, float, float, float] = (0,0,0,0)
    pwm_values: Tuple[float, float, float, float, float] = (0,0,0,0,0)
    repeat_count: int = INFINITE_REPEATS

    TIMES = [0.0, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 1.0, 2.0, 4.0, 6.0, 8.0]
    PAUSE_TIME_REG = 0x1C
    REPEAT_COUNT_REG = 0x1D
    PWM_REG0 = 0x1E
    SLOPER_TIME_REG0 = 0x23

    @classmethod
    def get_nibble_from_time(cls, tm: float):
        try:
            return cls.TIMES.index(tm)
        except ValueError:
            raise ValueError(f"Invalid time: {tm}. Valid times are: {cls.TIMES}")

    def write_register(self, reg: int, value: int):
        reg += self.index*9
        print(f"reg: 0x{reg:x}, data: 0x{value:x}, index:{self.index}")
        self.bus.write_byte_data(self.address, reg, value)

    def write_block(self, reg: int, data: List[int]):
        reg += self.index*9
        print(f"reg: 0x{reg:x}, data: {data}, index: {self.index}")
        self.bus.write_i2c_block_data(self.address, reg, data)

    def write_pause_times(self):
        value = self.get_nibble_from_time(self.pause_time[1]) << 4 | self.get_nibble_from_time(self.pause_time[0])
        self.write_register(self.PAUSE_TIME_REG, value)

    def write_repeat_count(self):
        self.write_register(self.REPEAT_COUNT_REG, self.repeat_count)

    def write_sloper_times(self):
        values = [self.get_nibble_from_time(x) for x in self.sloper_time]
        result = [values[0] | values[1] <<4, values[2] | values[3] <<4]
        self.write_block(self.SLOPER_TIME_REG0, result)

    def write_pwm_values(self):
        values = [int(x*255) for x in self.pwm_values]
        self.write_block(self.PWM_REG0, values)

    def write_all_data(self):
        self.write_pause_times()
        self.write_repeat_count()
        self.write_sloper_times()
        self.write_pwm_values()

@dataclass
class Engine:
    INFINITE_REPEATS = 0x03
    index: int
    bus: smbus2.SMBus
    address: int = 0x2d
    patterns: List[int] = field(default_factory=list)
    repeats: int = 0x1

    CONFIG_REG = 0x06
    ENABLE_REG = 0x0A
    REPEAT_REG = 0x0C

    def patch_bitfield(self, reg: int, val: int, offset: int, width: int):
        mask = 0
        for i in range(width):
            mask |= 1 << i+offset
        val = val << offset
        mask = ~mask # invert the mask
        if mask & val:
            # this should be zero...
            raise ValueError(f"{val} does not fit in {width} bits")
        current = self.bus.read_byte_data(self.address, reg)
        current = (current & mask) | val
        self.bus.write_byte_data(self.address, reg, current)

    def write_data(self):
        if len(self.patterns) > 4:
            raise ValueError(f"Too many patterns in engine {self.index}")
        if max(self.patterns) > 4:
            raise ValueError("Max pattern number is 3")
        if min(self.patterns) < 0:
            raise ValueError("Min pattern number is 0")
        config_val = 0
        enable_val = 0
        for i, pattern in enumerate(self.patterns):
            config_val |= pattern << i*2
            enable_val |= 1 << i
        self.bus.write_byte_data(self.address, self.CONFIG_REG+self.index, config_val)
        enable_offset = (self.index % 2) * 4 # 4 if odd, 0 if even
        reg = self.ENABLE_REG
        if self.index >=2:
            reg +=1
        self.patch_bitfield(reg, enable_val, enable_offset, 4)
        self.patch_bitfield(self.REPEAT_REG, self.repeats, self.index*2, 2)


class LP5815:
    ENABLE_REG = 0x00
    MAX_CURRENT_REG = 0x01
    LED_ENABLE_REG = 0x02
    AUTO_ENABLE_REG = 0x04
    AUTO_CHANNEL_REG = 0x05
    ANALOG_CURRENT_REG0 = 0x14
    PWM_CURRENT_REG0 = 0x18
    RESET_REG = 0x0E
    START_REG=0x10
    STOP_REG=0x11


    def __init__(self, i2c_bus_num: int, address: int = 0x2d, red_I = 20, green_I=20, blue_I=20):
        self.bus = smbus2.SMBus(i2c_bus_num)
        self.address = address
        self.currents = (red_I, green_I, blue_I)
        self.high_current = max(self.currents) > 25
        self.setup_device()

    def get_dot_current(self, current: float) -> int:
        if self.high_current:
            return int(255*current/51)
        else:
            return int(255*current/25.5)

    def setup_device(self):
        #enable device
        self.bus.write_byte_data(self.address, self.RESET_REG, 0XCC) #reset device
        time.sleep(0.05)
        self.bus.write_byte_data(self.address, self.ENABLE_REG, 0x03) #disable blinking, enable chip
        #set maximum current
        if self.high_current:
            self.bus.write_byte_data(self.address, self.MAX_CURRENT_REG, 0x01) #set max current
        else:
            self.bus.write_byte_data(self.address, self.MAX_CURRENT_REG, 0x00) #set 25mA current
        self.bus.write_byte_data(self.address, self.LED_ENABLE_REG, 0x07) # enable all leds
        for i, current in enumerate(self.currents):
            self.bus.write_byte_data(self.address, self.ANALOG_CURRENT_REG0 + i, self.get_dot_current(current))
        #set individual LED max currents

    def update(self):
        self.bus.write_byte_data(self.address, 0x0F, 0x55)

    def start(self):
        self.bus.write_byte_data(self.address, self.START_REG, 0xff)

    def stop(self):
        self.bus.write_byte_data(self.address, self.STOP_REG, 0xaa)

    def set_colours(self, red: float=0.0, green=0.0, blue=0.0):
        """
        Set LED intensities
        :param red: Set Red LED to this intensity (off if omitted)
        :param green: Set Green LED to this intensity (if ommitted)
        :param blue: Set Blue LED to this intensity (if ommitted)
        :return:
        """
        for i, value in enumerate((red, green, blue)):
            self.bus.write_byte_data(self.address, self.PWM_CURRENT_REG0 + i, int(255*value))
        self.update()

    def set_led_engines(self, red = -1, green = -1, blue = -1):
        self.stop()
        rgb = (red, green, blue)
        enable_val = 0
        for i, val in enumerate(rgb):
            if val >= 0:
                enable_val |= 1 << i
        rgb = [max(0,x) for x in rgb]
        channel_val = 0
        for i, val in enumerate(rgb):
            channel_val |= val << i*2
        self.bus.write_byte_data(self.address, self.AUTO_ENABLE_REG, enable_val)
        self.bus.write_byte_data(self.address, self.AUTO_CHANNEL_REG, channel_val)
        self.update()
        self.start()


        #first turn on or off each automatic thang




    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.bus.close()


class OrbStateLed(Node):
    BREATHE_INDEX = 0
    FLASH_INDEX = 1
    i2c_bus: int = 1
    def __init__(self):
        super().__init__()
        self.led = LP5815(self.i2c_bus, green_I=15)
        breathe = Pattern(self.BREATHE_INDEX, self.led.bus, self.led.address,
                          pause_time=(0.0, 0.0),
                          sloper_time=(0.5, 0.5, 0.5, 0.5),
                          pwm_values=(0, 0.4, 0.6, 0.4, 0),
                          repeat_count=1)

        flash = Pattern(self.FLASH_INDEX, self.led.bus, self.led.address,
                        pause_time=(0.0, 0.0),
                        sloper_time=(0, 0.25, 0, 0.25),
                        pwm_values=(0, 0.5, 0.5, 0, 0),
                        repeat_count=3)
        self.led.stop()
        breathe.write_all_data()
        flash.write_all_data()
        breathe_engine = Engine(self.BREATHE_INDEX, self.led.bus, self.led.address, patterns=[breathe.index])
        flash_engine = Engine(self.FLASH_INDEX, self.led.bus, self.led.address, patterns=[flash.index])
        breathe_engine.write_data()
        flash_engine.write_data()
        self.led.update()

        self.last_noise_tm = 0
        self.last_state = 0
        self.lock_out = 0
        self.create_timer(1.0, self.timer_callback)

    def elapsed(self, seconds: float):
        now = time.monotonic_ns()
        if self.last_noise_tm + int(seconds*1e9) < now:
            self.last_noise_tm = now
            return True
        return False

    @subscription(State)
    async def state(self, msg: State):
        if self.lock_out < time.monotonic_ns():
            if msg.state==msg.OK:
                if self.elapsed(1.0):
                    self.led.set_led_engines(green=self.BREATHE_INDEX)
            elif msg.state==msg.RECENTLY_LOST:
                if self.elapsed(0.333):
                    self.led.set_led_engines(red=self.FLASH_INDEX, green=self.FLASH_INDEX)
            elif msg.state==msg.LOST or msg.state==msg.OTHER:
                self.last_noise_tm = time.monotonic_ns()
                self.led.set_led_engines(red=self.FLASH_INDEX)
            self.last_state = msg.state

    def timer_callback(self):
        if self.elapsed(3.0): #we haven't done anything for a bit, breathe blue
            self.led.set_led_engines(blue=self.BREATHE_INDEX)



def main(args=None):
    rclpy.init(args=args)
    node = OrbStateLed()
    node.run()

if __name__ == "__main__":
    led = LP5815(1)
    # led.set_colours(red=1.0)
    # time.sleep(1)
    # led.set_colours(green=1.0)
    # time.sleep(1)
    # led.set_colours(blue=1.0)
    # time.sleep(1)
    # led.set_colours() # off
    # time.sleep(1)
    breathe = Pattern(0, led.bus, led.address,
                      pause_time = (0.0,0.0),
                      sloper_time= (0.5,0.5,0.5,0.5),
                      pwm_values= (0,0.4,0.6,0.4,0),
                      repeat_count=1)

    flash = Pattern(1, led.bus, led.address,
                    pause_time = (0.0,0.0),
                    sloper_time= (0,0.25,0,0.25),
                    pwm_values= (0,0.5,0.5,0,0),
                    repeat_count=3)
    led.stop()
    breathe.write_all_data()
    flash.write_all_data()
    breathe_engine = Engine(0, led.bus, led.address, patterns=[breathe.index])
    flash_engine = Engine(1, led.bus, led.address, patterns=[flash.index])
    breathe_engine.write_data()
    fancy_engine.write_data()
    led.update()
    led.set_led_engines(blue=breathe_engine.index)
    time.sleep(5)
    led.set_led_engines(red=fancy_engine.index, green=fancy_engine.index, blue=fancy_engine.index)
    time.sleep(5)