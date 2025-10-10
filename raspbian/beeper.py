#!/usr/bin/env python3

import os, mmap, struct
from typing import Optional, Tuple, Dict, Sequence


class Beeper:
    RP1_BAR1 = 0x1f00000000
    RP1_BAR1_LEN = 0x400000

    RP1_SYSINFO_BASE = 0x000000
    SYSINFO_CHIP_ID_OFFSET = 0x00000000
    SYSINFO_PLATFORM_OFFSET = 0x00000004

    RP1_IO_BANK0_BASE = 0x0d0000
    RP1_PADS_BANK0_BASE = 0x0f0000

    RP1_PWM0_BASE = 0x098000
    PWM_GLOBAL_CTRL = 0x0
    PWM_FIFO_CTRL = 0x04
    PWM_COMMON_RANGE = 0x08
    PWM_COMMON_DUTY = 0x0C

    CLOCK_FREQ = 50_000_000
    PINS: Dict[int, Tuple[int,int]] = {
        12: (0,0),
        13: (0,1),
        14: (0,2),
        15: (0,3),
        18: (3,2),
        19: (3,3)
    }

    def __init__(self, pin_a: int, pin_b: Optional[int] = None):
        self.fd_periph = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
        self.mem_periph = mmap.mmap(self.fd_periph, self.RP1_BAR1_LEN, offset=self.RP1_BAR1)
        if pin_a not in self.PINS:
            raise ValueError(f"Pin A is {pin_a}, must be one of {list(self.PINS.keys())}")
        if pin_b is not None and pin_b not in self.PINS:
            raise ValueError(f"Pin B is {pin_b}, must be one of {list(self.PINS.keys())}")
        if pin_a == pin_b:
            raise ValueError("Pin A and Pin B should be different")
        self.pin_a = pin_a
        self.pin_b = pin_b

    def setup(self):
        self.enable_pin(self.pin_a)
        if self.pin_b is not None:
            self.enable_pin(self.pin_b)

    def reg_read(self, dev, reg, l=4):
        self.mem_periph.seek(dev + reg)
        if l == 4:
            return struct.unpack("<I", self.mem_periph.read(4))[0]
        else:
            return self.mem_periph.read(l)

    def reg_write(self, dev, reg, val):
        self.mem_periph.seek(dev + reg)
        self.mem_periph.write(struct.pack("<I", val))

    def identify(self):
        print("Chip ID:", hex(self.reg_read(self.RP1_SYSINFO_BASE, self.SYSINFO_CHIP_ID_OFFSET)))
        print("Platform", hex(self.reg_read(self.RP1_SYSINFO_BASE, self.SYSINFO_PLATFORM_OFFSET)))

    @staticmethod
    def get_pin_pad_ctrl(pin: int) -> Tuple[int,int]:
        """
        Get the GPIO and PAD offsets for a pin
        :param pin: The pin
        :return: GPIO offset, PAD offset
        """
        return pin*8+4, (pin+1)*4

    def get_pwm_ctrl(self, pin: int):
        channel = self.PINS[pin][1]
        return channel*0x10 +0x14

    def enable_pin(self, pin: int):
        ctrl, pad = self.get_pin_pad_ctrl(pin)
        self.reg_write(self.RP1_PADS_BANK0_BASE, pad, 0x10)  # enable output, 4mA, slow slew
        self.reg_write(self.RP1_IO_BANK0_BASE, ctrl, 0x0000_C080 | self.PINS[pin][0])

    def disable_pin(self, pin: int):
        ctrl, pad = self.get_pin_pad_ctrl(pin)
        self.reg_write(self.RP1_PADS_BANK0_BASE, pad, 0x92)  # disable output, 4mA, slow slew
        self.reg_write(self.RP1_IO_BANK0_BASE, ctrl, 0x0000_009f)


    def beep(self, freq: float, duty=0.5):
        self.stop()
        range = int(self.CLOCK_FREQ / freq)
        duty =  int(duty * range)
        self.reg_write(self.RP1_PWM0_BASE, self.PWM_COMMON_RANGE, range)
        self.reg_write(self.RP1_PWM0_BASE, self.PWM_COMMON_DUTY, duty)
        self.reg_write(self.RP1_PWM0_BASE, self.get_pwm_ctrl(self.pin_a), 0x11) # use common range, trailing edge
        pwm_en = 1 << self.PINS[self.pin_a][1]
        if self.pin_b is not None:
            self.reg_write(self.RP1_PWM0_BASE, self.get_pwm_ctrl(self.pin_b), 0x19) # use common range, trailing edge,
            # invert
            pwm_en |= 1 << self.PINS[self.pin_b][1]
        self.reg_write(self.RP1_PWM0_BASE, self.PWM_GLOBAL_CTRL, 0x8000_0000 | pwm_en) # enable two pwms


    def stop(self):
        self.reg_write(self.RP1_PWM0_BASE, self.PWM_GLOBAL_CTRL, 0x8000_0000) # disable all channels

    def finish(self):
        self.stop()
        self.disable_pin(self.pin_a)
        if self.pin_b is not None:
            self.disable_pin(self.pin_b)

