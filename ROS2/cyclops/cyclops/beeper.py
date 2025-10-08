#!/usr/bin/env python3

import os, mmap, struct
import time

RP1_BAR1 = 0x1f00000000
RP1_BAR1_LEN = 0x400000

RP1_BAR2 = 0x1f00400000
PR1_BAR2_LEN = 64 * 1024

SYSINFO_CHIP_ID_OFFSET = 0x00000000
SYSINFO_PLATFORM_OFFSET = 0x00000004

RP1_SYSINFO_BASE = 0x000000
RP1_POWER_BASE = 0x010000
RP1_RESETS_BASE = 0x014000
RP1_RAM_BASE = 0x1c0000
RP1_RAM_SIZE = 0x020000
RP1_IO_BANK0_BASE = 0x0d0000
RP1_PADS_BANK0_BASE = 0x0f0000

RP1_PWM0_BASE = 0x098000
RP1_PWM1_BASE = 0x09C000

PWM_GLOBAL_CTRL = 0x0
PWM_FIFO_CTRL = 0x04
PWM_COMMON_RANGE = 0x08
PWM_COMMON_DUTY = 0x0C
PWM_DUTY_FIFO = 0x10
PWM_CHAN0 = 0x14
PWM_CHAN1 = 0x24
PWM_CHAN2 = 0x34
PWM_CHAN3 = 0x44
# LED on GPIO17
GPIO17_CTRL = 0x8c
PAD_GPIO17 = (17 * 4) + 4

# SYS_RIO offsets
SYS_RIO_OUT_OFF = 0x00
SYS_RIO_OE_OFF = 0x04
SYS_RIO_IN_OFF = 0x08

CLOCK_FREQ = 50_000_000

PIN_A = 13
PIN_B = 19

PIN_A_PWM = PWM_CHAN1
PIN_A_FUNC = 0
PIN_B_PWM = PWM_CHAN3
PIN_B_FUNC = 3

PIN_A_CTRL = (PIN_A*8) + 4
PIN_A_PAD = (PIN_A+1) * 4

PIN_B_CTRL = (PIN_B*8) + 4
PIN_B_PAD = (PIN_B+1) * 4


def main():
    fd_periph = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
    mem_periph = mmap.mmap(fd_periph, RP1_BAR1_LEN, offset=RP1_BAR1)

    fd_sram = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
    mem_sram = mmap.mmap(fd_sram, PR1_BAR2_LEN, offset=RP1_BAR2)

    def reg_read(dev, reg, l=4):
        mem_periph.seek(dev + reg)
        if l == 4:
            return struct.unpack("<I", mem_periph.read(4))[0]
        else:
            return mem_periph.read(l)

    def reg_write(dev, reg, val):
        mem_periph.seek(dev + reg)
        mem_periph.write(struct.pack("<I", val))

    print("Chip ID:", hex(reg_read(RP1_SYSINFO_BASE, SYSINFO_CHIP_ID_OFFSET)))
    print("Platform", hex(reg_read(RP1_SYSINFO_BASE, SYSINFO_PLATFORM_OFFSET)))

    def setup():
        # setup gpios
        reg_write(RP1_PADS_BANK0_BASE, PIN_A_PAD, 0x10) # enable output, 4mA, slow slew
        reg_write(RP1_IO_BANK0_BASE, PIN_A_CTRL, 0x0000_C080 + PIN_A_FUNC)
        reg_write(RP1_PADS_BANK0_BASE, PIN_B_PAD, 0x10) # enable output, 4mA, slow slew
        reg_write(RP1_IO_BANK0_BASE, PIN_B_CTRL, 0x0000_C080 + PIN_B_FUNC)

    def beep(freq, duty=0.5):
        stop()
        RANGE = int(CLOCK_FREQ / freq)
        DUTY =  int(duty * RANGE)
        reg_write(RP1_PWM0_BASE, PWM_COMMON_RANGE, RANGE)
        reg_write(RP1_PWM0_BASE, PWM_COMMON_DUTY, DUTY)
        reg_write(RP1_PWM0_BASE, PIN_A_PWM, 0x11) # use common range, trailing edge
        reg_write(RP1_PWM0_BASE, PIN_B_PWM, 0x19) # use common range, trailing edge, inverted
        reg_write(RP1_PWM0_BASE, PWM_GLOBAL_CTRL, 0x8000_000A) # enable two pwms

    def stop():
        reg_write(RP1_PWM0_BASE, PWM_GLOBAL_CTRL, 0x8000_0000) # disable all channels

    setup()
    beep(256, 0.5)
    time.sleep(1)
    stop()
    time.sleep(0.5)

if __name__ == "__main__":
    main()