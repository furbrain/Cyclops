#!/usr/bin/env python3
import json
import mag_cal
import board
import icm
import time

i2c = board.I2C()
icm = icm.ICM20948(i2c)
with open("calibration.json") as f:
    cal = json.load(f)
s = mag_cal.Sensor.from_dict(cal)
while True:
    mag = icm.magnetic
    print(mag)
    corrected = s.apply(mag)
    print(corrected)
    time.sleep(0.5)
    