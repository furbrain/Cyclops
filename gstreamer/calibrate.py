#!/usr/bin/env python3

import time
import board
import numpy as np
import mag_cal
import icm
import json

i2c = board.I2C()
icm_dev = icm.ICM20948(i2c)
print("Starting calibration in 5 seconds")
time.sleep(5)
print("Go! You have 10 seconds")
data  = np.zeros((300,3))
for i in range(300):
    data[i] = icm_dev.magnetic
    time.sleep(0.03)
print(data)

s = mag_cal.Sensor()
s.fit_ellipsoid(data)
print(s.as_dict())
with open("calibration.json","w") as f:
    json.dump(s.as_dict(),f)