from typing import Sequence

import numpy as np
import cv2

from cyclopsreader import CyclopsReader

reader = CyclopsReader("../footage/vid1.mkv", lens_cal_file="lens_cal.npz")
reader.calibrate_imu()
