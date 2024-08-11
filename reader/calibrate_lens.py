from typing import Sequence

import numpy as np
import cv2

from cyclopsreader import CyclopsReader

reader = CyclopsReader("../footage/vid0.mkv")
reader.calibrate_lens()
reader.lens.save("lens_cal.npz")