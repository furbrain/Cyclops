from typing import Sequence

import numpy as np
import cv2

from cyclopsreader import CyclopsReader

reader = CyclopsReader("/home/phil/cal.mkv")
reader.calibrate_lens(show_results=True)
reader.lens.save("lens_cal.npz")