from pathlib import Path
from typing import Optional
from skimage.metrics import structural_similarity
import os

import cv2
import numpy as np

WORKING_DIR: Path = Path("/home/phil/footage/bpotw/")
DECIMATION = 10
cap = cv2.VideoCapture(str(WORKING_DIR / "vid.mkv"))

index = 0
oldGray: Optional[np.ndarray] = None
diff_sum: Optional[np.ndarray] = None
sum_image: Optional[np.ndarray] = None
sum_sq_image: Optional[np.ndarray] = None
count = 0
while True:
    ret, image = cap.read()
    if image is None:
        break
    index += 1
    if (index % DECIMATION)==0:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if oldGray is not None:
            score, diff = structural_similarity(oldGray, gray, full=True)
            if diff_sum is not None:
                diff_sum += diff
            else:
                diff_sum = diff
        oldGray = gray
        print(index)
diff_sum = cv2.blur(diff_sum, (5,5))
diff_sum /= np.max(diff_sum)
diff_u8 = (diff_sum * 255).astype("uint8")
val, thresh = cv2.threshold(diff_u8,127, 255.0, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
# clean it
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(7,7))
thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=4)
thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=4)

#save it
MASK_DIR = WORKING_DIR / "masks"
os.makedirs(MASK_DIR,exist_ok=True)
cv2.imwrite(str(MASK_DIR/"mask.png"), thresh)
cv2.imwrite(str(MASK_DIR/"gray.png"), gray)

