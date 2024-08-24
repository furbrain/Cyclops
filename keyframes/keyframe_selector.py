from typing import Optional, Iterator

import cv2
import numpy as np

FNAME: str = "/home/phil/footage/outside.mp4"
FLOW_WIDTH: int = 720
SHARP_WIDTH: int = 720
SHARPNESS_WINDOW_SIZE: int = 7

def CapReader(cap: cv2.VideoCapture) -> Iterator[np.ndarray]:
    while True:
        ret, image = cap.read()
        if not ret:
            break
        yield image


def compute_sharpness(img:np.ndarray, window_size=SHARPNESS_WINDOW_SIZE) -> float:
    laplacian = cv2.Laplacian(img, cv2.CV_64F)
    sum_arr, squared_sum = cv2.integral2(laplacian)
    

width: Optional[int] = None
height: Optional[int] = None
cap = cv2.VideoCapture(FNAME)
sharpness_scores = []
flow_scores = []
prev_flow: Optional[np.ndarray] = None
new_flow: Optional[np.ndarray] = None
flow = cv2.optflow.createOptFlow_DeepFlow()
for image in CapReader(cap):
    if width is None:
        width = image.shape[0]
        height = image.shape[1]
        flow_size = (width,int(FLOW_WIDTH*height/width))
        sharp_size = (width,int(FLOW_WIDTH*height/width))
        print(f"{width}x{height}")
    else:
        prev_flow = new_flow
    new_flow = cv2.resize(image, flow_size)
    sharp_frame = cv2.resize(image, flow_size)
