from pathlib import Path

import cv2
import numpy as np

WORKING_DIR: Path = Path("/home/phil/footage/bpotw/")

cap = cv2.VideoCapture(str(WORKING_DIR / "vid.mkv"))

clahe = cv2.createCLAHE()

def image_proc(img: np.ndarray) -> np.ndarray:
    img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    img = cv2.medianBlur(img,5)
    img[img<8] = 0 # set to black very low value pixels
    #img = prepare_img(img)
    #hist = cv2.calcHist([img],[0], None,[256],[0,256])
    #print(hist)

    img = cv2.equalizeHist(img)
    img = cv2.medianBlur(img, 3)
    return img

def image_proc2(img: np.ndarray) -> np.ndarray:
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    v = hsv[:,:,2]
    v = cv2.medianBlur(v,3)
    v[v<15] = 0 # set to black very low value pixels
    v = cv2.equalizeHist(v)
    v = cv2.medianBlur(v,5)
    hsv[:,:,2] = v
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

DEFAULT_SIZE = (640,480)

def prepare_img(img: np.ndarray):
    img = cv2.resize(img, DEFAULT_SIZE)
    if len(img.shape)>2 and img.shape[-1]>1:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return img

while True:
    ret, image = cap.read()
    if image is None:
        break
    pre = prepare_img(image)
    post = image_proc2(image)
    #composite = np.hstack((pre, post))
    cv2.imshow("main", post)
    cv2.waitKey(0)
