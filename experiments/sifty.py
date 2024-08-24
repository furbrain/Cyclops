import time
from dataclasses import dataclass, field
from typing import Optional, List, ClassVar

import cv2
import numpy as np

SHARP_WINDOW = 200

MAX_SIZE = 720
PX_DISTANCE = 15
print("importing")
#from pypopsift import popsift



@dataclass
class Frame:
    orig_img: np.ndarray
    mask: Optional[np.ndarray] = None
    kp: Optional[List[cv2.KeyPoint]] = None
    desc: Optional[np.ndarray] = None
    ts: int = 0
    distance: float = field(init=False)
    sharpness: float = field(init=False)
    width: int = field(init=False)
    height: int = field(init=False)
    img: np.ndarray = field(init=False)

    orb: ClassVar[cv2.ORB] = cv2.ORB.create()
    matcher: ClassVar[cv2.BFMatcher] = cv2.BFMatcher.create(cv2.NORM_HAMMING)

    def __post_init__(self):
        self.prepare_image()
        if self.kp is None:
            self.kp, self.desc = self.orb.detectAndCompute(self.img, self.mask)
        self.distance = 0
        self.get_sharpness()

    def prepare_image(self):
        h,w = self.orig_img.shape[:2]
        size = max(h,w)
        self.width = (MAX_SIZE * w) // size
        self.height = (MAX_SIZE * h) // size
        self.img = cv2.resize(self.orig_img, (self.width, self.height))
        if len(self.orig_img.shape)>=3 and self.orig_img.shape[-1] != 1:
            self.img = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)

    @staticmethod
    def get_sum(img, x, y, width, height):
        tl = img[y,x]
        tr = img[y + height,x]
        bl = img[y, x + width]
        br = img[y + height, x + width]
        return br + tl - tr - bl

    def get_sharpness(self):
        laplacian = cv2.Laplacian(self.img, cv2.CV_64F)
        sum, sq_sum = cv2.integral2(laplacian)
        wind_count = SHARP_WINDOW*SHARP_WINDOW
        stds = []
        for x in range(1, self.width-SHARP_WINDOW, SHARP_WINDOW // 4):
            for y in range(1, self.height-SHARP_WINDOW, SHARP_WINDOW // 4):
                s1 = self.get_sum(sum, x, y, SHARP_WINDOW, SHARP_WINDOW)
                s2 = self.get_sum(sq_sum, x, y, SHARP_WINDOW, SHARP_WINDOW)
                stds.append(np.sqrt((s2 - (s1*s1)/wind_count)/wind_count))
        self.sharpness = max(stds)

    def compare(self, other:"Frame"):
        matches = self.matcher.knnMatch(self.desc, other.desc, k=2)
        good_matches: List[cv2.DMatch] = [m for m,n in matches if m.distance < 0.75 * n.distance]
        new_idxs = [x.queryIdx for x in good_matches]
        old_idxs = [x.trainIdx for x in good_matches]
        new_points = cv2.KeyPoint.convert(self.kp, new_idxs)
        old_points = cv2.KeyPoint.convert(other.kp, old_idxs)
        self.distance = np.mean(np.linalg.norm(new_points-old_points, axis=1 ))


filename = "/home/phil/footage/outside.mp4"

flags = cv2.IMREAD_COLOR
cap = cv2.VideoCapture(filename)


def process_frame(f: Frame):
    print(f"saving frame {f.ts}")
    cv2.imwrite(f"/home/phil/footage/keys/{f.ts:05}.jpg", f.orig_img)

def find_keyframe(frames: List[Frame]) -> Frame:
    l = len(frames)
    half_l = l//2
    weights = np.ones((l,))
    added = np.linspace(0,1,half_l)
    weights[:half_l] += added
    weights[-half_l:] += added[::-1]

    sharps = [(f.sharpness + w, f) for f, w in zip(frames, weights)]
    return max(sharps)[1]

print("got capture")
frames = []
first_f = None
count = 0
while True:
    ret, image = cap.read()
    if image is None:
        process_frame(find_keyframe(frames))
        break
    if first_f is None:
        first_f = Frame(image, ts=count)
        frames.append(first_f)
    else:
        fr = Frame(image, ts=count)
        fr.compare(first_f)
        if fr.distance > PX_DISTANCE*MAX_SIZE/100:
            process_frame(find_keyframe(frames))
            first_f = fr
            fr.distance = 0
            frames = [fr]
        else:
            frames.append(fr)
    count+=1