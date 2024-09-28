import os.path
from dataclasses import dataclass, field
from pathlib import Path
from typing import List

import cv2
import numpy as np
import mem_top

import keypoints_json
from frames import Frame, FrameSet, MAX_SIZE, SHARP_WINDOW
from reader.gstreader import VidReader

keypoints_json.register_keypoint_pickles()

WORKING_DIR: Path = Path("/home/phil/footage/HE2a/")

PERCENTAGE_DISTANCE = 10
PX_DISTANCE = PERCENTAGE_DISTANCE * MAX_SIZE / 100
print("importing")
#from pypopsift import popsift



@dataclass
class ComparisonFrame(Frame):
    distance: float = field(init=False)
    sharpness: float = field(init=False)
    match_count: int = field(init=False)
    metric: float = field(init=False)

    def __post_init__(self):
        super().__post_init__()
        self.distance = 0
        self.sharpness = 0
        self.match_count = 0
        self.metric = 0

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
                if self.mask is not None and not np.all(self.mask[y:y+SHARP_WINDOW, x:x+SHARP_WINDOW]>0):
                    stds.append(0)
                    continue
                s1 = self.get_sum(sum, x, y, SHARP_WINDOW, SHARP_WINDOW)
                s2 = self.get_sum(sq_sum, x, y, SHARP_WINDOW, SHARP_WINDOW)
                stds.append(np.sqrt((s2 - (s1*s1)/wind_count)/wind_count))
        self.sharpness = max(stds)

    def compare_and_store(self, other: Frame):
        new, _, distance = self.compare(other)
        self.match_count = len(new)
        self.distance = distance
        self.metric = self.keyframe_metric()

    def distance_metric(self) -> float:
        if PX_DISTANCE * 0.5 < self.distance < PX_DISTANCE * 1.5:
            return 1.0
        if self.distance < PX_DISTANCE * 0.5:
            return self.distance / (PX_DISTANCE * 0.5)
        if self.distance > PX_DISTANCE * 1.5:
            return max(1 - ((self.distance/PX_DISTANCE) - 1.5),0) + 0.01

    def connectivity_metric(self) -> float:
        if self.match_count > 50: return 1.0
        if self.match_count < 25: return 0.0
        return (self.match_count - 25) / 25

    def keyframe_metric(self):
        return self.sharpness * self.distance_metric() * self.connectivity_metric()

    def bad_frame(self):
        return (self.distance > PX_DISTANCE * 1.5) and (self.connectivity_metric() == 0)


def get_keyframe(frames: List[ComparisonFrame], metrics: List[int]) -> Frame:
    valid = True
    index = metrics.index(max(metrics[1:]))
    if index==0:
        index = 5
        valid = False
    else:
        print(f"choosing {frames[0].ts}")
    frames[:] = frames[index:]
    for fr in frames:
        #update metrics
        fr.compare_and_store(frames[0])
    metrics[:] = [x.keyframe_metric() for x in frames]
    return frames[0], valid

flags = cv2.IMREAD_COLOR
cap = VidReader.from_filename(WORKING_DIR / "vid.mkv")
# if os.path.exists(str(WORKING_DIR / "vid.mkv")):
#     cap = cv2.VideoCapture(str(WORKING_DIR / "vid.mkv"))
# elif os.path.exists(str(WORKING_DIR / "vid.mp4")):
#     cap = cv2.VideoCapture(str(WORKING_DIR / "vid.mp4"))
# else:
#     print("Video file not found")
#     exit()

frames: List[ComparisonFrame] = []
metrics = []
keyframes = FrameSet(WORKING_DIR)
first_f = None
count = 0
target_bad_frame = -1
mask = cv2.imread(str(WORKING_DIR / "masks" / "mask.png"), cv2.IMREAD_GRAYSCALE)
while True:
    tm, image = cap.get_frame()
    if image is None:
        new_frame, valid = get_keyframe(frames, metrics)
        if valid:
            keyframes.add_frame(new_frame)
        break
    fr = ComparisonFrame(WORKING_DIR, image, ts=count, orig_mask=mask)
    fr.get_sharpness()
    fr.compute_kps()
    if first_f is None:
        first_f = fr
        frames.append(first_f)
        metrics.append(0)
    else:
        fr.compare_and_store(frames[0])
        frames.append(fr)
        metrics.append(fr.metric)
        if len(metrics) > 30:
            if all(x==0 for x in metrics[-30:]):
                new_frame, valid = get_keyframe(frames, metrics)
                if valid:
                    keyframes.add_frame(new_frame)
                    if len(keyframes.frames) > 10:
                        keyframes.save_images()
                        keyframes.save_descs()
                        keyframes.link_masks()
                        del keyframes
                        print("clearing keyframes")
                        keyframes = FrameSet(WORKING_DIR)
                else:
                    print("invalid", len(frames))

    count+=1
keyframes.save_images()
keyframes.save_descs()
keyframes.link_masks()
