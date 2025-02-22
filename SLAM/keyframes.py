import os.path
from dataclasses import dataclass, field
from pathlib import Path
from typing import List

import cv2
import numpy as np
import mem_top

from . import keypoints_json
from .frames import Frame, FrameSet, MAX_SIZE, SHARP_WINDOW
from reader.gstreader import VidReader

keypoints_json.register_keypoint_pickles()

WORKING_DIR: Path = Path("/home/phil/footage/sofa/")

PERCENTAGE_DISTANCE = 5
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


def get_keyframes_from_source(cap, save_frames=True, working_dir=WORKING_DIR):
    frames: List[ComparisonFrame] = []
    metrics = []
    keyframes = FrameSet(working_dir)
    first_f = None
    count = 0
    mask = cv2.imread(str(working_dir / "masks" / "mask.png"), cv2.IMREAD_GRAYSCALE)
    for tm, image in cap:
        fr = ComparisonFrame(working_dir, image, ts=count, orig_mask=mask)
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
                if all(x == 0 for x in metrics[-30:]):
                    new_frame, valid = get_keyframe(frames, metrics)
                    if valid:
                        keyframes.add_frame(new_frame)
                        if len(keyframes.frames) > 10 and save_frames:
                            keyframes.save_images()
                            keyframes.save_descs()
                            keyframes.link_masks()
                            del keyframes
                            print("clearing keyframes")
                            keyframes = FrameSet(working_dir)
                    else:
                        print("invalid", len(frames))

        count += 1
    print(f"finished, previous first frame is {frames[0].ts}")
    while True:
        new_frame, valid = get_keyframe(frames, metrics) # find last keyframe
        if valid:
            print("adding final keyframes")
            keyframes.add_frame(new_frame)
        if len(metrics) < 15:
            break
    if save_frames:
        keyframes.save_images()
        keyframes.save_descs()
        keyframes.link_masks()
    return keyframes

if __name__ == "__main__":
    cap = VidReader.from_filename(WORKING_DIR / "vid.mkv")
    keyframes = get_keyframes_from_source(cap, True, WORKING_DIR)
