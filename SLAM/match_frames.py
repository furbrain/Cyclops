import time
from collections import defaultdict
from pathlib import Path
from typing import List, Dict

import cv2
import numpy as np

from SLAM.frames import FrameSet
import keypoints_json
keypoints_json.register_keypoint_pickles()

WORKING_DIR: Path = Path("/home/phil/footage/outside3/")
SHORTCUT: bool = False

frameset = FrameSet(WORKING_DIR)
frameset.find_frames()
frameset.load_descs()
# don't need to load the images for this...

frames = list(x[1] for x in sorted(frameset.frames.items()))
matches: Dict[int,List[int]] = defaultdict(list)
fail_count = 0

start = time.time()
for i, frame_i in enumerate(frames):
    print(i)
    for j, frame_j in enumerate(frames[i+1:], i+1):
        new, _, distance = frame_i.compare(frame_j)
        if len(new) < 30 and SHORTCUT:
            fail_count += 1
            if fail_count >= 5:
                break
        elif len(new) >= 30:
            fail_count = 0
            matches[i].append(j)
            matches[j].append(i)
print(time.time()-start)
print(sum(len(x) for x in matches.values()))
