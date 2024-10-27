import pickle
from dataclasses import dataclass
from typing import Optional

import cv2

import numpy as np

from reader.LensCalibration import Lens, CHECKERBOARD, get_objpoints, DualCams
from reader.gstreader import VidReader, TOFReader

USE_SAVED = True

APPSINK = "appsink max-buffers=1 drop=true sync=true"
TOF = f"rtpvrawdepay ! videoconvert ! video/x-raw,format=GRAY8 ! {APPSINK}"
TOF_CAPS = "application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)RAW, sampling=(string)RGB, depth=(string)8, width=(string)480, height=(string)180"
CAM = f"rtph264depay ! avdec_h264 ! videoconvert ! video/x-raw,format=GRAY8 ! {APPSINK}"
CAM_CAPS = "application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96"
DEMUXER = "udpsrc"
SPEC_V = f"""{DEMUXER} caps={CAM_CAPS} port=8001 ! {CAM}"""
SPEC_T = f"""{DEMUXER} caps="{TOF_CAPS}" port=8000 ! {TOF}"""


@dataclass
class Snapshot:
    v_img: np.ndarray
    t_img: np.ndarray
    v_corners: Optional[np.ndarray] = None
    t_corners: Optional[np.ndarray] = None

    def find_corners(self):
        self.v_corners = Lens.find_cb_corners(self.v_img)
        self.t_corners = Lens.find_cb_corners(self.t_img)

    def both_corners(self) -> bool:
        return self.t_corners is not None and self.v_corners is not None

    def any_corners(self) -> bool:
        return self.t_corners is not None or self.v_corners is not None

class SnapshotList:
    def __init__(self):
        self.shots: list[Snapshot] = []

    def add_shot(self, shot: Snapshot):
        self.shots.append(shot)

    def draw_corners(self, shot: Snapshot):
        col_t = cv2.cvtColor(shot.t_img,cv2.COLOR_GRAY2BGR)
        col_v = cv2.cvtColor(shot.v_img,cv2.COLOR_GRAY2BGR)
        for corners in self.get_t_corners():
            col_t = cv2.drawChessboardCorners(col_t,CHECKERBOARD,corners, True)
        for corners in self.get_v_corners():
            col_v = cv2.drawChessboardCorners(col_v,CHECKERBOARD,corners, True)
        return col_t, col_v

    def get_t_corners(self):
        return [x.t_corners for x in self.shots]

    def get_v_corners(self):
        return [x.v_corners for x in self.shots]

    def get_corner_pairs(self):
        t_corners = [x.t_corners for x in self.shots if x.both_corners()]
        v_corners = [x.v_corners for x in self.shots if x.both_corners()]
        return t_corners, v_corners

if not USE_SAVED:
    cap_v = VidReader(SPEC_V)
    cap_t = TOFReader(SPEC_T, absolute=True)
    snapshots = SnapshotList()
    with cap_v, cap_t:
        while True:
            _, frame_v = cap_v.get_frame()
            _, frame_t = cap_t.get_frame()
            frame_t = cv2.normalize(frame_t, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            shot = Snapshot(frame_v, frame_t)
            col_t, col_v = snapshots.draw_corners(shot)
            cv2.imshow("TOF", col_t)
            cv2.imshow("CAM", col_v)
            key = cv2.waitKey(100)
            if key == ord('q'):
                break
            if key == ord(' '):
                shot.find_corners()
                if shot.any_corners():
                    print("checkerboard found")
                    snapshots.add_shot(shot)
                else:
                    print("checkerboard not found")
    print(len(snapshots.shots))
    corner_pairs = snapshots.get_corner_pairs()
    print(len(corner_pairs[0]))

    t_corners = snapshots.get_t_corners()
    v_corners = snapshots.get_v_corners()
    with open("dcal.pkl","wb") as f:
        pickle.dump({"t":t_corners, "v": v_corners, "pairs": corner_pairs}, f)
else:
    with open("dcal.pkl","rb") as f:
        data = pickle.load(f)
        t_corners = data['t']
        v_corners = data['v']
        corner_pairs = data['pairs']

v_corners = [x for x in v_corners if x is not None]
t_corners = [x for x in t_corners if x is not None]
v_lens = Lens.from_cb_points(v_corners,(1280,800), fisheye=True)
t_lens = Lens.from_cb_points(t_corners,(240,180))
print(t_lens)
cams = DualCams(t_lens, v_lens)
cams.calibrate(corner_pairs[0], corner_pairs[1])
cams.save("both_cams.json")
print(cams)