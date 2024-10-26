import os
import pickle
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, List, ClassVar, Dict

import cv2
import numpy as np

SHARP_WINDOW = 200
MAX_SIZE = 1280




@dataclass
class Frame:
    project_dir: Path
    orig_img: Optional[np.ndarray] = None
    orig_mask: Optional[np.ndarray] = None
    kp: Optional[List[cv2.KeyPoint]] = None
    desc: Optional[np.ndarray] = None
    ts: int = 0
    width: int = field(init=False)
    height: int = field(init=False)
    img: np.ndarray = field(init=False)
    mask: Optional[np.ndarray] = field(init=False, default=None)
    orb: ClassVar[cv2.ORB] = cv2.ORB.create()
    matcher: ClassVar[cv2.BFMatcher] = cv2.BFMatcher.create(cv2.NORM_HAMMING)

    def __post_init__(self):
        if self.orig_img is not None:
            self.prepare_image()

    def compute_kps(self):
        self.kp, self.desc = self.orb.detectAndCompute(self.img, self.mask)

    def prepare_image(self):
        h,w = self.orig_img.shape[:2]
        size = max(h,w)
        self.width = (MAX_SIZE * w) // size
        self.height = (MAX_SIZE * h) // size
        self.img = cv2.resize(self.orig_img, (self.width, self.height))
        if self.orig_mask is not None:
            self.mask = cv2.resize(self.orig_mask, (self.width, self.height))
        if len(self.orig_img.shape)>=3 and self.orig_img.shape[-1] != 1:
            self.img = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)

    def save_image(self):
        ident = f"{self.ts:05}"
        (self.project_dir / "images").mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(self.project_dir / "images" / f"{ident}.jpg"), self.orig_img)

    def save_descs(self):
        ident = f"{self.ts:05}"
        (self.project_dir / "keypoints").mkdir(parents=True, exist_ok=True)
        with open(self.project_dir / "keypoints" / f"{ident}.kps", "wb") as f:
            pickle.dump(self.kp, f)
        with open(self.project_dir / "keypoints" / f"{ident}.desc", "wb") as f:
            np.save(f, self.desc)

    @property
    def ident(self):
        return f"{self.ts:05}"

    def load_image(self, get_mask=True):
        (self.project_dir / "images").mkdir(parents=True, exist_ok=True)
        self.orig_img = cv2.imread(str(self.project_dir / "images" / f"{self.ident}.jpg"))
        if get_mask:
            self.orig_mask = cv2.imread(str(self.project_dir / "masks" / f"{self.ident}.jpg.png"), cv2.IMREAD_GRAYSCALE)
        self.prepare_image()

    def load_descs(self):
        (self.project_dir / "keypoints").mkdir(parents=True, exist_ok=True)
        with open(self.project_dir / "keypoints" / f"{self.ident}.kps", "rb") as f:
            self.kp = pickle.load(f)
        with open(self.project_dir / "keypoints" / f"{self.ident}.desc", "rb") as f:
            self.desc = np.load(f)

    def compare(self, other:"Frame"):
        if self.desc is None or other.desc is None:
            return [],[],0
        matches = self.matcher.knnMatch(self.desc, other.desc, k=2)
        try:
            good_matches: List[cv2.DMatch] = [m for m,n in matches if m.distance < 0.75 * n.distance]
        except ValueError:
            return [],[],0
        new_idxs = [x.queryIdx for x in good_matches]
        old_idxs = [x.trainIdx for x in good_matches]
        if good_matches:
            new_points = cv2.KeyPoint.convert(self.kp, new_idxs)
            old_points = cv2.KeyPoint.convert(other.kp, old_idxs)
            distance = np.mean(np.linalg.norm(new_points - old_points, axis=1))
        else:
            new_points = np.ndarray((0,0))
            old_points = np.ndarray((0,0))
            distance = np.Inf
        return new_points, old_points, distance

    def link_mask(self):
        target = self.project_dir / "masks" / f"{self.ident}.jpg.png"
        if os.path.exists(target):
            os.unlink(target)
        try:
            os.symlink(self.project_dir / "masks" / "mask.png",target)
        except OSError:
            pass

@dataclass
class FrameSet:
    project_dir: Path
    frames: Dict[int, Frame] = field(init=False, default_factory=dict)

    def find_frames(self):
        image_dir = (self.project_dir / "images")
        image_dir.mkdir(parents=True, exist_ok=True)
        timestamps = [int(x.stem) for x in image_dir.glob("*.jpg")]
        self.frames = {x:Frame(self.project_dir,ts=x) for x in timestamps}


    def add_frame(self, frame:Frame):
        self.frames[frame.ts] = frame

    def save_images(self):
        for frame in self.frames.values():
            frame.save_image()

    def save_descs(self):
        for frame in self.frames.values():
            frame.save_descs()

    def load_images(self, get_masks=True):
        for frame in self.frames.values():
            frame.load_image(get_mask=get_masks)

    def load_descs(self):
        for frame in self.frames.values():
            frame.load_descs()

    def link_masks(self):
        for frame in self.frames.values():
            frame.link_mask()

