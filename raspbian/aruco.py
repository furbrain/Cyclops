#!/usr/bin/env python3
import re
import tarfile
from pathlib import Path
from typing import Tuple, Iterable, Optional

import numpy as np

from atlas import Atlas, Keyframe
import cv2

BOARD_CORNERS = np.array([
    [ 1.0,  1.0],  # Top-Left
    [ 1.0, -1.0],  # Top-Right
    [-1.0, -1.0],  # Bottom-Right
    [-1.0,  1.0]   # Bottom-Left
], dtype=np.float32)

def get_midpoint(corners: np.ndarray) -> Tuple[float, float]:
        """
        Finds the intersection of two lines defined by points (p1, p2) and (p3, p4).
        Robust to horizontal and vertical lines.
        """
        H, _ = cv2.findHomography(BOARD_CORNERS, corners)
        return H[0:2,2]

def get_arucos(img, arucoDict, params):
    if img is None:
        return {}
    corners, ids, rejected = cv2.aruco.detectMarkers(img, arucoDict, parameters=params)
    if ids is None:
        return {}
    return {id[0]: get_midpoint(corner) for id, corner in zip(ids, corners)}


def normalise_name(name: str) -> str:
    """Strip zero padding so that 000005.jpg and 5.jpg both become 5.jpg"""
    return re.sub(r"^0*(?=\d)", "", name)


class ImageSource:
    """Reads grayscale images by filename from either a directory or a .tar file"""
    def __init__(self, images_dir: Path):
        self.tar = None
        # keys are normalised basenames, so any directory prefix inside the tar and any zero padding are ignored
        if images_dir.is_file() and tarfile.is_tarfile(images_dir):
            self.tar = tarfile.open(images_dir)
            self.files = {normalise_name(Path(m.name).name): m for m in self.tar.getmembers() if m.isfile()}
        else:
            self.files = {normalise_name(p.name): p for p in images_dir.iterdir() if p.is_file()}

    def read(self, name: str):
        entry = self.files.get(normalise_name(name))
        if entry is None:
            return None
        if self.tar is None:
            return cv2.imread(str(entry), cv2.IMREAD_GRAYSCALE)
        data = np.frombuffer(self.tar.extractfile(entry).read(), dtype=np.uint8)
        return cv2.imdecode(data, cv2.IMREAD_GRAYSCALE)

    def close(self):
        if self.tar is not None:
            self.tar.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()


def add_arucos(atlas: Atlas, images_dir: Path, keyframes: Optional[Iterable[Keyframe]] = None, kf_id_offset: int = 0):
    """Detect arucos in images_dir for keyframes (default all in atlas).
    Images are named by keyframe id less kf_id_offset"""
    if keyframes is None:
        keyframes = atlas.keyframes.values()
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    params = cv2.aruco.DetectorParameters_create()
    #params.minOtsuStdDev = 1.0
    #params.errorCorrectionRate = 0.8
    #params.polygonalApproxAccuracyRate = 0.05
    params.perspectiveRemovePixelPerCell = 1
    params.minMarkerPerimeterRate = 0.02
    with ImageSource(images_dir) as images:
        for kf in keyframes:
            imgL = images.read(f"{kf.id - kf_id_offset}.jpg")
            imgR = images.read(f"{kf.id - kf_id_offset}r.jpg")
            ar_l = get_arucos(imgL, arucoDict, params)
            ar_r = get_arucos(imgR, arucoDict, params)
            all_idx = set(ar_l.keys()) | set(ar_r.keys())
            kf.arucos.clear()
            for idx in all_idx:
                name = chr(ord('@')+idx)
                x_left = x_right = y_left = -1
                if idx in ar_r:
                    print(f"Adding Aruco Observation: {name} to Keyframe: {kf.id}R")
                    x_right, y_left = ar_r[idx]
                if idx in ar_l:
                    print(f"Adding Aruco Observation: {name} to Keyframe: {kf.id}L")
                    x_left, y_left = ar_l[idx]
                kf.arucos[name] = (float(x_left), float(y_left), float(x_right))

if __name__ == "__main__":
    pth = Path("/footage/storrs_aruco_4")
    atlas = Atlas.from_json_file(pth / "atlas.json.gz")
    print("finding arucos")
    add_arucos(atlas, pth / "images")
    atlas.save()
