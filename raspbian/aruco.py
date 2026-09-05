#!/usr/bin/env python3
from pathlib import Path
from typing import Tuple

import numpy as np

import orb_slam3_py as op
from atlas_tools import Atlas
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


def add_arucos(atlas: Atlas, images_dir: Path):
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    params = cv2.aruco.DetectorParameters_create()
    #params.minOtsuStdDev = 1.0
    #params.errorCorrectionRate = 0.8
    #params.polygonalApproxAccuracyRate = 0.05
    params.perspectiveRemovePixelPerCell = 1
    params.minMarkerPerimeterRate = 0.02
    for kf in atlas.kfs.values():
        path_l = images_dir / f"{kf.id}.jpg"
        path_r = images_dir / f"{kf.id}r.jpg"
        if path_l.exists():
            imgL = cv2.imread(str(path_l), cv2.IMREAD_GRAYSCALE)
        else:
            imgL = None
        if path_r.exists():
            imgR = cv2.imread(str(path_r), cv2.IMREAD_GRAYSCALE)
        else:
            imgR = None
        ar_l = get_arucos(imgL, arucoDict, params)
        ar_r = get_arucos(imgR, arucoDict, params)
        all_idx = set(ar_l.keys()) | set(ar_r.keys())
        kf.kf.clear_aruco_observations()
        for idx in all_idx:
            obs = op.ArucoObservation()
            obs.id = chr(ord('@')+idx)
            obs.xLeft = -1
            obs.xRight = -1
            if idx in ar_r:
                print(f"Adding Aruco Observation: {obs.id} to Keyframe: {kf.id}R")
                obs.xRight, obs.yLeft = ar_r[idx]
            if idx in ar_l:
                print(f"Adding Aruco Observation: {obs.id} to Keyframe: {kf.id}L")
                obs.xLeft, obs.yLeft = ar_l[idx]
            kf.kf.add_aruco_observation(obs)

if __name__ == "__main__":
    pth = Path("/footage/storrs_aruco_4")
    atlas = Atlas.from_file(pth / "atlas.txt.gz")
    print("finding arucos")
    add_arucos(atlas, pth / "images")
    atlas.save_file(pth/"atlas.txt.gz")
