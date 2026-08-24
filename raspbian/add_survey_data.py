#!/usr/bin/env python3
import argparse
from collections import defaultdict
from pathlib import Path

import orb_slam3_py as op
import cv2
import numpy as np
import itertools
from typing import List, Tuple, Any, Sequence, Dict, Set, Optional
import scipy.spatial.transform as sst

from orb_slam3_py import ArucoObservation


parser = argparse.ArgumentParser(description="Add found aruco codes to atlas",
                                 epilog="Survey file format is similar to standard Survex files, "
                                        "but only *DATA NORMAL and *SD are recognised. Other *COMMANDS "
                                        "are silently ignored. Semicolons indicate comments as normal")
parser.add_argument('-a', '--atlas', help="provide an atlas message file to use", required=True)
parser.add_argument('-d', '--images-dir', help="images directory", required=True)
parser.add_argument('-s', '--survey', help="survey file", required=True)
parser.add_argument('-v', '--verbose', help="verbose output", action='store_true')
opts = parser.parse_args()




def get_char(value: int) ->str:
    return chr(value + ord('A')-1)

def get_centre(corners: np.ndarray, K: np.ndarray) -> Tuple[float, float]:
    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.1, K, None)
    center_2d, _ = cv2.projectPoints(
        np.array([[0., 0., 0.]]),  # marker-frame center
        rvec, tvec, K, None
    )
    cx, cy = center_2d.ravel()
    return cx, cy

def find_markers(img: np.ndarray, K: np.ndarray, arucoDict, params) -> Dict[str, Tuple[int,int]]:
    all_corners, ids, _ = cv2.aruco.detectMarkers(img, arucoDict, parameters=params)
    if ids is None:
        return {}
    dct = {}
    for corners, id in zip(all_corners, ids):
        dct[get_char(id[0])] = get_centre(corners, K)
    return dct



def add_arucos(atlas):
    if opts.verbose:
        print("Starting aruco detection")
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    params = cv2.aruco.DetectorParameters_create()
    params.perspectiveRemovePixelPerCell = 1
    image_path = Path(opts.images_dir)
    kfs = atlas.get_all_keyframes()

    for kf in sorted(kfs, key=lambda x: x.id):

        img_path_l = image_path / f"{kf.id}.jpg"
        img_path_r = image_path / f"{kf.id}r.jpg"
        if not img_path_l.exists() or not img_path_r.exists():
            if opts.verbose:
                print(f"file not found for {kf.id}")
            continue
        img_l = cv2.imread(str(img_path_l), cv2.IMREAD_GRAYSCALE)
        img_r = cv2.imread(str(img_path_r), cv2.IMREAD_GRAYSCALE)
        if img_l is None or img_r is None:
            print(f"difficulty opening images for {kf.id}")
            continue
        if opts.verbose:
            print(f"Checking {kf.id}")
        l_markers = find_markers(img_l, kf.K(), arucoDict, params)
        r_markers = find_markers(img_r, kf.K(), arucoDict, params)
        all_ids = l_markers.keys() | r_markers.keys()
        kf.clear_aruco_observations()
        if all_ids:
            print(f"Found {all_ids} markers in {kf.id}")
            for id in all_ids:
                ob = ArucoObservation()
                ob.id = id
                if id in l_markers:
                    ob.xLeft, ob.yLeft = l_markers[id]
                    if id in r_markers:
                        ob.xRight = r_markers[id][0]
                    else:
                        print("missed right image")
                        ob.xRight = -1
                else:
                    print("missed left image")
                    ob.xRight, ob.yLeft = r_markers[id]
                    ob.xLeft = -1
                kf.add_aruco_observation(ob)

class SVXParser:
    DEFAULT_ORDER = ("FROM", "TO", "TAPE", "COMPASS", "CLINO")

    def __init__(self):
        self.mode = "NORMAL"
        self.order = self.DEFAULT_ORDER
        self.tape_noise = 0.0025 # ±5mm
        self.azimuth_noise = 0.00873 # ±1°
        self.inclination_noise = 0.00436 # ±0.5°

    def parse_line(self, line:str) -> Optional[op.Leg]:
        # remove comments
        idx = line.find(';')
        if idx >= 0:
            line = line[:idx]
        line = line.strip()
        if len(line)==0:
            return None
        if line.upper().startswith('*DATA'):
            tokens = line.upper().split()
            if len(tokens) < 2:
                return None
            self.mode = tokens[1]
            if tokens[1] == "NORMAL":
                self.order = tokens[2:]
                if len(self.order) != len(self.DEFAULT_ORDER):
                    raise ValueError("Not enough reading types: {line}")
                if set(self.order) != set(self.DEFAULT_ORDER):
                    raise ValueError("Incorrect reading types: {line}")
            return None
        if line.upper().startswith("*SD"):
            tokens = line.upper().split()
            if len(tokens) < 3:
                return None
            if tokens[1] == "TAPE":
                self.tape_noise = float(tokens[2])
            elif tokens[1] == "COMPASS":
                self.azimuth_noise = np.deg2rad(float(tokens[2]))
            elif tokens[1] == "CLINO":
                self.inclination_noise = np.deg2rad(float(tokens[2]))
            return None
        if line.startswith("*"):
            return None

        #everything else at this point should be data reading
        if self.mode != "NORMAL":
            return None
        values = line.split()
        if len(values) < 5:
            raise ValueError("Not enough reading values: {line}")
        data = {reading: value for reading, value in zip(self.order, values[:5])}
        leg = op.Leg(from_station_id=data['FROM'],
                     to_station_id=data['TO'],
                     distance=float(data['TAPE']),
                     azimuth=float(data['COMPASS']),
                     inclination=float(data['CLINO']),
                     distance_noise=self.tape_noise,
                     azimuth_noise=self.azimuth_noise,
                     inclination_noise=self.inclination_noise)
        return leg

    def parse_file(self, pth: Path):
        legs = []
        with open(pth) as f:
            for line in f:
                leg = self.parse_line(line)
                if leg is not None:
                    legs.append(leg)
        return legs

#atlas = op.load_atlas(opts.atlas)
#add_arucos(atlas)
parser = SVXParser()
legs = parser.parse_file(Path(opts.survey))
for leg in legs:
    print(leg)