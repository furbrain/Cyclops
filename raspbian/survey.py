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
import gtsam
A = gtsam.symbol_shorthand.A # aruco points
S = gtsam.symbol_shorthand.S # non-aruco survey points



def get_station_symbol(text:str):
    if len(text)==1 and text.isalpha():
        return A(ord(text.upper()) - ord("A"))
    else:
        try:
            return S(int(text))
        except ValueError:
            raise ValueError(f"Invalid station symbol '{text}': should be either a number or a single letter")


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
        print(pth)
        with open(pth) as f:
            for line in f:
                leg = self.parse_line(line)
                if leg is not None:
                    legs.append(leg)
        return legs

def leg_offset_and_covariance(leg: op.Leg):
    d = leg.distance
    sigma_d = leg.distance_noise
    az = np.radians(leg.azimuth)
    incl = np.radians(leg.inclination)
    sigma_az = leg.azimuth_noise
    sigma_incl = leg.inclination_noise
    ci, si = np.cos(incl), np.sin(incl)

    R = (sst.Rotation.from_euler('z', -az) * sst.Rotation.from_euler('x', incl)).as_matrix()
    # R columns are [e_az, e_d, e_incl] natively — no reordering needed
    offset = d * R[:,1]
    var_d = sigma_d ** 2
    var_az = (d * ci * sigma_az) ** 2 + (d * si * sigma_incl) ** 2  # blends to incl-scale near vertical
    var_incl = (d * sigma_incl) ** 2

    Sigma_local = np.diag([var_az, var_d, var_incl])  # matches R's column order
    return offset, R @ Sigma_local @ R.T

if __name__=="__main__":
    if False: #testing branch
        print("starting")
        parser = SVXParser()
        legs = parser.parse_file(Path(__file__).parent / "test.svx")
        for leg in legs:
            print(leg)
    elif True:
        pth = Path("/footage/storrs_aruco_3/")
        atlas = op.load_atlas(str(pth / "aruco.txt.gz"), binary=False)
        legs = SVXParser().parse_file(pth / "survey.svx")
        print(legs)
        atlas.add_survey(0, legs)
        print(atlas.surveys)
        op.save_atlas(atlas, str(pth / "atlas_all_data.txt.gz"), binary=False)
    else:
        l = op.Leg("1","2",10,0,0)#
        print(l)
        print(leg_offset_and_covariance(l))
        l = op.Leg("1","2",1,60,0)#
        print(l)
        print(leg_offset_and_covariance(l))
        l = op.Leg("1","2",1,315,30)#
        print(l)
        print(leg_offset_and_covariance(l))