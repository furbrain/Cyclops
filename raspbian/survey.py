#!/usr/bin/env python3
from collections import defaultdict
from pathlib import Path

import orb_slam3_py as op
import numpy as np
from typing import List, Tuple, Optional
import scipy.spatial.transform as sst

import gtsam

from atlas_tools import run_optimisation, Atlas, get_station_symbol
from ply_maker import PlyMaker

MERGE_NOISE_BASE = gtsam.noiseModel.Isotropic.Sigma(3, 0.1)
MERGE_NOISE = gtsam.noiseModel.Robust.Create(gtsam.noiseModel.mEstimator.Huber.Create(1.345), MERGE_NOISE_BASE)


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

def add_survey(atlas: Atlas, file_name: Path):
    legs = SVXParser().parse_file(file_name)
    atlas.atlas.add_survey(0, legs)

def add_legs_to_graph(atlas: Atlas, G: gtsam.NonlinearFactorGraph, values: gtsam.Values) -> None:
    legs: List[op.Leg]
    for idx, legs in atlas.atlas.surveys.items():
        for leg in legs:
            from_station = get_station_symbol(leg.from_station_id)
            to_station = get_station_symbol(leg.to_station_id)
            offset, noise = leg_offset_and_covariance(leg)
            noise_model = gtsam.noiseModel.Gaussian.Covariance(noise)
            G.add(gtsam.BetweenFactorPoint3(from_station, to_station,offset, noise_model))
            if not values.exists(from_station):
                values.insert(from_station, gtsam.Point3(np.random.rand(3)))
            if not values.exists(to_station):
                values.insert(to_station, gtsam.Point3(np.random.rand(3)))


def initialise_stations(atlas: Atlas) -> gtsam.Values:
    G = gtsam.NonlinearFactorGraph()
    factors: List[Tuple[gtsam.PinholePoseCal3_S2, Tuple[float,float], gtsam.Symbol]] = []
    for m in atlas.maps.values():
        for kf in m.kfs:
            factors.extend(kf.get_aruco_observations())
    cams = defaultdict(gtsam.CameraSetCal3_S2)
    observations = defaultdict(gtsam.Point2Vector)
    for cam, measurements, symbol in factors:
        cams[symbol].append(cam)
        observations[symbol].append(measurements)

    params = gtsam.TriangulationParameters()
    points = {}
    for symbol, cam in cams.items():
        point = gtsam.triangulateSafe(cam,observations[symbol], params)
        print(gtsam.Symbol(symbol).string(), point.valid())
        if point.valid():
            points[symbol] = point.get()
    values = gtsam.Values()
    for symbol, point in points.items():
        values.insert(symbol, point)
        G.add(gtsam.PriorFactorPoint3(symbol, point, MERGE_NOISE))
    add_legs_to_graph(atlas, G, values)
    print("initialising stations")
    #print(G)
    #print(values)
    result = run_optimisation(G, values)
    print(result)
    return result

def make_survey_ply(file_name: Path, atlas: Atlas, values: gtsam.Values) -> None:
    ply = PlyMaker()
    for idx, legs in atlas.atlas.surveys.items():
        for leg in legs:
            from_station = get_station_symbol(leg.from_station_id)
            from_pt = values.atPoint3(from_station)
            to_station = get_station_symbol(leg.to_station_id)
            to_pt = values.atPoint3(to_station)
            ply.make_cylinder(from_pt, to_pt, color=(255,255,0))
    for key in values.keys():
        symbol = gtsam.Symbol(key)
        if symbol.string().startswith("s"):
            ply.make_sphere(values.atPoint3(key), color=(255,0,0))
        elif symbol.string().startswith("a"):
            ply.make_sphere(values.atPoint3(key), color=(0, 0, 255))
    ply.write_ply(file_name)

if __name__=="__main__":
    pth = Path("/footage/storrs_aruco_4/")
    atlas = Atlas.from_file(pth/ "aruco.txt.gz")
    add_survey(atlas, pth / "survey.svx")
    atlas.save_file(pth / "atlas_all_data.txt.gz")
