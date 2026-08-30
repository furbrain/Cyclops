#!/usr/bin/env python3
import itertools
import re
from collections import defaultdict
from pathlib import Path
from typing import Tuple, List, Union, Dict

import orb_slam3_py as op
import numpy as np
import gtsam
import networkx as nx
import scipy.spatial.transform as sst

from orb_slam3_py import Map
from survey import get_station_symbol, leg_offset_and_covariance

CONNECTION_FILE = Path("/data/connection.txt")

M = gtsam.symbol_shorthand.M # map points
K = gtsam.symbol_shorthand.K # keyframes
R = gtsam.symbol_shorthand.R # right keyframes
T = gtsam.symbol_shorthand.T # transforms between maps
A = gtsam.symbol_shorthand.A # aruco points
S = gtsam.symbol_shorthand.S # non-aruco survey points

SMART_NOISE = gtsam.noiseModel.Isotropic.Sigma(2, 3.0)  # 1px noise
SMART_NOISE = gtsam.noiseModel.Robust.Create(gtsam.noiseModel.mEstimator.Huber.Create(1.345), SMART_NOISE)
IMU_NOISE = gtsam.noiseModel.Isotropic.Sigma(3, np.deg2rad(10)) #10 degrees
ANCHOR_NOISE = gtsam.noiseModel.Isotropic.Sigma(3, 10.0)  # anchor noise
RIG_NOISE = gtsam.noiseModel.Isotropic.Sigma(6, 1e-2)
MERGE_NOISE = gtsam.noiseModel.Isotropic.Sigma(3, 0.1)
MERGE_NOISE = gtsam.noiseModel.Robust.Create(gtsam.noiseModel.mEstimator.Huber.Create(1.345), MERGE_NOISE)


class KeyFrameData:
    def __init__(self, kf: op.KeyFrame):
        self.kf = kf
        self.id = kf.id
        self.K = kf.K()
        self.cal = gtsam.Cal3_S2(self.K[0, 0], self.K[1, 1], 0, self.K[0, 2], self.K[1, 2])
        self.all_descs = kf.get_descriptors()
        self.all_mps = kf.get_map_point_matches()
        self.all_kps = kf.get_keypoints_undistorted()
        self.right_kps = [(x, y[1]) for x,y in zip(kf.get_u_right(), kf.get_keypoints_undistorted())]
        self.mp_descs = np.array([desc for desc, mp in zip(self.all_descs, self.all_mps) if mp])
        self.mp_kps = {mp:kp for kp, mp in zip(self.all_kps, self.all_mps) if mp}
        self.mp_rkps = {mp:kp for kp, mp in zip(self.right_kps, self.all_mps) if mp and kp[0]>=0}
        self.mp_set = set(self.mp_kps.keys())
        self.imu_pose = gtsam.Pose3(kf.get_imu_estimate().inv().as_matrix())
        self.orig_pose = gtsam.Pose3(kf.get_pose().inv().as_matrix())
        self.stereo_offset = gtsam.Pose3(gtsam.Rot3(), [self.kf.baseline, 0, 0])
        print(self.kf.baseline)
        self.arucos: Dict[int, op.ArucoObservation] = {get_station_symbol(obs.id): obs for obs in
                                                       self.kf.aruco_observations}

    def get_rig_factor(self):
        return gtsam.BetweenFactorPose3(K(self.id), R(self.id), self.stereo_offset, RIG_NOISE)

    def get_imu_factor(self):
        return gtsam.PoseRotationPrior3D(K(self.id), self.imu_pose, IMU_NOISE)

    def get_mp_factors(self) -> List[gtsam.Factor]:
        factors: List[gtsam.GenericProjectionFactorCal3_S2] = []
        for mp, kp in self.mp_kps.items():
            # smart_factors[mp.id].add(np.array(kp), K(kf.id), 0)
            if mp in self.mp_rkps:
                factors.append(gtsam.GenericProjectionFactorCal3_S2(np.array(self.mp_rkps[mp]), SMART_NOISE,
                                                                              R(self.id), M(mp.id),
                                                                              self.cal))
                # smart_factors[mp.id].add(np.array(kf.mp_rkps[mp]), K(kf.id), 1)
            factors.append(gtsam.GenericProjectionFactorCal3_S2(np.array(kp), SMART_NOISE,
                                                                          K(self.id), M(mp.id),
                                                                          self.cal))
        return factors

    def get_aruco_factors(self) -> List[gtsam.Factor]:
        factors: List[gtsam.GenericProjectionFactorCal3_S2] = []
        obs: op.ArucoObservation
        for symbol, obs in self.arucos.items():
            print(f"adding {obs.id} to kf: {self.id}")
            if obs.xLeft >=0:
                factors.append(gtsam.GenericProjectionFactorCal3_S2((obs.xLeft, obs.yLeft),
                                                                    SMART_NOISE,
                                                                    K(self.id), symbol,
                                                                    self.cal))
            if obs.xRight >=0:
                factors.append(gtsam.GenericProjectionFactorCal3_S2((obs.xRight, obs.yLeft),
                                                                    SMART_NOISE,
                                                                    R(self.id), symbol,
                                                                    self.cal))
        return factors

    def get_aruco_observations(self):
        results = []
        for symbol, obs in self.arucos.items():
            if obs.xLeft >=0:
                left_cam = gtsam.PinholeCameraCal3_S2(self.orig_pose, self.cal)
                results.append([left_cam, (obs.xLeft, obs.yLeft), symbol])
            if obs.xRight >=0:
                right_cam = gtsam.PinholeCameraCal3_S2(self.orig_pose.compose(self.stereo_offset), self.cal)
                results.append([right_cam, (obs.xRight, obs.yLeft), symbol])
        return results


    def get_aruco_triangulation_factors(self) -> List[gtsam.TriangulationFactorCal3_S2]:
        factors: List[gtsam.TriangulationFactorCal3_S2] = []
        obs: op.ArucoObservation
        for cam, measurement, symbol in self.get_aruco_observations():
            factors.append(gtsam.TriangulationFactorCal3_S2(cam, measurement, SMART_NOISE, symbol))
        return factors


    def get_all_factors(self, include_survey: bool) -> List[gtsam.Factor]:
        factors = [self.get_rig_factor(), self.get_imu_factor()]
        factors.extend(self.get_mp_factors())
        if include_survey:
            factors.extend(self.get_aruco_factors())
        return  factors

    def initialise_pose(self, values: gtsam.Values, include_survey: bool = False):
        values.insert(K(self.id), self.orig_pose)
        values.insert(R(self.id), self.orig_pose.compose(self.stereo_offset))
        obs: op.ArucoObservation
        # if include_survey:
        #     for symbol in self.arucos.keys():
        #         if not values.exists(symbol):
        #             values.insert(symbol, self.orig_pose.translation())


class Map:
    def __init__(self, map: op.Map):
        self.map = map
        self.update_contents()

    def update_contents(self):
        self.kfs: List[KeyFrameData] = [KeyFrameData(kf) for kf in self.map.get_all_keyframes()]
        self.mps: List[op.MapPoint] = self.map.get_all_map_points()

    def update_values(self, values: gtsam.Values, transform: gtsam.Pose3):
        for kf in self.kfs:
            pose = transform.transformPoseTo(values.atPose3(K(kf.id)))
            kf.kf.set_pose(sst.RigidTransform.from_matrix(pose.inverse().matrix()))
            values.update(K(kf.id), pose)
        for mp in self.mps:
            point = transform.transformTo(values.atPoint3(M(mp.id)))
            mp.set_world_pos(point)
            values.update(M(mp.id), point)

    def merge_in(self, obsolete: "Map", connections: Dict[int, int]):
        for kf in obsolete.kfs:
            self.map.add_keyframe(kf.kf)
            obsolete.map.erase_keyframe(kf.kf)
        for mp in obsolete.mps:
            self.map.add_map_point(mp)
            obsolete.map.erase_map_point(mp)
        self.update_contents()
        mp_index = {mp.id: mp for mp in self.mps}
        for kf in self.kfs:
            for i,mp in enumerate(kf.all_mps):
                if mp is not None and mp.id in connections:
                    kf.kf.replace_map_point(i,mp_index[connections[mp.id]])
        for old in connections.keys():
            if old in mp_index:
                self.map.erase_map_point(mp_index[old])


def load_atlas(fname: Union[str, Path]) -> op.Atlas:
    print("loading atlas")
    atlas = op.load_atlas(str(fname), binary=False)
    op.align_atlas(atlas)
    for kf in atlas.get_all_keyframes():
        for i, mp in enumerate(kf.get_map_point_matches()):
            if mp is not None:
                mp.add_observation(kf, i)
    for kf in atlas.get_all_keyframes():
        kf.update_connections(False)
    print("Atlas loaded")
    return atlas


def load_poses(atlas: op.Atlas) -> List[Map]:
    maps ={m.get_id(): Map(m) for m in atlas.get_all_maps()}
    for id, m in maps.items():
        print(f"Loaded Map: {id} {min(k.id for k in m.kfs)} -> {max(k.id for k in m.kfs)}")
    return maps

def create_graph(maps: Dict[int, Map], include_survey=False) -> Tuple[
    gtsam.NonlinearFactorGraph, gtsam.Values]:
    G = gtsam.NonlinearFactorGraph()
    values = gtsam.Values()
    print("adding factors")
    kf_nearest_distance = 1e12
    kf_nearest_id = -1
    for m in maps.values():
        for kf in m.kfs:
            factors = kf.get_all_factors(include_survey)
            G.resize(G.size()+len(factors))
            for f in factors:
                G.add(f)
            kf.initialise_pose(values, include_survey)
            dist = np.linalg.norm(kf.orig_pose.translation())
            if dist < kf_nearest_distance:
                kf_nearest_distance = dist
                kf_nearest_id = kf.id
        for mp in m.mps:
            values.insert(M(mp.id), mp.get_world_pos())
    G.add(gtsam.PoseTranslationPrior3D(K(kf_nearest_id), gtsam.Pose3(), ANCHOR_NOISE))
    return G, values


def add_legs(atlas: op.Atlas, G: gtsam.NonlinearFactorGraph, values: gtsam.Values) -> None:
    legs: List[op.Leg]
    for idx, legs in atlas.surveys.items():
        for leg in legs:
            from_station = get_station_symbol(leg.from_station_id)
            to_station = get_station_symbol(leg.to_station_id)
            offset, noise = leg_offset_and_covariance(leg)
            noise_model = gtsam.noiseModel.Gaussian.Covariance(noise)
            print(f"{leg.from_station_id} -> {leg.to_station_id} {offset} {noise_model}")
            G.add(gtsam.BetweenFactorPoint3(from_station, to_station,offset, noise_model))
            if not values.exists(from_station):
                values.insert(from_station, gtsam.Point3(np.random.rand(3)))
            if not values.exists(to_station):
                values.insert(to_station, gtsam.Point3(np.random.rand(3)))

def initialise_stations(atlas: op.Atlas, maps: Dict[int, Map], ) -> None:
    G = gtsam.NonlinearFactorGraph()
    factors: List[gtsam.TriangulationFactorCal3_S2] = []
    for m in maps.values():
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
        print(symbol, point.valid())
        if point.valid():
            points[symbol] = point.get()
    values = gtsam.Values()
    for symbol, point in points.items():
        values.insert(symbol, point)
        G.add(gtsam.PriorFactorPoint3(symbol, point, MERGE_NOISE))
    add_legs(atlas, G, values)
    print("initialising stations")
    #print(G)
    #print(values)
    result = run_optimisation(G, values)
    print(result)
    return result


def add_connections(G: gtsam.NonlinearFactorGraph, values: gtsam.Values) -> Dict[Tuple[int,int], int]:
    t_index = 0
    factor_count = 0
    offset = np.zeros(3)
    mapping = {}
    src_map = 0
    dest_map = 0
    connections = defaultdict(list)
    with open(CONNECTION_FILE, "r") as f:
        for line in f:
            if match := re.match(r"# (\d+) -> (\d+)", line):
                src_map = int(match.group(1))
                dest_map = int(match.group(2))
                continue
            if match := re.match(r"(\d+), (\d+)", line):
                src_idx = int(match.group(1))
                dest_idx = int(match.group(2))
                connections[(src_map, dest_map)].append((src_idx, dest_idx))
                continue
            print("Unrecognized line: ", line)
    for i, (src_map, dest_map) in enumerate(sorted(connections.keys())):
        offset = np.zeros(3)
        for src_idx, dest_idx in connections[(src_map, dest_map)]:
            G.add(gtsam.ReferenceFrameFactorPoint3Pose3(M(src_idx), T(i), M(dest_idx), MERGE_NOISE))
            offset += values.atPoint3(M(dest_idx))
            offset -= values.atPoint3(M(src_idx))
        values.insert(T(i), gtsam.Pose3(gtsam.Rot3(),offset / len(connections[(src_map, dest_map)])))
    return connections

def print_errors(G, values):
    initial_error = G.error(values)
    print(f"total initial error: {initial_error}")
    for key in values.keys():
        symbol = gtsam.Symbol(key)
        if symbol.string().startswith("a"):
            print(symbol.string(), values.atPoint3(key))
        if symbol.string().startswith("s"):
            print(symbol.string(), values.atPoint3(key))

    errors = []
    for i in range(G.size()):
        factor = G.at(i)
        if factor is not None:
            err = factor.error(values)
            if np.isnan(err):
                print("NAN:  ", i, err, factor)
            errors.append((i, err))

    errors.sort(key=lambda x: -x[1])
    for i, e in errors[:10]:
        print(i, e, G.at(i))

def run_optimisation(G: gtsam.NonlinearFactorGraph, values: gtsam.Values) -> gtsam.Values:
    lm_params = gtsam.LevenbergMarquardtParams()
    lm_params.setMaxIterations(100)
    lm_params.setVerbosityLM("SUMMARY")
    lm_params.setLinearSolverType("MULTIFRONTAL_CHOLESKY")
    lm_params.setlambdaUpperBound(1e7)
    # lm_params.iterationHook =python_logging_hook
    print("creating optimisation")
    optimizer = gtsam.LevenbergMarquardtOptimizer(G, values, lm_params)
    print("running optimisation")
    final = optimizer.optimize()
    print("optimisation complete")
    return final

print("starting optimisation")
atlas = load_atlas("/footage/storrs_aruco_3/atlas_corrected.txt.gz")
print(atlas.surveys)
maps = load_poses(atlas)
graph, vals = create_graph(maps)
map_links = add_connections(graph, vals)
result = run_optimisation(graph, vals)
with open("/data/raw_values.gtsam", "w") as f:
     f.write(result.serialize())
with open("/data/raw_values.gtsam", "r") as f:
    result = gtsam.Values()
    result.deserialize(f.read())
for id, map in maps.items():
    print(f"Map: {id}")
    print(f"KFs: {min(x.id for x in map.kfs)} -> {max(x.id for x in map.kfs)}")
print("Transforming maps")
map_graph = nx.DiGraph()
map_indexes = {k: i for i, k in enumerate(sorted(map_links.keys()))}
map_graph.add_edges_from(sorted(map_links.keys()))
transforms: Dict[int, gtsam.Pose3] = {0: gtsam.Pose3()}
for source, dest in nx.bfs_edges(map_graph, source=0):
    tr = result.atPose3(T(map_indexes[(source,dest)]))
    print(tr, source, dest)
    transforms[dest] = tr.transformPoseFrom(transforms[source])
for idx, t in transforms.items():
    maps[idx].update_values(result, t)

print("Merging Maps")
links = [y for x in map_links for y in map_links[x]]
G = nx.Graph()
G.add_edges_from(links)
connections = {}
for group in nx.connected_components(G):
    dest = min(group)
    for x in list(sorted(group))[1:]:
        connections[x] = dest
min_map_id = min(maps.keys())
for map_id, map in maps.items():
    if map_id != min_map_id:
        maps[min_map_id].merge_in(map, connections)
        atlas.erase_map(map.map)
del maps
del graph
del vals

print("Final BA")
maps = load_poses(atlas)
station_positions = initialise_stations(atlas, maps)
G, values = create_graph(maps, include_survey=True)
#add_legs(atlas, G, values)
values.insert_or_assign(station_positions)
print_errors(G, values)
result = run_optimisation(G, values)
print_errors(G, result)
print("Saving results")
with open("/data/values.gtsam", "w") as f:
    f.write(result.serialize())
