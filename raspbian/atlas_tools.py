from pathlib import Path
from typing import Dict, List, Tuple

import gtsam
import numpy as np
from scipy.spatial import transform as sst

import orb_slam3_py as op

M = gtsam.symbol_shorthand.M # map points
K = gtsam.symbol_shorthand.K # keyframes
R = gtsam.symbol_shorthand.R # right keyframes
T = gtsam.symbol_shorthand.T # transforms between maps
A = gtsam.symbol_shorthand.A # aruco points
S = gtsam.symbol_shorthand.S # non-aruco survey points
SMART_NOISE_BASE = gtsam.noiseModel.Isotropic.Sigma(2, 3.0)  # 1px noise
SMART_NOISE = gtsam.noiseModel.Robust.Create(gtsam.noiseModel.mEstimator.Huber.Create(1.345), SMART_NOISE_BASE)
IMU_NOISE = gtsam.noiseModel.Isotropic.Sigma(3, np.deg2rad(10)) #10 degrees
ANCHOR_NOISE = gtsam.noiseModel.Isotropic.Sigma(3, 10.0)  # anchor noise
RIG_NOISE = gtsam.noiseModel.Isotropic.Sigma(6, 1e-2)


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
        self.just_mps = [mp for mp in self.all_mps if mp]
        self.mp_kps = {mp:kp for kp, mp in zip(self.all_kps, self.all_mps) if mp}
        self.mp_rkps = {mp:kp for kp, mp in zip(self.right_kps, self.all_mps) if mp and kp[0]>=0}
        self.mp_set = set(self.mp_kps.keys())
        self.imu_pose = gtsam.Pose3(kf.get_imu_estimate().inv().as_matrix())
        self.orig_pose = gtsam.Pose3(kf.get_pose().inv().as_matrix())
        self.stereo_offset = gtsam.Pose3(gtsam.Rot3(), [self.kf.baseline, 0, 0])
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
            print("adding aruco factor obs")
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


    # def get_aruco_triangulation_factors(self) -> List[gtsam.TriangulationFactorCal3_S2]:
    #     factors: List[gtsam.TriangulationFactorCal3_S2] = []
    #     obs: op.ArucoObservation
    #     for cam, measurement, symbol in self.get_aruco_observations():
    #         factors.append(gtsam.TriangulationFactorCal3_S2(cam, measurement, SMART_NOISE, symbol))
    #     return factors


    def get_all_factors(self, include_survey: bool) -> List[gtsam.Factor]:
        factors = [self.get_rig_factor(), self.get_imu_factor()]
        factors.extend(self.get_mp_factors())
        if include_survey:
            factors.extend(self.get_aruco_factors())
        return  factors

    def initialise_pose(self, values: gtsam.Values):
        values.insert(K(self.id), self.orig_pose)
        values.insert(R(self.id), self.orig_pose.compose(self.stereo_offset))
        for key in self.arucos:
            if not values.exists(key):
                values.insert(key, gtsam.Point3(np.random.rand(3)))


class Map:
    def __init__(self, map: op.Map):
        self.id = map.get_id()
        self.map = map
        self.update_contents()

    def update_contents(self):
        self.kfs: List[KeyFrameData] = [KeyFrameData(kf) for kf in self.map.get_all_keyframes()]
        self.mps: List[op.MapPoint] = self.map.get_all_map_points()

    def update_values(self, values: gtsam.Values, transform: gtsam.Pose3 = None):
        if transform is None:
            transform = gtsam.Pose3() # use identity if no transform specified
        for kf in self.kfs:
            pose = transform.transformPoseTo(values.atPose3(K(kf.id)))
            kf.kf.set_pose(sst.RigidTransform.from_matrix(pose.inverse().matrix()))
        for mp in self.mps:
            point = transform.transformTo(values.atPoint3(M(mp.id)))
            mp.set_world_pos(point)

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

class Atlas:
    def __init__(self, atlas:op.Atlas):
        self.atlas = atlas
        self.kfs = {kf.id: KeyFrameData(kf) for kf in self.atlas.get_all_keyframes()}
        self.maps = {mapp.get_id(): Map(mapp) for mapp in self.atlas.get_all_maps()}

    def reload(self):
        self.kfs = {kf.id: KeyFrameData(kf) for kf in self.atlas.get_all_keyframes()}
        self.maps = {mapp.get_id(): Map(mapp) for mapp in self.atlas.get_all_maps()}

    @classmethod
    def from_file(cls, path: Path) -> "Atlas":
        binary = str(path).endswith(".osa")
        return Atlas(op.load_atlas(str(path), binary=binary))

    def save_file(self, path: Path) -> None:
        binary = str(path).endswith(".osa")
        op.save_atlas(self.atlas, str(path), binary=binary)

    def create_graph(self, include_survey=False) -> Tuple[gtsam.NonlinearFactorGraph, gtsam.Values]:
        G = gtsam.NonlinearFactorGraph()
        values = gtsam.Values()
        print("adding factors")
        for m in self.maps.values():
            kf_nearest_distance = 1e12
            kf_nearest_id = -1
            kf_nearest_pose: gtsam.Pose3 = None
            for kf in m.kfs:
                factors = kf.get_all_factors(include_survey)
                G.resize(G.size() + len(factors))
                for f in factors:
                    G.add(f)
                kf.initialise_pose(values)
                dist = np.linalg.norm(kf.orig_pose.translation())
                if dist < kf_nearest_distance:
                    kf_nearest_distance = dist
                    kf_nearest_id = kf.id
                    kf_nearest_pose = kf.orig_pose
            for mp in m.mps:
                values.insert_point3(M(mp.id), mp.get_world_pos())
            #this adds a translation anchor to the point in each map nearest the origin
            G.add(gtsam.PoseTranslationPrior3D(K(kf_nearest_id), kf_nearest_pose, ANCHOR_NOISE))
        return G, values


def run_optimisation(G: gtsam.NonlinearFactorGraph, values: gtsam.Values, iters: int = 100) -> gtsam.Values:
    lm_params = gtsam.LevenbergMarquardtParams()
    lm_params.setMaxIterations(iters)
    #lm_params.setlambdaFactor(np.sqrt(10))
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


def get_station_symbol(text:str):
    if len(text)==1 and text.isalpha():
        return A(ord(text.upper()) - ord("A"))
    else:
        try:
            return S(int(text))
        except ValueError:
            raise ValueError(f"Invalid station symbol '{text}': should be either a number or a single letter")
