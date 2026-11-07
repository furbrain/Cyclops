from collections import defaultdict
from dataclasses import dataclass, field
import gzip
import os
from dataclasses_json import config, DataClassJsonMixin
from pathlib import Path
from typing import List, Dict, Optional, Tuple, Set
import scipy.spatial.transform as sst
import numpy as np
from numpy.lib.format import open_memmap
import gtsam

M = gtsam.symbol_shorthand.M # map points
K = gtsam.symbol_shorthand.K # keyframes
R = gtsam.symbol_shorthand.R # right keyframes
T = gtsam.symbol_shorthand.T # transforms between maps
A = gtsam.symbol_shorthand.A # aruco points
S = gtsam.symbol_shorthand.S # non-aruco survey points
SMART_NOISE_BASE = gtsam.noiseModel.Isotropic.Sigma(2, 3.0)  # 1px noise
SMART_NOISE = SMART_NOISE_BASE
#SMART_NOISE = gtsam.noiseModel.Robust.Create(gtsam.noiseModel.mEstimator.Huber.Create(1.345), SMART_NOISE_BASE)
IMU_NOISE = gtsam.noiseModel.Isotropic.Sigma(3, np.deg2rad(10)) #10 degrees
ANCHOR_NOISE = gtsam.noiseModel.Isotropic.Sigma(3, 0.1)  # anchor noise
RIG_NOISE = gtsam.noiseModel.Isotropic.Sigma(6, 1e-2)


# encoder stops dataclasses_json recursing into the field (exclude is only applied afterwards)
EXCLUDE = config(encoder=lambda _: None, exclude=lambda _: True)
RIGID = config(encoder=lambda t: t.as_matrix().tolist(), decoder=lambda m: sst.RigidTransform.from_matrix(np.array(m)))
def ARRAY(dtype):
    return config(encoder=lambda a: a.tolist(), decoder=lambda l: np.array(l, dtype=dtype))


@dataclass
class CameraIntrinsics(DataClassJsonMixin):
    size: Tuple[int, int]
    K: np.ndarray[Tuple[3, 3], np.float32] = field(metadata=ARRAY(np.float32))

@dataclass
class MapPoint(DataClassJsonMixin):
    id: int
    pos: Tuple[float, float, float]
    _atlas: Optional["Atlas"] = field(init=False, default=None, metadata=EXCLUDE)

    @classmethod
    def from_osa(cls, mp: "op.MapPoint") -> "MapPoint":
        return cls(
            id=mp.id,
            pos=tuple(mp.get_world_pos().flatten().tolist()),
        )

@dataclass
class Keyframe(DataClassJsonMixin):
    id: int
    pose: sst.RigidTransform = field(metadata=RIGID)
    imu_pose: sst.RigidTransform = field(metadata=RIGID)
    key_points: np.ndarray[Tuple[int, 3], np.float32] = field(metadata=ARRAY(np.float32)) # x,y, x_right
    baseline: float
    _map_points: List[int]
    _camera: int
    arucos: dict[str, Tuple[float, float, float]] = field(default_factory=dict)
    _atlas: Optional["Atlas"] = field(init=False, default=None, metadata=EXCLUDE, repr=False)
    descriptors: np.ndarray[Tuple[int, 32], np.uint8] = field(init=False, default=None, metadata=EXCLUDE)

    def set_atlas(self, atlas: "Atlas"):
        self._atlas = atlas
        self.camera = atlas.cameras[self._camera]
        self.map_points = [(atlas.map_points[mp_id] if mp_id >= 0 else None) for mp_id in self._map_points]

    def get_gtsam_pose(self) -> gtsam.Pose3:
        return gtsam.Pose3(self.pose.as_matrix())

    def set_gtsam_pose(self, pose: gtsam.Pose3):
        self.pose = sst.RigidTransform.from_matrix(pose.matrix())

    def get_cal(self) -> gtsam.Cal3_S2:
        k = self.camera.K
        return gtsam.Cal3_S2(k[0, 0], k[1, 1], 0, k[0, 2], k[1, 2])

    def get_stereo_offset(self) -> gtsam.Pose3:
        return gtsam.Pose3(gtsam.Rot3(), [self.baseline, 0, 0])

    def get_rig_factor(self):
        return gtsam.BetweenFactorPose3(K(self.id), R(self.id), self.get_stereo_offset(), RIG_NOISE)

    def get_imu_factor(self):
        return gtsam.PoseRotationPrior3D(K(self.id), gtsam.Pose3(self.imu_pose.as_matrix()), IMU_NOISE)

    def get_mp_factors(self) -> List[gtsam.Factor]:
        cal = self.get_cal()
        # keyed by map point so each point is only observed once per keyframe
        mp_kps = {mp.id: kp for kp, mp in zip(self.key_points, self.map_points) if mp is not None}
        factors: List[gtsam.Factor] = []
        for mp_id, (x, y, x_right) in mp_kps.items():
            if x_right >= 0:
                factors.append(gtsam.GenericProjectionFactorCal3_S2(np.array([x_right, y], dtype=float), SMART_NOISE,
                                                                    R(self.id), M(mp_id), cal))
            factors.append(gtsam.GenericProjectionFactorCal3_S2(np.array([x, y], dtype=float), SMART_NOISE,
                                                                K(self.id), M(mp_id), cal))
        return factors

    def get_aruco_factors(self) -> List[gtsam.Factor]:
        cal = self.get_cal()
        factors: List[gtsam.Factor] = []
        for name, (x_left, y_left, x_right) in self.arucos.items():
            symbol = get_station_symbol(name)
            if x_left >= 0:
                factors.append(gtsam.GenericProjectionFactorCal3_S2((x_left, y_left), SMART_NOISE,
                                                                    K(self.id), symbol, cal))
            if x_right >= 0:
                factors.append(gtsam.GenericProjectionFactorCal3_S2((x_right, y_left), SMART_NOISE,
                                                                    R(self.id), symbol, cal))
        return factors

    def get_all_factors(self, include_survey: bool) -> List[gtsam.Factor]:
        factors = [self.get_rig_factor(), self.get_imu_factor()]
        factors.extend(self.get_mp_factors())
        if include_survey:
            factors.extend(self.get_aruco_factors())
        return factors

    def initialise_pose(self, values: gtsam.Values, include_survey: bool):
        pose = self.get_gtsam_pose()
        values.insert(K(self.id), pose)
        values.insert(R(self.id), pose.compose(self.get_stereo_offset()))
        if include_survey:
            for name in self.arucos:
                key = get_station_symbol(name)
                if not values.exists(key):
                    values.insert(key, gtsam.Point3(np.random.rand(3)))

    @classmethod
    def from_osa(cls, kf: "op.KeyFrame") -> "Keyframe":
        kps = np.array(kf.get_keypoints_undistorted(), dtype=np.float32)
        kps_right = np.array(kf.get_u_right(), dtype=np.float32).reshape(-1, 1)

        mps = [(mp.id if mp else -1) for mp in kf.get_map_point_matches()]
        return cls(
            id=kf.id,
            pose=kf.get_pose_inverse(),
            imu_pose=kf.get_imu_estimate().inv(),
            key_points=np.column_stack([kps, kps_right]),
            baseline=kf.baseline,
            _map_points=mps,
            _camera=0, ## FIXME kf.camera_id,
            arucos={str(obs.id): (obs.xLeft, obs.yLeft, obs.xRight) for obs in kf.aruco_observations},
        )

@dataclass
class Map(DataClassJsonMixin):
    id: int
    keyframes: Dict[int, Keyframe]
    _atlas: Optional["Atlas"] = field(init=False, default=None, metadata=EXCLUDE, repr=False)

    def set_atlas(self, atlas: "Atlas"):
        self._atlas = atlas
        for kf in self.keyframes.values():
            kf.set_atlas(atlas)

    def map_point_ids(self) -> Set[int]:
        return {mp.id for kf in self.keyframes.values() for mp in kf.map_points if mp is not None}

    def add_to_graph(self, G: gtsam.NonlinearFactorGraph, values: gtsam.Values, include_survey: bool):
        nearest_kf = None
        nearest_distance = 1e12
        for kf in self.keyframes.values():
            factors = kf.get_all_factors(include_survey)
            for f in factors:
                G.add(f)
            kf.initialise_pose(values, include_survey)
            dist = np.linalg.norm(kf.pose.translation)
            if dist < nearest_distance:
                nearest_distance = dist
                nearest_kf = kf
        for mp_id in self.map_point_ids():
            values.insert(M(mp_id), np.array(self._atlas.map_points[mp_id].pos))
        # this adds a translation anchor to the keyframe in each map nearest the origin
        G.add(gtsam.PoseTranslationPrior3D(K(nearest_kf.id), nearest_kf.get_gtsam_pose(), ANCHOR_NOISE))

    def update_values(self, values: gtsam.Values):
        for kf in self.keyframes.values():
            kf.set_gtsam_pose(values.atPose3(K(kf.id)))
        for mp_id in self.map_point_ids():
            self._atlas.map_points[mp_id].pos = tuple(values.atPoint3(M(mp_id)).tolist())

    def apply_rotation(self, rotation: sst.Rotation):
        """Rotate the whole map (keyframe poses and map points) about the world origin"""
        transform = sst.RigidTransform.from_rotation(rotation)
        for kf in self.keyframes.values():
            kf.pose = transform * kf.pose
        for mp_id in self.map_point_ids():
            mp = self._atlas.map_points[mp_id]
            mp.pos = tuple(rotation.apply(mp.pos).tolist())

    def align_to_imu(self) -> sst.Rotation:
        """Rotate the map so the keyframe orientations best match their IMU orientations, in the
        least squares (chordal) sense. Returns the rotation applied"""
        if not self.keyframes:
            return sst.Rotation.identity()
        kfs = list(self.keyframes.values())
        slam_rots = sst.Rotation.concatenate([kf.pose.rotation for kf in kfs])
        imu_rots = sst.Rotation.concatenate([kf.imu_pose.rotation for kf in kfs])
        # we want the single rotation Q such that Q * slam_rot ~= imu_rot for every keyframe
        rotation = (imu_rots * slam_rots.inv()).mean()
        self.apply_rotation(rotation)
        return rotation

    @classmethod
    def from_osa(cls, mapp: "op.Map") -> "Map":
        return cls(
            id=mapp.get_id(),
            keyframes={kf.id: Keyframe.from_osa(kf) for kf in mapp.get_all_keyframes()},
        )

@dataclass
class SurveyLeg(DataClassJsonMixin):
    from_station: str
    to_station: str
    azimuth: float
    inclination: float
    distance: float

@dataclass
class SurveyStation(DataClassJsonMixin):
    name: str
    pos: Tuple[float, float, float]

@dataclass
class Survey(DataClassJsonMixin):
    stations: Dict[str, SurveyStation]
    legs: List[SurveyLeg]

@dataclass
class SubSurvey(DataClassJsonMixin):
    """A sub-survey folder (relative to the atlas), holding descriptors.npy and images.tar for some of its keyframes"""
    path: str
    survex_prefix: str
    kf_offsets: Dict[int, int] # row in descriptors.npy where each keyframe's descriptors start
    kf_id_offset: int = 0 # amount keyframe ids have been increased by since loading from this folder

    def descriptors_file(self, atlas_dir: Path) -> Path:
        return atlas_dir / self.path / "descriptors.npy"

    def images_file(self, atlas_dir: Path) -> Path:
        return atlas_dir / self.path / "images.tar"

@dataclass
class Atlas(DataClassJsonMixin):
    maps: Dict[int, Map]
    map_points: Dict[int, MapPoint]
    cameras: Dict[int, CameraIntrinsics]
    surveys: Dict[int, Survey] = field(default_factory=dict)
    # if empty, descriptors.npy and images.tar are alongside the atlas
    sub_surveys: List[SubSurvey] = field(default_factory=list)
    merged: bool = False
    dir: Path = field(init=False, default=None,  metadata=EXCLUDE)
    keyframes: Dict[int, Keyframe] = field(init=False, default_factory=dict, metadata=EXCLUDE)
    kf_offsets: Dict[int, int] = field(init=False, default_factory=dict, metadata=EXCLUDE)
    descriptors: np.ndarray[Tuple[int, 32], np.uint8] = field(init=False, default=None, metadata=EXCLUDE)
    desc_size: int = field(init=False, default=0, metadata=EXCLUDE)

    def __post_init__(self):
        for map in self.maps.values():
            map.set_atlas(self)
            self.keyframes.update(map.keyframes)
        for mp in self.map_points.values():
            mp._atlas = self
        self.set_kf_offsets()

    def set_kf_offsets(self):
        self.kf_offsets = {}
        i = 0
        for kf_id in sorted(self.keyframes.keys()):
            self.kf_offsets[kf_id] = i
            i += len(self.keyframes[kf_id].key_points)
        self.desc_size = i

    @classmethod
    def from_osa(cls, fname: Path) -> "Atlas":
        import orb_slam3_py as op
        binary = fname.suffix == ".osa"
        atl = op.load_atlas(str(fname), binary=binary)
        mps = {mp.id: MapPoint.from_osa(mp) for mp in atl.get_all_map_points()}
        maps = {mapp.get_id(): Map.from_osa(mapp) for mapp in atl.get_all_maps()}
        #FIXME cameras: assumes all keyframes share one camera, and the image size is not available
        first_kf = next(iter(atl.get_all_keyframes()))
        obj = cls(
            maps=maps,
            map_points=mps,
            cameras={0: CameraIntrinsics(size=(640, 480), K=np.array(first_kf.K(), dtype=np.float32))},
        )
        obj.dir = fname.parent
        obj.descriptors = open_memmap(fname.parent / "descriptors.npy", dtype=np.uint8, mode="w+", shape=(obj.desc_size, 32))
        obj.set_kf_descriptor_views()
        for mapp in atl.get_all_maps():
            for kf in mapp.get_all_keyframes():
                obj.keyframes[kf.id].descriptors[:] = kf.get_descriptors()
        obj.descriptors.flush()
        return obj

    @classmethod
    def from_dir(cls, dir: Path) -> "Atlas":
        if (dir / "atlas.json.gz").exists():
            return cls.from_json_file(dir / "atlas.json.gz")
        return cls.from_osa(dir / "atlas.osa")

    @classmethod
    def from_json_file(cls, fname: Path) -> "Atlas":
        with gzip.open(fname, "rt") as f:
            obj = cls.from_json(f.read())
        obj.dir = fname.parent
        if obj.sub_surveys:
            for sub_survey in obj.sub_surveys:
                descriptors = np.load(sub_survey.descriptors_file(obj.dir), mmap_mode="r")
                for kf_id, offset in sub_survey.kf_offsets.items():
                    kf = obj.keyframes[kf_id]
                    kf.descriptors = descriptors[offset:offset + len(kf.key_points)]
        else:
            obj.descriptors = np.load(fname.parent / "descriptors.npy", mmap_mode="r")
            obj.set_kf_descriptor_views()
        return obj

    def save(self, fname: Optional[Path] = None):
        if fname is None:
            fname = self.dir / "atlas.json.gz"
        if self.descriptors is not None:
            self.descriptors.flush()
        # write then rename, so a failure part way through doesn't destroy an existing atlas
        tmp = fname.with_name(fname.name + ".tmp")
        with gzip.open(tmp, "wt") as f:
            f.write(self.to_json())
        os.replace(tmp, fname)

    def get_sub_surveys(self) -> List[SubSurvey]:
        if self.sub_surveys:
            return self.sub_surveys
        if self.keyframes:
            return [SubSurvey(".", "", dict(self.kf_offsets))]
        return []

    def find_arucos(self):
        from aruco import add_arucos # imported here as aruco imports this module
        for sub_survey in self.get_sub_surveys():
            images = sub_survey.images_file(self.dir)
            if images.exists():
                print(f"finding arucos in {images}")
                keyframes = [self.keyframes[kf_id] for kf_id in sub_survey.kf_offsets]
                add_arucos(self, images, keyframes, sub_survey.kf_id_offset)


    def get_aruco_positions(self) -> Dict[str, Tuple[float, float, float]]:
        """Triangulate the world position of each aruco marker from all its sightings in all keyframes.
        Both the left and right camera observations are used. Markers that can't be triangulated
        (e.g. too few sightings or too little parallax) are omitted."""
        cams: Dict[str, gtsam.CameraSetCal3_S2] = defaultdict(gtsam.CameraSetCal3_S2)
        observations: Dict[str, gtsam.Point2Vector] = defaultdict(gtsam.Point2Vector)
        for kf in self.keyframes.values():
            cal = kf.get_cal()
            left_pose = kf.get_gtsam_pose()
            right_pose = left_pose.compose(kf.get_stereo_offset())
            for name, (x_left, y, x_right) in kf.arucos.items():
                if x_left >= 0:
                    cams[name].append(gtsam.PinholeCameraCal3_S2(left_pose, cal))
                    observations[name].append(np.array([x_left, y], dtype=float))
                if x_right >= 0:
                    cams[name].append(gtsam.PinholeCameraCal3_S2(right_pose, cal))
                    observations[name].append(np.array([x_right, y], dtype=float))

        params = gtsam.TriangulationParameters()
        positions: Dict[str, Tuple[float, float, float]] = {}
        for name, cam_set in cams.items():
            if len(cam_set) < 2:
                continue
            result = gtsam.triangulateSafe(cam_set, observations[name], params)
            if result.valid():
                positions[name] = tuple(np.asarray(result.get()).tolist())
            else:
                print(f"could not triangulate aruco {name}")
        return positions

    def import_atlas(self, other: "Atlas", share_survey: bool = False, survex_prefix: str = "") -> int:
        """Move all of other's maps into this atlas, offsetting ids to avoid clashes (as op.Atlas.import_atlas).
        Other's sub-survey folders are referenced relative to this atlas's dir, with survex_prefix prepended
        to their survex prefixes. Returns the offset applied to keyframe ids"""
        kf_offset = max(self.keyframes, default=-1) + 1
        mp_offset = max(self.map_points, default=-1) + 1
        map_offset = max(self.maps, default=-1) + 1
        cam_offset = max(self.cameras, default=-1) + 1
        survey_offset = 0 if share_survey else max(self.surveys, default=-1) + 1
        self.sub_surveys = self.get_sub_surveys()
        for sub_survey in other.get_sub_surveys():
            path = Path(os.path.relpath(other.dir / sub_survey.path, self.dir)).as_posix()
            prefix = ".".join(x for x in (survex_prefix, sub_survey.survex_prefix) if x)
            offsets = {kf_id + kf_offset: offset for kf_id, offset in sub_survey.kf_offsets.items()}
            self.sub_surveys.append(SubSurvey(path, prefix, offsets, sub_survey.kf_id_offset + kf_offset))
        for mp in other.map_points.values():
            mp.id += mp_offset
            self.map_points[mp.id] = mp
        for cam_id, cam in other.cameras.items():
            self.cameras[cam_id + cam_offset] = cam
        for survey_id, survey in other.surveys.items():
            self.surveys[survey_id + survey_offset] = survey
        for mapp in other.maps.values():
            mapp.id += map_offset
            for kf in mapp.keyframes.values():
                kf.id += kf_offset
                kf._map_points = [(mp_id + mp_offset if mp_id >= 0 else mp_id) for mp_id in kf._map_points]
                kf._camera += cam_offset
            mapp.keyframes = {kf.id: kf for kf in mapp.keyframes.values()}
            self.maps[mapp.id] = mapp
            mapp.set_atlas(self)
            self.keyframes.update(mapp.keyframes)
        for mp in self.map_points.values():
            mp._atlas = self
        other.maps = {}
        return kf_offset

    def get_biggest_map(self) -> Map:
        return max(self.maps.values(), key=lambda x: len(x.keyframes))

    def create_graph(self, include_survey=False, biggest_only=False) -> Tuple[gtsam.NonlinearFactorGraph, gtsam.Values]:
        G = gtsam.NonlinearFactorGraph()
        values = gtsam.Values()
        print("adding factors")
        map_list = [self.get_biggest_map()] if biggest_only else self.maps.values()
        for m in map_list:
            m.add_to_graph(G, values, include_survey)
        return G, values

    def update_from_values(self, values: gtsam.Values) -> None:
        for mapp in self.maps.values():
            mapp.update_values(values)
        for key in values.keys():
            symbol = gtsam.Symbol(key)
            if symbol.string()[0] in "as": #it is a station
                ident = get_station_string(symbol)
                print("adding station: " + ident)
                ### FIXME maybe do the dataset index better???
                survey = self.surveys.setdefault(0, Survey(stations={}, legs=[]))
                survey.stations[ident] = SurveyStation(ident, tuple(values.atPoint3(key).tolist()))

    def align_to_imu(self) -> Dict[int, sst.Rotation]:
        """Rotate each map as a whole so its keyframe orientations match the IMU data as closely as
        possible. Returns the rotation applied to each map, by map id"""
        return {map_id: mapp.align_to_imu() for map_id, mapp in self.maps.items()}

    def optimize(self, include_survey: bool, iters: int = 100):
        G, values = self.create_graph(include_survey)
        result = run_optimisation(G, values, iters)
        self.update_from_values(result)

    def set_kf_descriptor_views(self):
        for kf_id, kf in self.keyframes.items():
            offset = self.kf_offsets[kf_id]
            kf.descriptors = self.descriptors[offset:offset + len(kf.key_points)]

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

def get_station_string(symbol: gtsam.Symbol) -> str:
    if symbol.string().startswith("s"):
        return str(symbol.index())
    elif symbol.string().startswith("a"):
        return chr(ord("A") + symbol.index())


if __name__ == "__main__":
    atlas = Atlas.from_dir(Path("."))
    print(atlas.get_aruco_positions())


def make_path(root: Path, name: str ) -> Path:
    if name.startswith("/"):
        return Path(name)
    else:
        return root / name

