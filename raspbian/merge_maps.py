#!/usr/bin/env python3
import json
from collections import defaultdict, Counter
from enum import Enum
from pathlib import Path

import gtsam
import networkx as nx

import cv2
import numpy as np
from scipy.spatial import transform as sst
from scipy.cluster.hierarchy import linkage, fcluster
from itertools import combinations, count
from typing import List, Tuple, Any, Sequence, Dict, Union, Optional, Set
from dataclasses import dataclass, field
from dataclasses_json import DataClassJsonMixin
from dataclasses_json.cfg import config as dj_conf, Exclude
import dataclasses_json.cfg

from atlas import Atlas, Map, Keyframe, SubAtlas, M, T

CONNECTIONS_FILE = "connections.json"

MERGE_NOISE_BASE = gtsam.noiseModel.Isotropic.Sigma(3, 0.1)
MERGE_NOISE = gtsam.noiseModel.Robust.Create(gtsam.noiseModel.mEstimator.Huber.Create(1.345), MERGE_NOISE_BASE)

_matcher = cv2.BFMatcher.create(normType=cv2.NORM_HAMMING)

def pose3_encode(pose: gtsam.Pose3):
    return pose.matrix().tolist()

def pose3_decode(data: List[List[float]]):
    matrix = np.array(data)
    assert matrix.shape == (4,4)
    return gtsam.Pose3(matrix)

dataclasses_json.cfg.global_config.encoders[gtsam.Pose3] = pose3_encode
dataclasses_json.cfg.global_config.encoders[Atlas] = lambda x: None
dataclasses_json.cfg.global_config.decoders[gtsam.Pose3] = pose3_decode


class KeyframeMatchData:
    """Local copy of the descriptors of a keyframe that have an associated map point,
    following the mp_descs/just_mps fields of atlas_tools.KeyFrameData"""
    def __init__(self, kf: Keyframe):
        self.id = kf.id
        has_mp = np.array([mp is not None for mp in kf.map_points], dtype=bool)
        self.mp_descs = np.array(kf.descriptors[has_mp]) # copy out of the atlas memmap
        self.just_mps = [mp.id for mp in kf.map_points if mp is not None]


class Link(Enum):
    START = 1
    END = 2
    BOTH = 3
    ALL = 4

    def slice(self, seq: List, size=50):
        if self == Link.START:
            return seq[:size]
        elif self == Link.END:
            return seq[-size:]
        elif self == Link.BOTH:
            return seq[:size//2] + seq[-size//2:]
        elif self == Link.ALL:
            return seq[:]
        else:
            raise ValueError("Unknown Link type")

    @classmethod
    def from_string(cls, text: str):
        text = text.upper()
        if len(text) == 1:
            expansion = {x[0]:x for x in cls.__members__}
            return cls[expansion[text[0]]]
        else:
            return cls[text]


@dataclass
class Connection(DataClassJsonMixin):
    from_map: int
    to_map: int
    keypoint_matches: List[Tuple[int,int]] = field(default_factory=list)
    transform: gtsam.Pose3 = gtsam.Pose3()
    transform_index: int = field(default_factory=count().__next__,
                                 init=False,
                                 metadata=dj_conf(exclude=lambda x: True))

    def compare_kfs(self, kfs_a: List[KeyframeMatchData], kfs_b: List[KeyframeMatchData]):
        links: Dict[int, List[int]] = defaultdict(list)
        for kf_a in kfs_a:
            for kf_b in kfs_b:
                matches = _matcher.knnMatch(kf_a.mp_descs, kf_b.mp_descs, 2)
                good: List[cv2.DMatch] = [m for m, n in matches if m.distance < 0.7 * n.distance]
                if len(good) > 6:
                    for x in good:
                        mp_a = kf_a.just_mps[x.queryIdx]
                        mp_b = kf_b.just_mps[x.trainIdx]
                        links[mp_a].append(mp_b)
                    print(f"{kf_a.id} -> {kf_b.id}: {len(good)} matches")
        for orig, dests in links.items():
            counts = defaultdict(int)
            counts = Counter(dests)
            histogram: List[Tuple[int, int]] = sorted([(y, x) for x, y in counts.items()], reverse=True)
            if histogram[0][0] >= 3: # only look at mappoints that match in at least 3 keyframes
                if len(histogram) == 1:
                    self.keypoint_matches.append((orig, histogram[0][1])) #if only one candidate, auto-approve
                else:
                    if histogram[0][1] >= (3 * histogram[1][0]): #otherwise needs to be much more popular than second
                        # best
                        self.keypoint_matches.append((orig, histogram[0][1]))

    def compare_maps(self, map_a: Map, map_b: Map, link_a: Link = Link.ALL, link_b: Link = Link.ALL):
        print(f"Looking for connection between Maps {map_a.id} and {map_b.id}")
        kfs_a = [KeyframeMatchData(map_a.keyframes[i]) for i in link_a.slice(sorted(map_a.keyframes))]
        kfs_b = [KeyframeMatchData(map_b.keyframes[i]) for i in link_b.slice(sorted(map_b.keyframes))]
        self.compare_kfs(kfs_a, kfs_b)

    def ransac_horn_analysis(self, atlas_a: Atlas, atlas_b: Optional[Atlas] = None,
                             translation_only = False,
                             threshold =0.5, iters = 100):
        if atlas_b is None:
            atlas_b = atlas_a
        pts_a = np.array([atlas_a.map_points[x[0]].pos for x in self.keypoint_matches])
        pts_b = np.array([atlas_b.map_points[x[1]].pos for x in self.keypoint_matches])
        R, t, inliers = ransac_horn(pts_a, pts_b, threshold, iters)
        print(self.from_map, self.to_map, R, t, inliers)
        good_matches = [x for x, inlier in zip(self.keypoint_matches, inliers) if inlier]
        self.keypoint_matches = good_matches
        if translation_only:
            pts_a = np.array([atlas_a.map_points[x[0]].pos for x in self.keypoint_matches])
            pts_b = np.array([atlas_b.map_points[x[1]].pos for x in self.keypoint_matches])
            self.transform = gtsam.Pose3(gtsam.Rot3(), np.mean(pts_b-pts_a, axis=0))
            print(self.transform)
        else:
            self.transform = gtsam.Pose3(gtsam.Rot3(R),t)

    def split(self, atlas_a: Atlas, atlas_b: Optional[Atlas] = None,
              distance: float = 5.0) -> List["Connection"]:
        """Split this connection into spatially separate clusters of matched points.

        Matches are single-linkage clustered, so two matches share a group if they are chained
        together by gaps of less than `distance` (in map units) - in both map A and map B.
        Returns one new Connection per group (largest first), each with its own transform
        index and a copy of the current transform. If the points form a single group,
        returns [self] unchanged."""
        if atlas_b is None:
            atlas_b = atlas_a
        if len(self.keypoint_matches) < 2:
            return [self]
        pts_a = np.array([atlas_a.map_points[x[0]].pos for x in self.keypoint_matches])
        pts_b = np.array([atlas_b.map_points[x[1]].pos for x in self.keypoint_matches])
        labels_a = fcluster(linkage(pts_a, method="single"), distance, criterion="distance")
        labels_b = fcluster(linkage(pts_b, method="single"), distance, criterion="distance")
        groups: Dict[Tuple[int, int], List[Tuple[int, int]]] = defaultdict(list)
        for match, la, lb in zip(self.keypoint_matches, labels_a, labels_b):
            groups[(la, lb)].append(match)
        if len(groups) < 2:
            return [self]
        return [Connection(self.from_map, self.to_map, matches, self.transform)
                for matches in sorted(groups.values(), key=len, reverse=True)]

    def add_graph_connections(self, G: gtsam.NonlinearFactorGraph, values: gtsam.Values):
        offset = np.zeros(3)
        for src_idx, dest_idx in self.keypoint_matches:
            G.add(gtsam.ReferenceFrameFactorPoint3Pose3(M(src_idx), T(self.transform_index), M(dest_idx), MERGE_NOISE))
            offset += values.atPoint3(M(dest_idx))
            offset -= values.atPoint3(M(src_idx))
        values.insert(T(self.transform_index), gtsam.Pose3(gtsam.Rot3(), offset / len(self.keypoint_matches)))

    def update_transform_from_values(self, values: gtsam.Values):
        self.transform = values.atPose3(T(self.transform_index))

    def is_valid(self):
        return len(self.keypoint_matches) > 6

@dataclass
class InternalConnections(DataClassJsonMixin):
    connections: List[Connection] = field(default_factory=list)
    atlas: Optional[Atlas] = field(metadata=dj_conf(exclude=lambda x: True), default=None)

    def find_connections(self):
        for m0, m1 in combinations(self.atlas.maps.values(), 2):
            connection = Connection(m0.id, m1.id)
            connection.compare_maps(m0, m1)
            if connection.is_valid():
                self.connections.append(connection)

    def split_connections(self, distance: float = 5.0):
        new_connections: List[Connection] = []
        for conn in self.connections:
            new_connections.extend(conn.split(self.atlas, distance=distance))
        self.connections = new_connections

    def get_ransac_transforms(self, translation_only=True):
        if self.atlas is None:
            raise ValueError("No atlas provided")
        for conn in self.connections:
            conn.ransac_horn_analysis(self.atlas, translation_only=translation_only)
        self.clean_connections()

    def apply_transform(self):
        if self.atlas is None:
            raise ValueError("No atlas provided")
        map_graph, relations = self.get_transform_graph()
        # place each map via the strongest chain of connections, not just the fewest hops
        tree = nx.maximum_spanning_tree(map_graph, weight="matches")
        transforms: Dict[int, gtsam.Pose3] = {}
        for i, subnet in enumerate(nx.connected_components(tree)):
            m0 = min(subnet)
            offset = np.array([i*20,0,0]) # offset subgraphs by 20m
            transforms[m0] = gtsam.Pose3(gtsam.Rot3(),offset)
            for source, dest in nx.bfs_edges(tree, source=m0):
                if (source, dest) in relations:
                    tr = relations[(source, dest)]
                else:
                    tr = relations[(dest, source)].inverse()
                print(f"Map connection: {source} -> {dest}: {tr}")
                transforms[dest] = tr.transformPoseFrom(transforms[source])
        for mapp, t in transforms.items():
            print(mapp, t)
            apply_map_transform(self.atlas, self.atlas.maps[mapp], t)

    def get_transform_graph(self) -> tuple[nx.Graph, dict[tuple[int, int], gtsam.Pose3]]:
        map_graph = nx.Graph()
        relations = {(x.from_map, x.to_map): x.transform for x in self.connections}
        map_graph.add_edges_from((x.from_map, x.to_map, {"matches": len(x.keypoint_matches)}) for x in self.connections)
        return map_graph, relations

    def clean_connections(self):
        self.connections = [x for x in self.connections if x.is_valid()]

    def unify_maps(self):
        mp_links = [y for x in self.connections for y in x.keypoint_matches]
        G = nx.Graph()
        G.add_edges_from(mp_links)
        connections = {}
        for group in nx.connected_components(G):
            dest = min(group)
            for x in list(sorted(group))[1:]:
                connections[x] = dest
        map_graph, relations = self.get_transform_graph()
        for subnet in nx.connected_components(map_graph):
            min_map_id = min(subnet)
            for map_id in subnet:
                if map_id != min_map_id:
                    print("merging map", map_id, "into", min_map_id)
                    self.atlas.maps[min_map_id].keyframes.update(self.atlas.maps.pop(map_id).keyframes)
        for kf in self.atlas.keyframes.values():
            kf._map_points = [connections.get(mp_id, mp_id) for mp_id in kf._map_points]
        for old in connections:
            self.atlas.map_points.pop(old, None)
        for mapp in self.atlas.maps.values():
            mapp.set_atlas(self.atlas)

    def get_connected_points(self):
        result = []
        for conn in self.connections:
            result.extend(conn.keypoint_matches)
        return result

def map_point_ids(mapp: Map) -> Set[int]:
    return {mp.id for kf in mapp.keyframes.values() for mp in kf.map_points if mp is not None}

def apply_map_transform(atlas: Atlas, mapp: Map, transform: gtsam.Pose3) -> None:
    # moves everything in the map by the inverse of transform, as atlas_tools.Map.apply_transform did
    inv = sst.RigidTransform.from_matrix(transform.inverse().matrix())
    for kf in mapp.keyframes.values():
        kf.pose = inv * kf.pose
    for mp_id in map_point_ids(mapp):
        mp = atlas.map_points[mp_id]
        mp.pos = tuple(inv.apply(np.array(mp.pos)).tolist())

def ransac_horn(pts_a, pts_b, thresh=0.05, iters=1000):
    # pts_a, pts_b: Nx3 correspondences, pts_b = R @ pts_a + t
    best_inliers = None
    n = len(pts_a)
    for _ in range(iters):
        idx = np.random.choice(n, 3, replace=False)
        R, t = horn(pts_a[idx], pts_b[idx])
        residuals = np.linalg.norm((R @ pts_a.T).T + t - pts_b, axis=1)
        inliers = residuals < thresh
        if best_inliers is None or inliers.sum() > best_inliers.sum():
            best_inliers = inliers
    R, t = horn(pts_a[best_inliers], pts_b[best_inliers])
    return R, t, best_inliers

def horn(A, B):
    ca, cb = A.mean(0), B.mean(0)
    H = (A - ca).T @ (B - cb)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1, 1, d]) @ U.T
    t = cb - R @ ca
    return R, t

def merge_maps(atlas: Atlas, dir: Path, survey_3d: Optional[Path] = None,
               survey_prefixes: Optional[List[str]] = None, bundle_adjust: bool = True):
    """Merge the maps of the atlas into one. If survey_3d is given the atlas is also morphed to fit
    the survey, as combine.py does (each map counts as a sub-atlas). The morph runs even with no survey,
    to close up any gap where maps that form a loop join"""
    if len(atlas.maps) == 0:
        raise ValueError("No maps found")
    if len(atlas.maps) == 1:
        print("Only one map found, skipping merge step")
        connections = InternalConnections(atlas=atlas)
    else:
        print("Multiple maps: trying to merge")
        print("finding connections")
        if (dir / CONNECTIONS_FILE).exists():
            with open(dir / CONNECTIONS_FILE) as f:
                connections = InternalConnections.from_json(f.read())
                connections.atlas = atlas
                connections.get_ransac_transforms(translation_only=True)
        else:
            connections = InternalConnections(atlas=atlas)
            connections.find_connections()
            connections.get_ransac_transforms(translation_only=True)
            with open(dir / CONNECTIONS_FILE, "w") as f:
                f.write(connections.to_json())

        connections.apply_transform()
    from survey_morph import morph_to_survey, parse_prefixes, read_survey_stations, bundle_adjust as bundle_adjust_atlas # imported here as survey_morph imports this module
    # each map is a sub-atlas, so a connection between two maps is a seam that the morph closes up (which matters
    # when the maps form a loop and don't quite meet). This is done with or without a survey
    sub_atlases = {f"map{map_id}": SubAtlas(maps={map_id}) for map_id in sorted(atlas.maps)}
    prefixes = parse_prefixes(survey_prefixes or [], list(sub_atlases))
    if survey_3d is not None:
        # unless told otherwise, markers are stations under the survey's one top level prefix (e.g. garden.a)
        tops = {label.split(".")[0] for label in read_survey_stations(survey_3d) if "." in label}
        if len(tops) == 1:
            prefixes = {name: prefixes.get(name, next(iter(tops))) for name in sub_atlases}
    survey_fit = morph_to_survey(atlas, sub_atlases, connections.connections, survey_3d, prefixes)
    connections.unify_maps()
    if bundle_adjust:
        bundle_adjust_atlas(atlas, survey_fit)
    print("Merge complete")


if __name__=="__main__":
    import argparse
    parser = argparse.ArgumentParser(description="merge the maps in an atlas, saving the result as atlas.json.gz in dir")
    parser.add_argument('-a', '--atlas', help="atlas file to use (.osa or .json.gz), default is atlas.json.gz in dir")
    parser.add_argument('-d', '--dir', help="directory to use", required=True)
    parser.add_argument('-s', '--survey', help="survey .3d file to fit the atlas to, default is survey.3d in dir "
                        "(made from survey.svx with cavern if there is one)")
    parser.add_argument('--no-survey', action='store_true', help="ignore any survey")
    parser.add_argument('--survey-prefix', action='append', default=[],
                        help="survex prefix the aruco markers are under, e.g. 'garden' for station garden.a. "
                        "Default is the survey's top level prefix")
    parser.add_argument('--no-bundle', action='store_true',
                        help="with a survey, skip the final bundle adjustment")
    opts = parser.parse_args()

    model_dir = Path(opts.dir)
    atlas_file = Path(opts.atlas) if opts.atlas else model_dir / "atlas.json.gz"
    if atlas_file.suffix == ".osa":
        atlas = Atlas.from_osa(atlas_file)
    else:
        atlas = Atlas.from_json_file(atlas_file)
    atlas.align_to_imu()
    if opts.no_survey:
        survey_3d = None
    elif opts.survey:
        survey_3d = Path(opts.survey)
    else:
        from survey_data import ensure_survey_3d
        survey_3d = ensure_survey_3d(model_dir)
    merge_maps(atlas, model_dir, survey_3d, opts.survey_prefix, not opts.no_bundle)
    atlas.merged = True
    atlas.save(model_dir / "atlas.json.gz")
