#!/usr/bin/env python3
import json
from collections import defaultdict, Counter
from enum import Enum
from pathlib import Path

import gtsam
import networkx as nx
from gtsam import Pose3
from networkx import Graph

import orb_slam3_py as op
import cv2
import numpy as np
from itertools import combinations, count
from typing import List, Tuple, Any, Sequence, Dict, Union, Optional
from dataclasses import dataclass, field
from dataclasses_json import DataClassJsonMixin
from dataclasses_json.cfg import config as dj_conf, Exclude
import dataclasses_json.cfg

from atlas_tools import Map, KeyFrameData, M, T, Atlas, run_optimisation

CONNECTIONS_FILE = "connections.json"

MERGE_NOISE_BASE = gtsam.noiseModel.Isotropic.Sigma(3, 0.1)
MERGE_NOISE = gtsam.noiseModel.Robust.Create(gtsam.noiseModel.mEstimator.Huber.Create(1.345), MERGE_NOISE_BASE)

#This is dict that stores a list of similar mappoints between pairs of maps
ConnectionType = Dict[Tuple[Map, Map], List[Tuple[int,int]]]
RelationType = Dict[Tuple[Map, Map], gtsam.Pose3]

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

    def compare_kfs(self, kfs_a, kfs_b):
        links: Dict[int, List[int]] = defaultdict(list)
        for kf_a in kfs_a:
            for kf_b in kfs_b:
                matches = _matcher.knnMatch(kf_a.mp_descs, kf_b.mp_descs, 2)
                good: List[cv2.DMatch] = [m for m, n in matches if m.distance < 0.7 * n.distance]
                if len(good) > 6:
                    for x in good:
                        mp_a = kf_a.just_mps[x.queryIdx].id
                        mp_b = kf_b.just_mps[x.trainIdx].id
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
        self.compare_kfs(link_a.slice(map_a.kfs), link_b.slice(map_b.kfs))

    def ransac_horn_analysis(self, atlas_a: Atlas, atlas_b: Optional[Atlas] = None,
                             translation_only = False,
                             threshold =0.5, iters = 100):
        if atlas_b is None:
            atlas_b = atlas_a
        pts_a = np.array([atlas_a.mps[x[0]].get_world_pos() for x in self.keypoint_matches])
        pts_b = np.array([atlas_b.mps[x[1]].get_world_pos() for x in self.keypoint_matches])
        R, t, inliers = ransac_horn(pts_a, pts_b, threshold, iters)
        print(self.from_map, self.to_map, R, t, inliers)
        good_matches = [x for x, inlier in zip(self.keypoint_matches, inliers) if inlier]
        self.keypoint_matches = good_matches
        if translation_only:
            pts_a = np.array([atlas_a.mps[x[0]].get_world_pos() for x in self.keypoint_matches])
            pts_b = np.array([atlas_b.mps[x[1]].get_world_pos() for x in self.keypoint_matches])
            self.transform = gtsam.Pose3(gtsam.Rot3(), np.mean(pts_b-pts_a, axis=0))
            print(self.transform)
        else:
            self.transform = gtsam.Pose3(gtsam.Rot3(R),t)

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

    def get_optimized_transforms(self):
        if self.atlas is None:
            raise ValueError("No atlas provided")
        G, values = self.atlas.create_graph()
        for conn in self.connections:
            conn.add_graph_connections(G, values)
        results = run_optimisation(G, values)
        for m in self.atlas.maps.values():
            m.update_values(results)

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
        transforms: Dict[Map, gtsam.Pose3] = {}
        for i, subnet in enumerate(nx.connected_components(map_graph)):
            m0 = min(subnet)
            offset = np.array([i*20,0,0]) # offset subgraphs by 20m
            transforms[m0] = gtsam.Pose3(gtsam.Rot3(),offset)
            for source, dest in nx.bfs_edges(map_graph, source=m0):
                if (source, dest) in relations:
                    tr = relations[(source, dest)]
                else:
                    tr = relations[(dest, source)].inverse()
                print(f"Map connection: {source} -> {dest}: {tr}")
                transforms[dest] = tr.transformPoseFrom(transforms[source])
        for mapp, t in transforms.items():
            print(mapp, t)
            self.atlas.maps[mapp].apply_transform(t)

    def get_transform_graph(self) -> tuple[nx.Graph, dict[tuple[int, int], Pose3]]:
        map_graph = nx.Graph()
        relations = {(x.from_map, x.to_map): x.transform for x in self.connections}
        map_graph.add_edges_from(relations.keys())
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
                    mapp = self.atlas.maps[map_id]
                    self.atlas.maps[min_map_id].merge_in(mapp, connections)
                    self.atlas.atlas.erase_map(mapp.map)

    def get_connected_points(self):
        result = []
        for conn in self.connections:
            result.extend(conn.keypoint_matches)
        return result

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

def merge_maps(atlas: Atlas, dir: Path):
    if len(atlas.maps) == 0:
        raise ValueError("No maps found")
    if len(atlas.maps) == 1:
        print("Only one map found, skipping merge step")
        return # only one map so no merge needed
    print("Multiple maps: trying to merge")
    print("finding connections")
    if (dir / CONNECTIONS_FILE).exists():
        with open(dir / CONNECTIONS_FILE) as f:
            connections = InternalConnections.from_json(f.read())
            connections.atlas = atlas
            connections.get_ransac_transforms()
    else:
        connections = InternalConnections(atlas=atlas)
        connections.find_connections()
        connections.get_ransac_transforms()
        with open(dir / CONNECTIONS_FILE, "w") as f:
            f.write(connections.to_json())

    connections.apply_transform()
    connections.unify_maps()
    atlas.reload()
    #atlas.align()

# if __name__ == "__main__":
#     atlas = Atlas.from_file("atlas.osa")
#     for i in range(3):
#         connection = Connection(i, 3)
#         connection.compare_maps(atlas.maps[i], atlas.maps[3], link_b=Link.START)
#         if connection.keypoint_matches:
#             print(connection)
#             connection.ransac_horn_analysis(atlas)
#             print(connection)
#             print(connection.to_json())
