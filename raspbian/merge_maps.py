#!/usr/bin/env python3
import json
from collections import defaultdict

import gtsam
import networkx as nx

import orb_slam3_py as op
import cv2
import numpy as np
import itertools
from typing import List, Tuple, Any, Sequence, Dict, Union

from atlas_tools import Map, KeyFrameData, M, T, Atlas, run_optimisation

MERGE_NOISE_BASE = gtsam.noiseModel.Isotropic.Sigma(3, 0.1)
MERGE_NOISE = gtsam.noiseModel.Robust.Create(gtsam.noiseModel.mEstimator.Huber.Create(1.345), MERGE_NOISE_BASE)

#This is dict that stores a list of similar mappoints between pairs of maps
ConnectionType = Dict[Tuple[Map, Map], List[Tuple[int,int]]]
RelationType = Dict[Tuple[Map, Map], gtsam.Pose3]

_matcher = cv2.BFMatcher.create(normType=cv2.NORM_HAMMING)

def compare_all_kfs(m0: Map, m1: Map) -> List[Tuple[KeyFrameData, KeyFrameData, Any]]:
    results: List[Tuple[KeyFrameData, KeyFrameData, Any]] = []
    for kf1 in m0.kfs:
        # print(kf1.id)
        for kf2 in m1.kfs:
            matches = _matcher.knnMatch(kf1.mp_descs, kf2.mp_descs, 2)
            good: List[cv2.DMatch] = [m for m, n in matches if m.distance < 0.7 * n.distance]
            if len(good) > 6:
                links: List[Tuple[op.MapPoint, op.MapPoint]] = []
                for x in good:
                    links.append((kf1.just_mps[x.queryIdx].id, kf2.just_mps[x.trainIdx].id))
                results.append((kf1, kf2, links))
                print(f"{kf1.id} -> {kf2.id}: {len(good)} matches")
    return results

def find_good_matches(matches: List[Tuple[KeyFrameData, KeyFrameData, Any]]) -> List[Tuple[int,int]]:
    mp_matches = defaultdict(list)
    #find candidates for each mappoint in m0
    for kf1, kf2, links in matches:
        for mp1, mp2 in links:
            mp_matches[mp1].append(mp2)
    connections = []
    for orig, dests in mp_matches.items():
        counts = defaultdict(int)
        for dest in dests:
            counts[dest] += 1
        # this creates a histogram of the candidates, with msot frequent first
        histogram: List[Tuple[op.MapPoint, int]] = sorted(list(counts.items()), key=lambda x: x[1],
                                                          reverse=True)
        if histogram[0][1] > 6: # only look at mappoints that match in more than 6 keyframes
            if len(histogram) == 1:
                connections.append((orig, histogram[0][0])) #if only one candidate, auto-approve
            else:
                if histogram[0][1] > (3 * histogram[1][1]): #otherwise needs to be much more popular than second best
                    connections.append((orig, histogram[0][0]))
    return connections

def find_connections(maps: Dict[int,Map]) -> ConnectionType:
    connections: ConnectionType = defaultdict(list)
    for m0,m1 in itertools.combinations(maps.values(), 2):
        results = compare_all_kfs(m0, m1)
        matches = find_good_matches(results)
        if len(matches) >= 6: #only report maps that are well-connected
            connections[(m0,m1)] = matches
    return connections

def ransac_horn_analysis(connections: ConnectionType, threshold =0.5, iters = 100):
    new_connections = {}
    relations = {}
    for (m0, m1), matches in connections.items():
        pts_a = np.array([m0.mps[x[0]].get_world_pos() for x in matches])
        pts_b = np.array([m1.mps[x[1]].get_world_pos() for x in matches])
        R, t, inliers = ransac_horn(pts_a, pts_b, threshold, iters)
        print(m0.id, m1.id, R, t, inliers)
        good_matches = [x for x, inlier in zip(matches, inliers) if inlier]
        if len(good_matches) >= 6:
            new_connections[(m0,m1)] = good_matches
            relations[(m0,m1)] = gtsam.Pose3(gtsam.Rot3(R),t)
    return new_connections, relations

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

def add_connections_to_graph(G: gtsam.NonlinearFactorGraph, values: gtsam.Values, connections: ConnectionType):
    for i, (src_map, dest_map) in enumerate(sorted_connection_keys(connections)):
        offset = np.zeros(3)
        for src_idx, dest_idx in connections[(src_map, dest_map)]:
            G.add(gtsam.ReferenceFrameFactorPoint3Pose3(M(src_idx), T(i), M(dest_idx), MERGE_NOISE))
            offset += values.atPoint3(M(dest_idx))
            offset -= values.atPoint3(M(src_idx))
        values.insert(T(i), gtsam.Pose3(gtsam.Rot3(), offset / len(connections[(src_map, dest_map)])))
        print(f"rough pose at {i}")
        print(values.atPose3(T(i)))

def sorted_connection_keys(connections: Union[ConnectionType, RelationType]):
    return sorted(connections.keys(), key=lambda x: (x[0].id, x[1].id))

def get_transform_from_vals(connections: ConnectionType, values: gtsam.Values) -> RelationType:
    sorted_map_pairs = sorted_connection_keys(connections)
    map_indexes = {k: i for i, k in enumerate(sorted_map_pairs)}
    relations = {maps: values.atPose3(T(map_indexes[maps])) for maps in sorted_map_pairs}
    return relations

def transform_maps(relations: RelationType):
    map_graph = nx.Graph()
    sorted_map_pairs = sorted_connection_keys(relations)
    map_graph.add_edges_from(sorted_map_pairs)
    m0 = sorted_map_pairs[0][0]
    transforms: Dict[Map, gtsam.Pose3] = {m0: gtsam.Pose3()}
    for source, dest in nx.bfs_edges(map_graph, source=m0):
        if (source, dest) in relations:
            tr = relations[(source, dest)]
        else:
            tr = relations[(dest, source)].inverse()
        print(f"Map connection: {source.id} -> {dest.id}: {tr}")
        transforms[dest] = tr.transformPoseFrom(transforms[source])
    for m, tr in transforms.items():
        print(m.id, tr)
    for mapp, t in transforms.items():
        mapp.apply_transform(t)

def unify_maps(atlas: Atlas, connections: ConnectionType):
    links = [y for x in connections for y in connections[x]]
    G = nx.Graph()
    G.add_edges_from(links)
    connections = {}
    for group in nx.connected_components(G):
        dest = min(group)
        for x in list(sorted(group))[1:]:
            connections[x] = dest
    min_map_id = min(atlas.maps.keys())
    for map_id, map in atlas.maps.items():
        if map_id != min_map_id:
            atlas.maps[min_map_id].merge_in(map, connections)
            atlas.atlas.erase_map(map.map)

def merge_maps(atlas: Atlas):
    if len(atlas.maps) == 0:
        raise ValueError("No maps found")
    if len(atlas.maps) == 1:
        print("Only one map found, skipping merge step")
        return # only one map so no merge needed
    print("Multiple maps: trying to merge")
    print("finding connections")
    connections = find_connections(atlas.maps)
    for (m0,m1), matches in connections.items():
        print(f"{m0.id} -> {m1.id}: {len(matches)} matches")
    connections, relations = ransac_horn_analysis(connections)
    mps = {x.id: x for x in atlas.atlas.get_all_map_points()}
    for a,b in list(connections.values())[1]:
        print(f"{a} -> {b}: {mps[a].get_world_pos()} -> {mps[b].get_world_pos()}")
    print("After ransac")
    for (m0,m1), matches in connections.items():
        print(f"{m0.id} -> {m1.id}: {len(matches)} matches")
    #relations = get_relations_from_optimisation(atlas, connections, relations)
    print("transforming maps")
    transform_maps(relations)
    connected = []
    for matches in connections.values():
        for x,y in matches:
            connected.extend([x,y])
    with open("connections.json", "w") as f:
        json.dump(connected, f)
    print("unifying maps")
    unify_maps(atlas, connections)
    atlas.reload()
    print("Merge complete")


def get_relations_from_optimisation(atlas: Atlas, connections: dict[Any, Any], relations: dict[tuple[Map, Map], Any]) -> \
dict[tuple[Map, Map], Any]:
    print("Adding connections")
    G, values = atlas.create_graph(include_survey=False)
    add_connections_to_graph(G, values, connections)
    print("running optimisation")
    results = run_optimisation(G, values)
    relations = get_transform_from_vals(connections, results)
    for m in atlas.maps:
        m.update_values(results)
    return relations
