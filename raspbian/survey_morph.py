#!/usr/bin/env python3
"""Morph the sub-atlases of a combined atlas so they agree with a survex survey.

The aruco markers seen in each sub-atlas are matched to stations in a survex .3d file and held fixed
at their surveyed positions. Each sub-atlas gets a tree (acyclic graph) whose nodes are its aruco
markers and its connections to other maps; the trees of the different sub-atlases join up at the
connection nodes, which belong to both sides of the connection. The graph is solved with gtsam, and
the movement of the nodes is then spread over every map point and keyframe by interpolating along the
nearest tree edge, so the maps bend to follow the line of the cave rather than being moved rigidly.
"""
import re
import subprocess
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

import gtsam
import networkx as nx
import numpy as np
import scipy.spatial.transform as sst

from atlas import Atlas, SubAtlas, run_optimisation, K, M, R as RIGHT, SMART_NOISE, ANCHOR_NOISE
from scipy.optimize import least_squares

from merge_maps import Connection

EDGE_NOISE_BASE = 0.05 # metres of slack on every edge
EDGE_NOISE_PER_METRE = 0.02 # extra slack, as a fraction of the edge length
MARKER_SIGMA = 0.1 # metres: how well an aruco position triangulated from the atlas matches the survey
CONNECTION_SIGMA = 0.2 # metres: how well the two sides of a connection meet, per connection
REGISTRATION_ROTATION_PRIOR = 1.0 # radians
REGISTRATION_SHIFT_PRIOR = 10.0 # metres
REGISTRATION_LOG_SCALE_PRIOR = 0.5 # a scale of 0.6 to 1.65 is within one sigma
ARUCO_PRIOR_NOISE = 0.01 # metres: how firmly bundle adjustment holds aruco markers on their stations
SEAM_NOISE = 0.01 # metres: how closely the two sides of a connection must meet
WEAK_PRIOR_NOISE = 5.0 # metres: stops free-floating parts of the graph being under-constrained
UNREACHABLE_HOPS = 1e6 # tree edge cost between nodes with no chain of shared map points between them
DISTANCE_TIEBREAK = 1e-3 # per metre, so equal keyframe counts prefer the physically closer node
NODE_RE = re.compile(r"^NODE\s+(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)\s+\[([^\]]*)\]")


def read_survey_stations(path: Optional[Path]) -> Dict[str, np.ndarray]:
    if path is None:
        return {}
    """Station positions by full survex label (e.g. "storrs.A"), read from a .3d file using survex's dump3d"""
    out = subprocess.run(["dump3d", str(path)], capture_output=True, text=True, check=True).stdout
    return parse_dump3d(out)


def parse_dump3d(text: str) -> Dict[str, np.ndarray]:
    stations: Dict[str, np.ndarray] = {}
    for line in text.splitlines():
        m = NODE_RE.match(line)
        if m:
            stations[m.group(4)] = np.array([float(m.group(i)) for i in (1, 2, 3)])
    return stations


def find_station_label(stations: Dict[str, np.ndarray], prefix: str, name: str) -> Optional[str]:
    """Look up an aruco marker as a station: prefix.name, allowing for further prefixes above it.
    Survex labels are case insensitive (dump3d gives them in lower case), so this is too"""
    wanted = (f"{prefix}.{name}" if prefix else name).lower()
    for label in stations:
        if label.lower() == wanted or label.lower().endswith("." + wanted):
            return label
    return None


def find_station(stations: Dict[str, np.ndarray], prefix: str, name: str) -> Optional[np.ndarray]:
    label = find_station_label(stations, prefix, name)
    return None if label is None else stations[label]


@dataclass
class Node:
    key: int
    pos: np.ndarray
    label: str
    maps: Set[int] = field(default_factory=set) # maps this node touches
    kfs: Set[int] = field(default_factory=set) # keyframes that see this node (its aruco marker or matched points)
    fixed: Optional[np.ndarray] = None # surveyed position, for aruco markers
    new_pos: Optional[np.ndarray] = None

    @property
    def displacement(self) -> np.ndarray:
        return self.new_pos - self.pos


def yaw_fit(atlas_pts: np.ndarray, survey_pts: np.ndarray) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """Best rotation about the vertical (z) axis plus translation taking atlas_pts to survey_pts.
    The atlas has already been levelled using the IMU, so unlike a full 3D fit this works with as few as
    two markers, or markers in a line. Returns None if the markers are directly above each other,
    which gives no heading"""
    a = atlas_pts[:, :2] - atlas_pts[:, :2].mean(axis=0)
    b = survey_pts[:, :2] - survey_pts[:, :2].mean(axis=0)
    if np.linalg.norm(a, axis=1).max() < 1e-3 or np.linalg.norm(b, axis=1).max() < 1e-3:
        return None
    theta = np.arctan2(np.sum(a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]), np.sum(a[:, 0] * b[:, 0] + a[:, 1] * b[:, 1]))
    R = sst.Rotation.from_euler("z", theta).as_matrix()
    return R, survey_pts.mean(axis=0) - R @ atlas_pts.mean(axis=0)


Affine = Tuple[np.ndarray, np.ndarray] # matrix (a rotation times a scale), translation: p -> A @ p + t


def apply_affine(atlas: Atlas, sub: SubAtlas, A: np.ndarray, t: np.ndarray) -> None:
    """Move and scale a whole sub-atlas (keyframes, their stereo baselines and map points)"""
    scale = abs(np.linalg.det(A)) ** (1 / 3)
    rotation = sst.Rotation.from_matrix(A / scale)
    for map_id in sub.maps:
        for kf in atlas.maps[map_id].keyframes.values():
            kf.pose = sst.RigidTransform.from_components(A @ kf.pose.translation + t, rotation * kf.pose.rotation)
            # the stereo baseline is a distance in the same units as the map, so it scales with it, and the IMU
            # orientation is in the same world frame as the poses so it turns with them
            kf.baseline *= scale
            kf.imu_pose = sst.RigidTransform.from_components(kf.imu_pose.translation, rotation * kf.imu_pose.rotation)
        for mp_id in atlas.maps[map_id].map_point_ids():
            mp = atlas.map_points[mp_id]
            mp.pos = tuple((A @ np.array(mp.pos) + t).tolist())


def register_sub_atlases(atlas: Atlas, sub_atlases: Dict[str, SubAtlas], connections: List[Connection],
                         markers: List[Tuple[str, np.ndarray, np.ndarray]]) -> Dict[str, Affine]:
    """Move each sub-atlas (a heading, a shift and a scale: the IMU has already levelled them) so that its
    aruco markers sit on the survey and the two sides of every connection between sub-atlases meet.
    markers is a list of (sub-atlas name, position in the atlas, surveyed position).

    First each sub-atlas with 2+ markers is fitted to them alone, and the rest are fitted to their already
    placed neighbours through their connections. Then everything is refined together by robust least
    squares, so sub-atlases with few markers are held by their connections and bad matches are down-weighted.
    Returns the transform applied to each sub-atlas, which is also applied to the atlas"""
    map_sub = {map_id: name for name, sub in sub_atlases.items() for map_id in sub.maps}
    names = list(sub_atlases)
    index = {name: i for i, name in enumerate(names)}
    # connection points on each side, in the atlas before any registration
    links: List[Tuple[str, np.ndarray, str, np.ndarray]] = []
    for conn in connections:
        sa, sb = map_sub.get(conn.from_map), map_sub.get(conn.to_map)
        if sa is None or sb is None or sa == sb or not conn.keypoint_matches:
            continue
        links.append((sa, np.array([atlas.map_points[a].pos for a, _ in conn.keypoint_matches]),
                      sb, np.array([atlas.map_points[b].pos for _, b in conn.keypoint_matches])))

    # 1. initial placement
    start: Dict[str, Affine] = {}
    for name in names:
        mine = [(m[1], m[2]) for m in markers if m[0] == name]
        fit = yaw_fit(np.array([m[0] for m in mine]), np.array([m[1] for m in mine])) if len(mine) >= 2 else None
        if fit is not None:
            start[name] = fit
    # if not start:
    #     raise ValueError("No sub-atlas had 2 matched aruco markers (not directly above each other) to register with")
    progress = True
    while progress and len(start) < len(names):
        progress = False
        for name in names:
            if name in start:
                continue
            mine, target = [], []
            for sa, pa, sb, pb in links:
                for here, other in ((sa, sb), (sb, sa)):
                    if here == name and other in start:
                        mine.append(pa if here == sa else pb)
                        R, t = start[other]
                        target.append((pb if here == sa else pa) @ R.T + t)
            if mine:
                mine_pts, target_pts = np.vstack(mine), np.vstack(target)
                fit = yaw_fit(mine_pts, target_pts)
                if fit is None: # too few points to give a heading: just shift
                    fit = (np.eye(3), target_pts.mean(axis=0) - mine_pts.mean(axis=0))
                start[name] = fit
                progress = True
    for name in names:
        if name not in start:
            print(f"{name} has no aruco markers and no connections to a sub-atlas that does, leaving it as it is")
            start[name] = (np.eye(3), np.zeros(3))

    # 2. joint refinement. Each sub-atlas turns by theta and scales by exp(log_scale) about its own centre, then shifts by tau
    centres = {}
    for name in names:
        pts = [atlas.map_points[i].pos for m in sub_atlases[name].maps for i in atlas.maps[m].map_point_ids()]
        R, t = start[name]
        centres[name] = (np.mean(pts, axis=0) @ R.T + t) if pts else t

    def total(name: str, params: np.ndarray) -> Affine:
        theta, tau, scale = params[0], params[1:4], np.exp(params[4])
        R0, t0 = start[name]
        Rt = scale * sst.Rotation.from_euler("z", theta).as_matrix()
        c = centres[name]
        return Rt @ R0, c + Rt @ (t0 - c) + tau

    def residuals(x: np.ndarray) -> np.ndarray:
        affines = {name: total(name, x[5 * i:5 * i + 5]) for name, i in index.items()}
        out = []
        for name, atlas_pt, survey_pt in markers:
            R, t = affines[name]
            out.append((R @ atlas_pt + t - survey_pt) / MARKER_SIGMA)
        for sa, pa, sb, pb in links:
            (Ra, ta), (Rb, tb) = affines[sa], affines[sb]
            # each connection counts for the same however many points it has
            out.append((((pa @ Ra.T + ta) - (pb @ Rb.T + tb)) / (CONNECTION_SIGMA * np.sqrt(len(pa)))).ravel())
        params = x.reshape(-1, 5) # weak priors to stay near the start
        out.append(params[:, 0] / REGISTRATION_ROTATION_PRIOR)
        out.append(params[:, 1:4].ravel() / REGISTRATION_SHIFT_PRIOR)
        out.append(params[:, 4] / REGISTRATION_LOG_SCALE_PRIOR)
        return np.concatenate([np.ravel(o) for o in out])

    def report(label: str, x: Optional[np.ndarray]) -> None: # x=None: the atlas as it was before any registration
        affines = {name: (np.eye(3), np.zeros(3)) if x is None else total(name, x[5 * i:5 * i + 5])
                   for name, i in index.items()}
        for name in names:
            errs = [np.linalg.norm(affines[name][0] @ p + affines[name][1] - q) for n, p, q in markers if n == name]
            scale = abs(np.linalg.det(affines[name][0])) ** (1 / 3)
            print(f"  {label} {name}: scale {scale:.3f}, {len(errs)} markers" + (f", rms error {np.sqrt(np.mean(np.square(errs))):.2f}m" if errs else ""))
        for sa, pa, sb, pb in links:
            (Ra, ta), (Rb, tb) = affines[sa], affines[sb]
            gap = np.linalg.norm((pa @ Ra.T + ta) - (pb @ Rb.T + tb), axis=1)
            print(f"  {label} connection {sa}-{sb}: {len(pa)} points, median gap {np.median(gap):.2f}m")

    x0 = np.zeros(5 * len(names))
    report("unregistered", None)
    report("initial", x0)
    result = least_squares(residuals, x0, loss="huber", f_scale=1.0)
    report("after", result.x)

    final: Dict[str, Affine] = {}
    for name in names:
        final[name] = total(name, result.x[5 * index[name]:5 * index[name] + 5])
        apply_affine(atlas, sub_atlases[name], *final[name])
    return final


def build_nodes(atlas: Atlas, sub_atlases: Dict[str, SubAtlas], connections: List[Connection],
                stations: Dict[str, np.ndarray], prefixes: Dict[str, str]
                ) -> Tuple[Dict[str, List[Node]], List[Node], List[Tuple[Node, Node]]]:
    """Make the graph nodes, registering the sub-atlases to the survey (and each other) on the way.
    Markers are looked up as <prefix>.<id>, where the prefix of a sub-atlas is prefixes[name] (default: its name).
    Returns the nodes of each sub-atlas, all the nodes, and the pairs of nodes that must stay together.
    A connection between two sub-atlases has a node on each side, as the sub-atlases can't be
    registered to meet exactly. The two are tied together by the solver (a "seam")"""
    # aruco markers, per sub-atlas as marker ids are only unique within a survey
    aruco_info: List[Tuple[str, str, np.ndarray, np.ndarray, Set[int]]] = [] # sub, name, atlas pos, survey pos, keyframe ids
    for sub_name, sub in sub_atlases.items():
        kfs = [kf for map_id in sub.maps for kf in atlas.maps[map_id].keyframes.values()]
        for name, pos in atlas.get_aruco_positions(kfs).items():
            survey_pos = find_station(stations, prefixes.get(sub_name, sub_name), name)
            if survey_pos is None:
                print(f"aruco {name} in {sub_name} has no matching survey station")
                continue
            seen_in = {kf.id for kf in kfs if name in kf.arucos}
            aruco_info.append((sub_name, name, np.array(pos), survey_pos, seen_in))
    # if not aruco_info:
    #     raise ValueError("No aruco markers matched survey stations")
    transforms = register_sub_atlases(atlas, sub_atlases, connections,
                                      [(a[0], a[2], a[3]) for a in aruco_info])

    nodes: Dict[str, List[Node]] = {name: [] for name in sub_atlases}
    all_nodes: List[Node] = []
    kf_map = {kf_id: map_id for map_id, mapp in atlas.maps.items() for kf_id in mapp.keyframes}
    mp_kfs: Dict[int, Set[int]] = defaultdict(set)
    for kf in atlas.keyframes.values():
        for mp_id in kf._map_points:
            if mp_id >= 0:
                mp_kfs[mp_id].add(kf.id)
    for i, (sub_name, name, atlas_pos, survey_pos, kf_ids) in enumerate(aruco_info):
        # the position in the registered atlas, which is near the survey position but not on it
        A, t = transforms[sub_name]
        pos = A @ atlas_pos + t
        node = Node(gtsam.symbol('a', i), pos, f"{sub_name}.{name}", {kf_map[k] for k in kf_ids},
                    kf_ids, fixed=survey_pos)
        nodes[sub_name].append(node)
        all_nodes.append(node)
    map_sub = {map_id: sub_name for sub_name, sub in sub_atlases.items() for map_id in sub.maps}
    seams: List[Tuple[Node, Node]] = []
    for i, conn in enumerate(connections):
        if not conn.keypoint_matches:
            continue
        sub_a, sub_b = map_sub.get(conn.from_map), map_sub.get(conn.to_map)
        src = [a for a, _ in conn.keypoint_matches]
        dst = [b for _, b in conn.keypoint_matches]
        label = f"connection {conn.from_map}-{conn.to_map}"
        if sub_a == sub_b:
            pts = [atlas.map_points[x].pos for x in src + dst]
            kf_ids = {k for x in src + dst for k in mp_kfs[x]}
            node = Node(gtsam.symbol('c', 2 * i), np.mean(pts, axis=0), label,
                        {conn.from_map, conn.to_map}, kf_ids)
            all_nodes.append(node)
            if sub_a in nodes:
                nodes[sub_a].append(node)
        else:
            side_nodes = []
            for j, (mps, map_id, sub_name) in enumerate(((src, conn.from_map, sub_a), (dst, conn.to_map, sub_b))):
                node = Node(gtsam.symbol('c', 2 * i + j), np.mean([atlas.map_points[x].pos for x in mps], axis=0),
                            f"{label} (map {map_id})", {map_id}, {k for x in mps for k in mp_kfs[x]})
                all_nodes.append(node)
                if sub_name in nodes:
                    nodes[sub_name].append(node)
                side_nodes.append(node)
            seams.append((side_nodes[0], side_nodes[1]))
    return nodes, all_nodes, seams


def node_distances(atlas: Atlas, all_nodes: List[Node], connections: List[Connection]
                   ) -> Dict[int, Dict[Tuple[str, int], float]]:
    """For each node, the number of keyframe steps to every keyframe ("k", id) and map point ("m", id),
    found by walking through keyframes that share map points. Matched map points of a connection count
    as the same point, so the walk can cross from one map to another there. Unreachable things are left out"""
    H = nx.Graph()
    for kf in atlas.keyframes.values():
        for mp_id in set(kf._map_points):
            if mp_id >= 0:
                H.add_edge(("k", kf.id), ("m", mp_id), weight=0.5) # keyframe - mappoint - keyframe is one step
    for conn in connections:
        for a, b in conn.keypoint_matches:
            H.add_edge(("m", a), ("m", b), weight=0.0)
    out: Dict[int, Dict[Tuple[str, int], float]] = {}
    for node in all_nodes:
        sources = [("k", k) for k in node.kfs if ("k", k) in H]
        # a node isn't a vertex of H, so routes can't take a shortcut through it
        found = nx.multi_source_dijkstra_path_length(H, sources, weight="weight") if sources else {}
        # a map point is as far away as the nearest keyframe that sees it, not half a step further
        out[node.key] = {v: (d - 0.5 if v[0] == "m" else d) for v, d in found.items()}
    return out


def keyframe_hops(all_nodes: List[Node], dists: Dict[int, Dict[Tuple[str, int], float]]
                  ) -> Dict[Tuple[int, int], float]:
    """Keyframe steps between each pair of nodes that have a route between them"""
    hops: Dict[Tuple[int, int], float] = {}
    for node in all_nodes:
        for other in all_nodes:
            if other is node:
                continue
            steps = [dists[node.key][("k", k)] for k in other.kfs if ("k", k) in dists[node.key]]
            if steps:
                hops[(node.key, other.key)] = min(steps)
    return hops


def build_tree(nodes: List[Node], hops: Dict[Tuple[int, int], float]) -> List[Tuple[Node, Node]]:
    """Minimum spanning tree of the nodes, where the cost of an edge is the number of keyframes
    between its nodes (with distance only breaking ties), so the tree follows the path the camera took"""
    G = nx.Graph()
    G.add_nodes_from(range(len(nodes)))
    for i in range(len(nodes)):
        for j in range(i + 1, len(nodes)):
            steps = hops.get((nodes[i].key, nodes[j].key), UNREACHABLE_HOPS)
            distance = float(np.linalg.norm(nodes[i].pos - nodes[j].pos))
            G.add_edge(i, j, weight=steps + DISTANCE_TIEBREAK * distance)
    tree = nx.minimum_spanning_tree(G)
    return [(nodes[i], nodes[j]) for i, j in tree.edges]


def solve_nodes(all_nodes: List[Node], edges: List[Tuple[Node, Node]], seams: List[Tuple[Node, Node]]) -> None:
    G = gtsam.NonlinearFactorGraph()
    values = gtsam.Values()
    for node in all_nodes:
        if node.fixed is not None:
            # surveyed points are invariant
            G.add(gtsam.NonlinearEqualityPoint3(node.key, node.fixed))
            values.insert(node.key, node.fixed)
        else:
            G.add(gtsam.PriorFactorPoint3(node.key, node.pos,
                                          gtsam.noiseModel.Isotropic.Sigma(3, WEAK_PRIOR_NOISE)))
            values.insert(node.key, node.pos)
    for a, b in edges:
        delta = b.pos - a.pos
        sigma = EDGE_NOISE_BASE + EDGE_NOISE_PER_METRE * float(np.linalg.norm(delta))
        G.add(gtsam.BetweenFactorPoint3(a.key, b.key, delta, gtsam.noiseModel.Isotropic.Sigma(3, sigma)))
    for a, b in seams: # the two sides of a connection are the same place
        G.add(gtsam.BetweenFactorPoint3(a.key, b.key, np.zeros(3), gtsam.noiseModel.Isotropic.Sigma(3, SEAM_NOISE)))
    result = run_optimisation(G, values)
    for node in all_nodes:
        node.new_pos = np.array(result.atPoint3(node.key))
        print(f"{node.label}: moved {np.linalg.norm(node.displacement):.3f}m")


def edge_offsets(points: np.ndarray, edges: List[Tuple[Node, Node]], chunk: int = 20000) -> np.ndarray:
    """Offset for each point: the node displacements interpolated along the nearest tree edge"""
    A = np.array([a.pos for a, _ in edges])
    B = np.array([b.pos for _, b in edges])
    dA = np.array([a.displacement for a, _ in edges])
    dB = np.array([b.displacement for _, b in edges])
    AB = B - A
    len2 = np.einsum("ij,ij->i", AB, AB)
    len2[len2 == 0] = 1.0
    out = np.zeros_like(points)
    for start in range(0, len(points), chunk):
        P = points[start:start + chunk]
        t = np.clip(np.einsum("pei,ei->pe", P[:, None, :] - A[None], AB) / len2, 0.0, 1.0)
        closest = A[None] + t[..., None] * AB[None]
        nearest = np.argmin(np.linalg.norm(P[:, None, :] - closest, axis=2), axis=1)
        rows = np.arange(len(P))
        tn = t[rows, nearest][:, None]
        out[start:start + chunk] = dA[nearest] + tn * (dB[nearest] - dA[nearest])
    return out


def hop_offsets(items: List[Tuple[str, int]], positions: np.ndarray, nodes: List[Node],
                edges: List[Tuple[Node, Node]], dists: Dict[int, Dict[Tuple[str, int], float]]
                ) -> Tuple[np.ndarray, np.ndarray]:
    """Offset for each keyframe ("k", id) or map point ("m", id), by keyframe steps rather than distance.
    The tree edge with the smallest total steps to its two nodes is the one the item lies along, and its
    offset is those two nodes' displacements blended by how close (in steps) it is to each.
    Distance only breaks ties, as it can't tell apart nodes that see the same keyframes.
    Returns the offsets and a mask of the items that could be placed (some have no route to any edge)"""
    index = {node.key: i for i, node in enumerate(nodes)}
    D = np.array([[dists[node.key].get(item, np.inf) for node in nodes] for item in items], dtype=float).reshape(len(items), len(nodes))
    D += DISTANCE_TIEBREAK * np.linalg.norm(positions[:, None, :] - np.array([n.pos for n in nodes])[None], axis=2)
    disp = np.array([node.displacement for node in nodes])
    iu = np.array([index[a.key] for a, _ in edges])
    iv = np.array([index[b.key] for _, b in edges])
    Du, Dv = D[:, iu], D[:, iv]
    best = np.argmin(Du + Dv, axis=1)
    rows = np.arange(len(items))
    du, dv = Du[rows, best], Dv[rows, best]
    placed = np.isfinite(du + dv)
    total = np.where(placed, du + dv, 1.0)
    t = np.where(total > 0, np.where(placed, du, 0.0) / np.where(total > 0, total, 1.0), 0.5) # 0 at u, 1 at v
    offsets = (1 - t)[:, None] * disp[iu[best]] + t[:, None] * disp[iv[best]]
    return np.where(placed[:, None], offsets, 0.0), placed


def morph_sub_atlas(atlas: Atlas, sub: SubAtlas, nodes: List[Node], edges: List[Tuple[Node, Node]],
                    dists: Dict[int, Dict[Tuple[str, int], float]], moved_points: Set[int]) -> None:
    if not edges: # a lone node: it just moves everything by its displacement
        edges = [(nodes[0], nodes[0])]
    mp_ids = sorted({mp_id for m in sub.maps for mp_id in atlas.maps[m].map_point_ids()} - moved_points)
    if mp_ids:
        pts = np.array([atlas.map_points[i].pos for i in mp_ids])
        offsets, placed = hop_offsets([("m", i) for i in mp_ids], pts, nodes, edges, dists)
        if not placed.all(): # not connected to any node by shared map points: use distance instead
            offsets[~placed] = edge_offsets(pts[~placed], edges)
        for mp_id, pos in zip(mp_ids, pts + offsets):
            atlas.map_points[mp_id].pos = tuple(pos.tolist())
        moved_points.update(mp_ids)
    kfs = [kf for m in sub.maps for kf in atlas.maps[m].keyframes.values()]
    if kfs:
        positions = np.array([kf.pose.translation for kf in kfs])
        offsets, placed = hop_offsets([("k", kf.id) for kf in kfs], positions, nodes, edges, dists)
        if not placed.all():
            offsets[~placed] = edge_offsets(positions[~placed], edges)
        for kf, pos in zip(kfs, positions + offsets):
            kf.pose = sst.RigidTransform.from_components(pos, kf.pose.rotation)


def parse_prefixes(specs: List[str], sub_names: List[str]) -> Dict[str, str]:
    """Survey prefix for each sub-atlas from command line values: "prefix" applies to all
    sub-atlases, "name=prefix" to just one (and wins over the general one)"""
    prefixes: Dict[str, str] = {}
    for spec in specs:
        if "=" not in spec:
            prefixes.update({name: spec for name in sub_names if name not in prefixes})
    for spec in specs:
        if "=" in spec:
            name, prefix = spec.split("=", 1)
            if name not in sub_names:
                raise ValueError(f"No sub-atlas called {name!r} (have: {', '.join(sub_names)})")
            prefixes[name] = prefix
    return prefixes


@dataclass
class SurveyFit:
    """What was learned while fitting to the survey that the bundle adjustment needs afterwards, when the
    maps have been merged and the sub-atlases they came from are no longer visible"""
    stations: Dict[str, np.ndarray]
    prefixes: Dict[str, str]
    kf_sub: Dict[int, str] # sub-atlas of each keyframe


def morph_to_survey(atlas: Atlas, sub_atlases: Dict[str, SubAtlas], connections: List[Connection],
                    survey_3d: Optional[Path], prefixes: Optional[Dict[str, str]] = None) -> SurveyFit:
    """Register each sub-atlas to the survey, then bend each sub-atlas so its aruco markers sit on their stations.
    prefixes gives the survex prefix of each sub-atlas where it isn't just the sub-atlas's name"""
    stations = read_survey_stations(survey_3d)
    fit = SurveyFit(stations, prefixes or {}, {kf_id: name for name, sub in sub_atlases.items()
                                                for m in sub.maps for kf_id in atlas.maps[m].keyframes})
    nodes, all_nodes, seams = build_nodes(atlas, sub_atlases, connections, stations, prefixes or {})
    dists = node_distances(atlas, all_nodes, connections)
    hops = keyframe_hops(all_nodes, dists)
    edges: Dict[str, List[Tuple[Node, Node]]] = {}
    seen: Set[frozenset] = set()
    all_edges: List[Tuple[Node, Node]] = []
    for sub_name, sub_nodes in nodes.items():
        edges[sub_name] = build_tree(sub_nodes, hops) if len(sub_nodes) > 1 else []
        for a, b in edges[sub_name]:
            if frozenset((a.key, b.key)) not in seen: # two sub-atlases may share an edge
                seen.add(frozenset((a.key, b.key)))
                all_edges.append((a, b))
    solve_nodes(all_nodes, all_edges, seams)
    moved_points: Set[int] = set()
    for sub_name, sub in sub_atlases.items():
        if nodes[sub_name]:
            morph_sub_atlas(atlas, sub, nodes[sub_name], edges[sub_name], dists, moved_points)
        else:
            print(f"{sub_name} has no aruco markers or connections, so can't be morphed")
    return fit


def bundle_adjust(atlas: Atlas, fit: SurveyFit, iters: int = 50, prior_sigma: float = ARUCO_PRIOR_NOISE,
                  robust: bool = False) -> None:
    """Bundle adjustment of the whole (merged) atlas: keyframe poses and map points are refined against the
    image measurements, the IMU, and the surveyed aruco stations. Run after the maps have been unified, so
    the sub-atlases are held together by their shared map points.
    The stations are held by very tight priors (prior_sigma) rather than as hard constraints. With
    robust=True they use a Huber loss, which lets a marker that disagrees with the survey drift away, but
    on test data that let markers wander by up to a metre, so it is off by default"""
    G = gtsam.NonlinearFactorGraph()
    values = gtsam.Values()
    station_keys: Dict[str, int] = {}
    mp_ids: Set[int] = set()
    prior_noise = gtsam.noiseModel.Isotropic.Sigma(3, prior_sigma)
    if robust:
        prior_noise = gtsam.noiseModel.Robust.Create(gtsam.noiseModel.mEstimator.Huber.Create(1.345), prior_noise)
    for mapp in atlas.maps.values():
        has_marker = False
        for kf in mapp.keyframes.values():
            for factor in kf.get_all_factors(False):
                G.add(factor)
            kf.initialise_pose(values, False)
            mp_ids.update(mp.id for mp in kf.map_points if mp is not None)
            sub_name = fit.kf_sub.get(kf.id)
            cal = kf.get_cal()
            for name, (x_left, y, x_right) in kf.arucos.items():
                label = find_station_label(fit.stations, fit.prefixes.get(sub_name, sub_name), name)
                if label is None:
                    continue
                key = station_keys.setdefault(label, gtsam.symbol('z', len(station_keys)))
                has_marker = True
                if x_left >= 0:
                    G.add(gtsam.GenericProjectionFactorCal3_S2((x_left, y), SMART_NOISE, K(kf.id), key, cal))
                if x_right >= 0:
                    G.add(gtsam.GenericProjectionFactorCal3_S2((x_right, y), SMART_NOISE, RIGHT(kf.id), key, cal))
        if not has_marker and mapp.keyframes: # nothing ties this map to the survey, so stop it drifting
            anchor = next(iter(mapp.keyframes.values()))
            G.add(gtsam.PoseTranslationPrior3D(K(anchor.id), anchor.get_gtsam_pose(), ANCHOR_NOISE))
    for mp_id in mp_ids:
        values.insert(M(mp_id), np.array(atlas.map_points[mp_id].pos))
    for label, key in station_keys.items():
        G.add(gtsam.PriorFactorPoint3(key, fit.stations[label], prior_noise))
        values.insert(key, fit.stations[label])
    print(f"Bundle adjusting {len(atlas.keyframes)} keyframes, {len(mp_ids)} map points, "
          f"{len(station_keys)} surveyed stations. Error before: {G.error(values):.1f}")
    before = {kf.id: np.array(kf.pose.translation) for kf in atlas.keyframes.values()}
    result = run_optimisation(G, values, iters)
    print(f"Error after: {G.error(result):.1f}")
    for kf in atlas.keyframes.values():
        kf.set_gtsam_pose(result.atPose3(K(kf.id)))
    for mp_id in mp_ids:
        atlas.map_points[mp_id].pos = tuple(result.atPoint3(M(mp_id)).tolist())
    moved = np.array([np.linalg.norm(np.array(kf.pose.translation) - before[kf.id]) for kf in atlas.keyframes.values()])
    print(f"Keyframes moved by a median of {np.median(moved):.3f}m, at most {moved.max():.3f}m")
    for label, key in station_keys.items():
        print(f"  {label}: {np.linalg.norm(result.atPoint3(key) - fit.stations[label]):.3f}m from its surveyed position")
