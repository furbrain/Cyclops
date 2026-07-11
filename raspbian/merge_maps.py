#!/usr/bin/env python3
import argparse
from collections import defaultdict

import orb_slam3_py
import cv2
import numpy as np
import itertools
from typing import List, Tuple, Any
import scipy.spatial.transform as sst

class KeyFrameData:
    def __init__(self, kf: orb_slam3_py.KeyFrame):
        self.kf = kf
        self.id = kf.id
        self.all_descs = kf.get_descriptors()
        self.all_mps = kf.get_map_point_matches()
        self.all_kps = kf.get_keypoints_undistorted()
        self.mp_descs = np.array([desc for desc, mp in zip(self.all_descs, self.all_mps) if mp])
        self.mp_kps = [kp for kp, mp in zip(self.all_kps, self.all_mps) if mp]
        self.just_mps = [mp for mp in self.all_mps if mp]

class MapData:
    def __init__(self, map: orb_slam3_py.Map):
        self.map = map
        self.kfs = [KeyFrameData(x) for x in sorted(map.get_all_keyframes(), key=lambda x: x.id)]

def robust_translation_mask(translations, threshold):
    """Modified z-score outlier rejection based on distance from the median."""
    median_t = np.median(translations, axis=0)
    dist = np.linalg.norm(translations - median_t, axis=1)
    mad = np.median(np.abs(dist - np.median(dist)))
    scale = 1.4826 * mad if mad > 1e-9 else 1e-9
    modified_z = dist / scale
    return modified_z < threshold, dist


def average_matches(transforms: sst.RigidTransform, inlier_counts, total_counts,
                    labels=None, threshold=3.5, use_weights=True):
    """
    transforms:    RigidTransform of shape (N,) -- e.g. RigidTransform.concatenate([...])
    inlier_counts: array-like (N,) RANSAC inlier counts
    total_counts:  array-like (N,) total match counts
    labels:        optional list of (kf1, kf2) tuples for printing
    use_weights:   if True, weight the mean by raw RANSAC inlier count
    """
    transforms = sst.RigidTransform.concatenate(transforms)
    inlier_counts = np.asarray(inlier_counts, dtype=float)
    total_counts = np.asarray(total_counts, dtype=float)
    translations = transforms.translation  # (N, 3)

    mask, dist = robust_translation_mask(translations, threshold)
    inlier_tf = transforms[mask]
    weights = inlier_counts[mask] if use_weights else None  # raw inlier count as weight

    mean_tf = inlier_tf.mean(weights=weights)
    return mean_tf, mask


parser = argparse.ArgumentParser(description="Attempt to merge maps within an atlas")
parser.add_argument('-a', '--atlas', help="provide an atlas message file to use", default="")
opts = parser.parse_args()


atlas: orb_slam3_py.Atlas = orb_slam3_py.load_atlas(opts.atlas)
#lets just start comparing first two maps
print("aligning maps")
orb_slam3_py.align_atlas(atlas)
maps = [MapData(m) for m in atlas.get_all_maps()]
matcher = cv2.BFMatcher.create(normType=cv2.NORM_HAMMING)
for m0,m1 in itertools.combinations(maps, 2):
    results: List[Tuple[KeyFrameData, KeyFrameData, Any]] = []
    for kf1 in m0.kfs:
        #print(kf1.id)
        for kf2 in m1.kfs:
            matches = matcher.knnMatch(kf1.mp_descs, kf2.mp_descs,2)
            good: List[cv2.DMatch] = [m for m,n in matches if m.distance < 0.7 * n.distance]
            if len(good) > 6:
                links: List[Tuple[orb_slam3_py.MapPoint, orb_slam3_py.MapPoint]] = []
                for x in good:
                    links.append((kf1.just_mps[x.queryIdx], kf2.just_mps[x.trainIdx]))
                results.append((kf1, kf2, links))
                print(f"{kf1.id} -> {kf2.id}: {len(good)} matches")
    mp_matches = defaultdict(list)
    for kf1, kf2, links in results:
        for mp1, mp2 in links:
            mp_matches[mp1].append(mp2)
    connections: List[Tuple[orb_slam3_py.MapPoint, orb_slam3_py.MapPoint]] = []
    for orig, dests in mp_matches.items():
        counts = defaultdict(int)
        for dest in dests:
            counts[dest] += 1
        histogram: List[Tuple[orb_slam3_py.MapPoint, int]] = sorted(list(counts.items()), key=lambda x: x[1],
                                                                  reverse=True)
        if histogram[0][1] > 6:
            if len(histogram) == 1:
                connections.append((orig, histogram[0][0]))
            else:
                if histogram[0][1] > (3 * histogram[1][1]):
                    connections.append((orig, histogram[0][0]))

    points_a = []
    points_b = []
    print(len(connections))
    for a, b in connections:
        points_a.append(a.get_world_pos())
        points_b.append(b.get_world_pos())
    if len(connections) > 4:
        a = np.array(points_a)
        b = np.array(points_b)
        print(a.shape, b.shape)
        for th in (0.01, 0.05, 0.1, 0.2, 0.3, 0.4, 0.5,0.6, 0.7,0.8, 0.9, 1.0):
            ransacs = orb_slam3_py.ransac_horn_alignment(a, b, inlierThreshold=th)
            print(th, ransacs[2], ransacs[0].rotation.as_euler("ZYX",degrees=True), ransacs[0].translation)
    rotations: List[Tuple[sst.RigidTransform,int,int]] = []
    for kf1, kf2, _ in results:
        matches = matcher.knnMatch(kf1.mp_descs, kf2.all_descs, 2)
        good: List[cv2.DMatch] = [m for m, n in matches if m.distance < 0.7 * n.distance]
        if len(good) > 8:
            rw_coords = np.array([kf1.just_mps[x.queryIdx].get_world_pos() for x in good])
            img_coords = np.array([kf2.all_kps[x.trainIdx] for x in good])
            retval, rvec, tvec, inliers = cv2.solvePnPRansac(rw_coords, img_coords, kf2.kf.K(), None,
                                                             reprojectionError=5.0)
            if inliers is None:
                print("no inliers")
                continue
            r_a = sst.Rotation.from_rotvec(rvec.T[0])
            rota = sst.RigidTransform.from_components(tvec.T, r_a)
            rotb = kf2.kf.get_pose_inverse()
            offset =(rotb*rota)
            rotations.append((offset, len(inliers), len(good)))
            print(f"{kf1.id} -> {kf2.id} ({len(inliers)}/{len(good)}):"
                  f" {offset.rotation.as_euler('ZYX',degrees=True)}  "
                  f"{offset.translation}")
    if rotations:
        tfs, mask = average_matches(*(zip(*rotations)))
        print("---AVERAGES---")
        print(tfs.rotation.as_euler("ZYX",degrees=True))
        print(tfs.translation)
        print(mask)
