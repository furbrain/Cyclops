#!/usr/bin/env python3

"""
This program is to correct a bug in the original software where we had specified an incorrect transform
between the left and right cameras. The transform given was T_rl, when what was wanted was T_lr
This introduced subtle bugs leading to bad recognition and scale distortions.

This code reads an atlas and the Tlr transform and inverts it.
"""
import argparse
import functools

import cv2
import scipy.spatial.transform as sst
import yaml
import numpy as np

import orb_slam3_py as op


def load_stereo_extrinsics(yaml_path):
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    t = data['translation']
    T = np.array([t['x'], t['y'], t['z']])

    r = data['rotation']
    q = np.array([r['x'], r['y'], r['z'], r['w']])  # xyzw convention
    R = sst.Rotation.from_quat(q)
    return sst.RigidTransform.from_components(T, R)

def correct_point(maps, K, R, P, point):
    x = maps[0][round(point[1]),round(point[0])]
    y = maps[1][round(point[1]),round(point[0])]
    return cv2.undistortPoints(np.array([[x,y]]),K, None, R=R, P=P)[0,0]

def get_transform(Trl, K, image_size=(640,480)):
    Tlr = Trl.inv()
    bad_results = cv2.stereoRectify(K,None,K,None, image_size,
                                    Tlr.rotation.as_matrix(),
                                    Tlr.translation,
                                    flags=cv2.CALIB_ZERO_DISPARITY)
    good_results = cv2.stereoRectify(K,None,K,None, image_size,
                                    Trl.rotation.as_matrix(),
                                    Trl.translation,
                                    flags=cv2.CALIB_ZERO_DISPARITY)
    R_bad_1, R_bad_2, P_bad_1, P_bad_2, *_ = bad_results
    R_good_1, R_good_2, P_good_1, P_good_2, *_ = good_results
    print(K)
    bad_maps_1 = cv2.initUndistortRectifyMap(K, None, R_bad_1, P_bad_1,image_size, cv2.CV_32FC1)
    bad_maps_2 = cv2.initUndistortRectifyMap(K, None, R_bad_2, P_bad_2,image_size, cv2.CV_32FC1)
    return (functools.partial(correct_point, bad_maps_1, K, R_good_1, P_good_1),
            functools.partial(correct_point, bad_maps_2, K, R_good_2, P_good_2))


parser = argparse.ArgumentParser(description="Correct and atlas file")
parser.add_argument('-a', '--atlas', help="provide an atlas message file to use", required=True)
parser.add_argument('-t', '--trl', help="give the Trl transform", required=True)
parser.add_argument('-o', '--output', help="output file to use", required=True)
opts = parser.parse_args()

Trl = load_stereo_extrinsics(opts.trl)
atlas = op.load_atlas(opts.atlas, binary=False)
kfs = atlas.get_all_keyframes()
t1, t2 = get_transform(Trl, kfs[0].K())

for k in kfs:
    kps = []
    us = []
    print(f"correcting keyframe: {k.id}")
    for idx, (kp, x2) in enumerate(zip(k.get_keypoints_undistorted(), k.get_u_right())):
        xy1 = t1(kp)
        k.set_keypoint_at(idx, xy1[0], xy1[1])
        kps.append(xy1)
        if x2 >=0:
            xy2 = t2((x2,xy1[1]))
            k.set_u_right_at(idx, xy2[0])

print("saving atlas")
op.save_atlas(atlas, opts.output, binary=False)