#!/usr/bin/env python3
from collections import defaultdict
from pathlib import Path
from typing import List, Dict
import os

import argparse
import numpy as np

import orb_slam3_py as op

from atlas_tools import Atlas, KeyFrameData
from utils import IMAGE_FILENAME, CAM_FILENAME, POINTS_FILENAME, SPARSE_POINTS_FILENAME
import scipy.spatial.transform as sst

def init_map_dir(map_dir: Path) -> Path:
    sparse_dir = map_dir / "sparse"
    sparse_dir.mkdir(parents=True, exist_ok=True)
    # create symlink to images (like original)
    if not (map_dir / "images").exists():
        os.symlink("../images", map_dir / "images")

    # camera file symlink in sparse
    if not (sparse_dir / CAM_FILENAME).exists():
        os.symlink("../../" + CAM_FILENAME, sparse_dir / CAM_FILENAME)
    return sparse_dir

def create_colmap(atlas:Atlas, model_dir: Path, image_dir: Path):
    maps = list(reversed(sorted(atlas.maps.values(), key=lambda x:len(x.kfs))))
    ##FIXME## Just assume all cameras the same at this point...
    kf0: KeyFrameData = maps[0].kfs[0]
    K = kf0.K
    with open(model_dir / CAM_FILENAME, "w") as f:
        f.write(f"1 PINHOLE 640 480 {K[0,0]} {K[1,1]} {K[0,2]} {K[1,2]}")
    for map_idx, m in enumerate(maps, start=1):
        map_dir = model_dir / f"map_{map_idx}"
        map_dir.mkdir(parents=True, exist_ok=True)
        sparse_dir = init_map_dir(map_dir)
        found_mps: Dict[op.MapPoint, List[str]] = defaultdict(list)
        right_id_offset = max(x.id for x in m.kfs)+1
        with open(sparse_dir / IMAGE_FILENAME, "w") as f:
            for kf in m.kfs:
                pose: sst.RigidTransform = kf.kf.get_pose()
                if (image_dir / f"{kf.id}.jpg").exists():
                    o = pose.rotation.as_quat(scalar_first=True)
                    t: np.ndarray = pose.translation
                    f.write(f"{kf.id} {o[0]} {o[1]} {o[2]} {o[3]} {t[0]} {t[1]} {t[2]} 1 {kf.id}.jpg\n")
                    line = []
                    idx = 0
                    for mp, (x, y) in kf.mp_kps.items():
                        if mp:
                            line.append(f"{x} {y} {mp.id}")
                            found_mps[mp].append(f"{kf.id} {idx}")
                            idx += 1
                    f.write(" ".join(line) + "\n")
                if (image_dir / f"{kf.id}r.jpg").exists():
                    T_left_right = sst.RigidTransform.from_translation([kf.kf.baseline, 0, 0])
                    pose = pose * T_left_right
                    o = pose.rotation.as_quat(scalar_first=True)
                    t: np.ndarray = pose.translation
                    f.write(f"{kf.id + right_id_offset} {o[0]} {o[1]} {o[2]} {o[3]} {t[0]} {t[1]} {t[2]} 1 {kf.id}r.jpg\n")
                    line = []
                    idx = 0
                    for mp, (x, y) in kf.mp_rkps.items():
                        if mp and (x>=0):
                            line.append(f"{x} {y} {mp.id}")
                            found_mps[mp].append(f"{kf.id + right_id_offset} {idx}")
                            idx += 1
                    f.write(" ".join(line) + "\n")
        with open(sparse_dir / POINTS_FILENAME, "w") as points_f:
            for mp, track in found_mps.items():
                pt = mp.get_world_pos()
                points_f.write(f"{mp.id} {pt[0]} {pt[1]} {pt[2]} 255 255 255 0 {' '.join(track)}\n")

if __name__=="__main__":
    parser = argparse.ArgumentParser(description="extract keyframes and points from an atlas and save as COLMAP")
    parser.add_argument('-a', '--atlas', help="provide an atlas message file to use")
    parser.add_argument('-d', '--dir', help="directory to use", required=True)
    opts = parser.parse_args()

    model_dir = Path(opts.dir)
    if opts.atlas:
        atlas = Atlas.from_file(opts.atlas)
    else:
        atlas = Atlas.from_file(model_dir / "atlas.txt.gz")
    image_dir = model_dir / "images"
    create_colmap(atlas, model_dir, image_dir)