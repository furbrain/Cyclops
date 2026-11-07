#!/usr/bin/env python3
from collections import defaultdict
from pathlib import Path
from typing import List, Dict, Optional
import os

import argparse
import numpy as np

from atlas import Atlas, Keyframe, SubSurvey, make_path
from utils import IMAGE_FILENAME, CAM_FILENAME, POINTS_FILENAME, SPARSE_POINTS_FILENAME
import scipy.spatial.transform as sst


def init_map_dir(map_dir: Path) -> Path:
    sparse_dir = map_dir / "sparse"
    sparse_dir.mkdir(parents=True, exist_ok=True)
    # create symlink to images (like original)
    if not (map_dir / "images").exists():
        os.symlink("../images", map_dir / "images")
    return sparse_dir

def get_img_stem(image_dir: Path, kf: Keyframe, sub_survey: SubSurvey) -> Optional[str]:
    """Image path (relative to image_dir, without .jpg) of a keyframe's left image, or None if not found.
    Sub-survey images are expected at <sub_survey.path>/images/<id within sub-survey>.jpg"""
    kf_id = kf.id - sub_survey.kf_id_offset
    folder = Path() if sub_survey.path == "." else Path(sub_survey.path) / "images"
    for name in (str(kf_id), f"{kf_id:06d}"): # try short form first, then try longer
        if (image_dir / folder / f"{name}.jpg").exists():
            return (folder / name).as_posix()
    return None

def create_colmap(atlas:Atlas, model_dir: Path, image_dir: Path, single: bool = False):
    maps = sorted(atlas.maps.values(), key=lambda x: len(x.keyframes), reverse=True)
    kf_sub_surveys = {kf_id: sub for sub in atlas.get_sub_surveys() for kf_id in sub.kf_offsets}
    ##FIXME## Just assume all cameras the same at this point...
    camera = next(iter(maps[0].keyframes.values())).camera
    if single:
        maps = maps[:1]
    K = camera.K
    for map_idx, m in enumerate(maps, start=1):
        if single:
            map_dir = model_dir
        else:
            map_dir = model_dir / f"map_{map_idx}"
        map_dir.mkdir(parents=True, exist_ok=True)
        sparse_dir = init_map_dir(map_dir)
        with open(sparse_dir / CAM_FILENAME, "w") as f:
            f.write(f"1 PINHOLE {camera.size[0]} {camera.size[1]} {K[0,0]} {K[1,1]} {K[0,2]} {K[1,2]}")
        found_mps: Dict[int, List[str]] = defaultdict(list)
        right_id_offset = max(m.keyframes)+1
        with open(sparse_dir / IMAGE_FILENAME, "w") as f:
            for kf in sorted(m.keyframes.values(), key=lambda x: x.id):
                img_stem = get_img_stem(image_dir, kf, kf_sub_surveys[kf.id])
                if img_stem is None:
                    continue
                # keyed by map point so each point is only observed once per image
                mp_kps = {mp.id: kp for kp, mp in zip(kf.key_points, kf.map_points) if mp is not None}
                pose: sst.RigidTransform = kf.pose.inv() # colmap wants world to camera
                o = pose.rotation.as_quat(scalar_first=True)
                t: np.ndarray = pose.translation
                f.write(f"{kf.id} {o[0]} {o[1]} {o[2]} {o[3]} {t[0]} {t[1]} {t[2]} 1 {img_stem}.jpg\n")
                line = []
                for idx, (mp_id, (x, y, _)) in enumerate(mp_kps.items()):
                    line.append(f"{x} {y} {mp_id}")
                    found_mps[mp_id].append(f"{kf.id} {idx}")
                f.write(" ".join(line) + "\n")
                if (image_dir / f"{img_stem}r.jpg").exists():
                    T_left_right = sst.RigidTransform.from_translation([kf.baseline, 0, 0])
                    pose = (kf.pose * T_left_right).inv()
                    o = pose.rotation.as_quat(scalar_first=True)
                    t: np.ndarray = pose.translation
                    f.write(f"{kf.id + right_id_offset} {o[0]} {o[1]} {o[2]} {o[3]} {t[0]} {t[1]} {t[2]} 1 "
                            f"{img_stem}r.jpg\n")
                    line = []
                    right_kps = [(mp_id, x_right, y) for mp_id, (_, y, x_right) in mp_kps.items() if x_right >= 0]
                    for idx, (mp_id, x, y) in enumerate(right_kps):
                        line.append(f"{x} {y} {mp_id}")
                        found_mps[mp_id].append(f"{kf.id + right_id_offset} {idx}")
                    f.write(" ".join(line) + "\n")
        with open(sparse_dir / POINTS_FILENAME, "w") as points_f:
            for mp_id, track in found_mps.items():
                pt = atlas.map_points[mp_id].pos
                points_f.write(f"{mp_id} {pt[0]} {pt[1]} {pt[2]} 255 255 255 0 {' '.join(track)}\n")

if __name__=="__main__":

    parser = argparse.ArgumentParser(description="extract keyframes and points from an atlas and save as COLMAP")
    parser.add_argument('-a', '--atlas', help="provide an atlas message file to use")
    parser.add_argument('-c', '--combined', help="ignored: image locations now come from the atlas's sub-surveys",
                        action="store_true")
    parser.add_argument('-d', '--dir', help="directory to use", required=True)
    parser.add_argument('-s', '--single', help="just create largest map", action="store_true")
    opts = parser.parse_args()

    model_dir = Path(opts.dir)
    if opts.atlas:
        atlas_file = make_path(model_dir, opts.atlas)
        atlas = Atlas.from_osa(atlas_file) if atlas_file.suffix == ".osa" else Atlas.from_json_file(atlas_file)
    else:
        atlas = Atlas.from_dir(model_dir)
    image_dir = model_dir / "images"
    create_colmap(atlas, model_dir, image_dir, single=opts.single)
