#!/usr/bin/env python3
import json
from typing import List
from pathlib import Path
from atlas_tools import Atlas, KeyFrameData
from orb_slam3_py import Leg


def create_info(atlas: Atlas, model_dir: Path):
    info = {
        'points': len(atlas.atlas.get_all_map_points()),
        'keyframes': len(atlas.atlas.get_all_keyframes()),
        'submaps': len(atlas.maps)
    }
    mapp = atlas.get_biggest_map()
    pointcloud = [mp.get_world_pos().tolist() for mp in mapp.mps.values()]
    kfs: List[KeyFrameData] = sorted(mapp.kfs, key=lambda x: x.id)
    track = [kf.orig_pose.translation().tolist() for kf in kfs]
    survey = []
    for legs in atlas.atlas.surveys.values():
        for leg in legs:
            if leg.from_station:
                survey.append(leg.from_station.position.tolist())
            if leg.to_station:
                survey.append(leg.to_station.position.tolist())
    with open(model_dir / "info.json", "w") as f:
        json.dump(info, f, indent=4)
    with open(model_dir/ "pointcloud.json", "w") as f:
        json.dump({"points": pointcloud}, f, indent=4)
    with open(model_dir / "track.json", "w") as f:
        json.dump({"points": track}, f, indent=4)
    with open(model_dir / "survey.json", "w") as f:
        json.dump({"points": survey}, f, indent=4)


if __name__=="__main__":
    import argparse
    parser = argparse.ArgumentParser(description="extract keyframes and points from an atlas and save as COLMAP")
    parser.add_argument('-a', '--atlas', help="provide an atlas message file to use")
    parser.add_argument('-d', '--dir', help="directory to use", required=True)
    opts = parser.parse_args()

    model_dir = Path(opts.dir)
    if opts.atlas:
        atlas = Atlas.from_file(opts.atlas)
    else:
        atlas = Atlas.from_file(model_dir / "atlas.txt.gz")
    create_info(atlas, model_dir)