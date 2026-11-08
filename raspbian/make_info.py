#!/usr/bin/env python3
import json
from typing import List
from pathlib import Path
from atlas import Atlas, Keyframe
from survey_data import SURVEY_SVX, add_survey_to_atlas, ensure_survey_3d


def create_info(atlas: Atlas, model_dir: Path):
    map_info = []
    for mapp in atlas.maps.values():
        map_info.append({
            "id": mapp.id,
            "min": min(mapp.keyframes),
            "max": max(mapp.keyframes)
        })
    info = {
        'points': len(atlas.map_points),
        'keyframes': len(atlas.keyframes),
        'submaps': len(atlas.maps),
        'maps': map_info
    }
    mapp = atlas.get_biggest_map()
    pointcloud = [list(atlas.map_points[mp_id].pos) for mp_id in mapp.map_point_ids()]
    kfs: List[Keyframe] = sorted(mapp.keyframes.values(), key=lambda x: x.id)
    track = [kf.pose.translation.tolist() for kf in kfs]
    survey = []
    for srv in atlas.surveys.values():
        for leg in srv.legs:
            for name in (leg.from_station, leg.to_station):
                if name in srv.stations:
                    survey.append(list(srv.stations[name].pos))
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
        atlas_file = Path(opts.atlas)
        atlas = Atlas.from_osa(atlas_file) if atlas_file.suffix == ".osa" else Atlas.from_json_file(atlas_file)
    else:
        atlas = Atlas.from_dir(model_dir)
    svx = model_dir / SURVEY_SVX
    if svx.exists():
        add_survey_to_atlas(atlas, svx, ensure_survey_3d(model_dir))
        if not opts.atlas: # the atlas given with -a is only read from
            atlas.save(model_dir / "atlas.json.gz")
    create_info(atlas, model_dir)