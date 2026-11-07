#!/usr/bin/env python3
from pathlib import Path

from atlas import Atlas
from atlas_tools import make_path


def make_json(atlas_file: Path, output: Path):
    atlas = Atlas.from_osa(atlas_file)
    atlas.find_arucos()
    atlas.save(output)


if __name__=="__main__":
    import argparse
    parser = argparse.ArgumentParser(description="convert an osa atlas to json, adding aruco observations from images.tar")
    parser.add_argument('-a', '--atlas', help="osa atlas file to use", default="atlas.osa")
    parser.add_argument('-o', '--output', help="json atlas file to write", default="atlas.json.gz")
    parser.add_argument('-d', '--dir', help="directory to use", required=True)
    opts = parser.parse_args()

    model_dir = Path(opts.dir)
    make_json(make_path(model_dir, opts.atlas), make_path(model_dir, opts.output))
