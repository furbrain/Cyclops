#!/usr/bin/env python3
import pathlib
from pathlib import Path

from utils import as_usec_from_stamp, get_cameras, export_images_from_bag, IMAGE_TOPIC, IMAGE_TOPIC_RIGHT
from atlas_tools import Atlas
from rosbags.rosbag2 import Reader
import argparse



def extract_images(atlas: Atlas, image_dir: Path, bag_file: Path):
    ts_map = {}
    for kf in atlas.atlas.get_all_keyframes():
        ts = as_usec_from_stamp(kf.timestamp)
        ts_map[ts] = kf
    left_camera, right_camera = get_cameras(bag_file, model_dir, None)
    image_dir = model_dir / "images"
    print(image_dir)
    with Reader(bag_file) as reader:
        right_ts = ts_map.copy()
        export_images_from_bag(reader, ts_map, IMAGE_TOPIC, left_camera, model_dir / "images")
        export_images_from_bag(reader, right_ts, IMAGE_TOPIC_RIGHT, right_camera, model_dir / "images", right=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="extract images from a bag and save them with keyfram id")
    parser.add_argument('-a', '--atlas', help="provide an atlas message file to use")
    parser.add_argument('-d', '--dir', help="directory to use", required=True)
    parser.add_argument('-b', '--bag', help="bag to use")
    opts = parser.parse_args()

    model_dir = pathlib.Path(opts.dir)
    if opts.atlas:
        atlas = Atlas.from_file(opts.atlas)
    else:
        atlas = Atlas.from_file(model_dir / "atlas.txt.gz")

    if opts.bag:
        bag_file = pathlib.Path(opts.bag)
    else:
        bag_file = model_dir / "recording"
    image_dir = model_dir / "images"
    extract_images(atlas, image_dir, bag_file)

