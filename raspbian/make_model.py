#!/usr/bin/env python3
"""
colmap_from_bag_ros2.py

Convert a ROS1 or ROS2 bag into files usable by COLMAP (cameras.txt, images.txt, points3D.txt)
and extract images into an images/ directory.

Requires: rosbags (pip install rosbags), cv_bridge, opencv-python (cv2)

It then uses docker to create a rough or refined model
"""

import argparse
import json
import os
import pathlib
import sys
from collections import defaultdict
from typing import Dict, Tuple

import cv2

# rosbags high-level reader
from rosbags.rosbag2 import Reader

from utils import ROOT_DIR, IMAGE_TOPIC, BAG_NAME, CAM_FILENAME, IMAGE_FILENAME, POINTS_FILENAME, \
    TS_MAP_FILENAME, SPARSE_POINTS_FILENAME, bridge, as_usec_from_stamp, \
    get_image_from_timestamp, deserialize


parser = argparse.ArgumentParser(description="Create a colmap from a bag (ROS1 or ROS2)")
parser.add_argument('-n', '--name', help="Name of Recording")
parser.add_argument('-r', '--refined', help="Create a refined model, rather than rough", action="store_true")
parser.add_argument('-s', '--submap', help="Submap to create", default=1, type=int)
parser.add_argument('-p', '--prep-only', help="Just create data for colmap and indices", action="store_true")
opts = parser.parse_args()

model_dir = ROOT_DIR / opts.name
cam_file = model_dir / CAM_FILENAME
images_dir = model_dir / "images"
images_dir.mkdir(parents=True, exist_ok=True)


def export_map(mp, points: Dict, map_dir: pathlib.Path, known_images: Dict[int, int]):
    sparse_dir = map_dir / "sparse"
    sparse_dir.mkdir(parents=True, exist_ok=True)
    # create symlink to images (like original)
    try:
        if not (map_dir / "images").exists():
            os.symlink("../images", map_dir / "images")
    except Exception:
        # on some filesystems / platforms symlink may fail; ignore
        pass

    # camera file symlink in sparse
    try:
        if not (sparse_dir / CAM_FILENAME).exists():
            os.symlink("../../" + CAM_FILENAME, sparse_dir / CAM_FILENAME)
    except Exception:
        pass

    required_points = defaultdict(list)
    frame_tss = set()
    sparse_points: Dict[int, Tuple] = {}
    with open(sparse_dir / IMAGE_FILENAME, "w") as image_f:
        for f in mp.frames:
            stamp_usec = as_usec_from_stamp(f.stamp)
            if stamp_usec not in known_images:
                print(f"Image {stamp_usec} not in bag ... skipping")
                continue
            frame_tss.add(stamp_usec)
            o = f.pose.orientation
            t = f.pose.position
            image_f.write(f"{f.id} {o.w} {o.x} {o.y} {o.z} {t.x} {t.y} {t.z} 1 {stamp_usec}.jpg\n")
            line = []
            bag_path = model_dir / BAG_NAME
            with Reader(bag_path) as reader:
                image = get_image_from_timestamp(reader, known_images, stamp_usec)
            for idx, pt in enumerate(f.points):
                p3d_id = pt.point3d_id
                required_points[p3d_id].append(f"{f.id} {idx}")
                line.append(f"{pt.x} {pt.y} {p3d_id}")
                if p3d_id not in sparse_points:
                    p3d = points[p3d_id]
                    b, g, r = image[int(pt.y), int(pt.x)]
                    sparse_points[p3d_id] = (p3d.x, p3d.y, p3d.z, int(r), int(g), int(b))
            image_f.write(" ".join(line) + "\n")
    with open(map_dir / SPARSE_POINTS_FILENAME, "w") as f:
        json.dump(list(sparse_points.values()), f)
    with open(sparse_dir / POINTS_FILENAME, "w") as points_f:
        for pt in points.values():
            if pt.id in required_points:
                track = " ".join(required_points[pt.id])
                points_f.write(f"{pt.id} {pt.x} {pt.y} {pt.z} 255 255 255 0 {track}\n")
    return frame_tss

def add_camera_from_bag(reader):
    """
    Search for a CameraInfo message in the bag and write cameras.txt
    Uses reader.messages() and reader.deserialize() for convenience.
    """
    for connection, timestamp, rawdata in reader.messages():
        topic = connection.topic
        # accept camera_info topic names ending with camera_info
        if topic.endswith("camera_info") or topic.endswith("camera/camera_info"):
            msg = deserialize(rawdata, "sensor_msgs/msg/CameraInfo")
            # try to read width/height and K and D fields from msg
            width = getattr(msg, "width", None)
            height = getattr(msg, "height", None)
            K = getattr(msg, "k", None)
            if K is None:
                K = getattr(msg, "K", None)
            D = getattr(msg, "d", None)
            if D is None:
                D = getattr(msg, "D", None)
            # Fallback field names for ROS1/ROS2
            if width is None or height is None:
                # maybe field names with capital letters or different structure:
                width = getattr(msg, "width", width)
                height = getattr(msg, "height", height)
            if width is None or height is None or K is None:
                # not enough info
                continue
            with open(cam_file, "w") as f:
                # PINHOLE requires fx fy cx cy
                # MUST be rectified
                f.write(f"1 PINHOLE {width} {height} {K[0]} {K[4]} {K[2]} {K[5]}")
            return True
    return False

def export_images_from_bag(reader: Reader, tss, image_topic):
    """
    reader: AnyReader
    tss: set of expected timestamps in usec
    image_topic: topic string to filter
    """
    # iterate messages on image_topic
    for connection, timestamp, rawdata in reader.messages():
        if connection.topic != image_topic:
            continue
        msg = deserialize(rawdata, connection.msgtype)

        # attempt to get header stamp
        header = getattr(msg, "header", None)
        if header is None:
            # some compressed images may have stamp at top-level; try msg.stamp
            stamp = getattr(msg, "stamp", None)
        else:
            stamp = header.stamp
        stamp_usec = as_usec_from_stamp(stamp)
        if stamp_usec in tss:
            path = str(images_dir / f"{stamp_usec}.jpg")
            # remove from set so we don't write duplicates
            tss.remove(stamp_usec)
            # check message type shape
            # For sensor_msgs/Image the deserialized msg should have .data / .height / .width / .encoding
            if getattr(msg, "_type", "").endswith("Image") or hasattr(msg, "data") and hasattr(msg, "height"):
                try:
                    img = bridge.imgmsg_to_cv2(msg)
                    cv2.imwrite(path, img)
                except Exception:
                    # fallback: try to access raw data and decode via numpy+cv2 if possible
                    try:
                        # this is best-effort and may fail for unusual encodings
                        import numpy as np
                        arr = np.asarray(msg.data)
                        # try OpenCV imdecode if msg is compressed bytes
                        nparr = np.frombuffer(arr.tobytes(), np.uint8)
                        img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                        if img is not None:
                            cv2.imwrite(path, img)
                        else:
                            raise
                    except Exception:
                        print(f"Failed to save image for timestamp {stamp_usec}")
                        raise
            else:
                # Maybe sensor_msgs/CompressedImage
                if getattr(msg, "format", None) is not None and getattr(msg, "data", None) is not None:
                    fmt = getattr(msg, "format", "")
                    if "jpeg" in fmt.lower():
                        # save raw bytes
                        with open(path, "wb") as f:
                            f.write(bytes(msg.data))
                    else:
                        try:
                            img = bridge.compressed_imgmsg_to_cv2(msg)
                            cv2.imwrite(path, img)
                        except Exception:
                            print(f"Couldn't decode compressed image at {stamp_usec}")
                            raise
                else:
                    # unknown image message type
                    print(f"Unknown image message type for topic {connection.topic}; skipping timestamp {stamp_usec}")

def get_available_images(reader: Reader, topic: str) -> Dict[int, int]:
    tss: Dict[int, int] = {}
    for connection, timestamp, rawdata in reader.messages():
        if connection.topic == topic:
            msg = deserialize(rawdata, connection.msgtype)
            ts = as_usec_from_stamp(msg.header.stamp)
            tss[ts] = timestamp
    return tss

def create_colmap():
    bag_path = model_dir / BAG_NAME
    if not (bag_path / "metadata.yaml").exists():
        os.system(f"ros2 bag reindex {bag_path}")
    with Reader(bag_path) as reader:
        known_images = get_available_images(reader, IMAGE_TOPIC)

    with Reader(bag_path) as reader:
        # 1) try to find camera info and write cameras.txt
        ok = add_camera_from_bag(reader)
        if not ok:
            print("Warning: camera_info not found in bag. cameras.txt will not be created or may be incomplete.")

        # 2) find the last atlas message on /orb_slam3/atlas (mimics original script behavior)
        # We'll find all messages on that topic then pick last
        atlas_topic = "/orb/ORB/atlas"
        last_atlas = None
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == atlas_topic:
                msg = deserialize(rawdata, connection.msgtype)
                last_atlas = msg
                last_ts = timestamp
        if last_atlas is None:
            raise RuntimeError(f"No messages found on {atlas_topic} — cannot extract maps/points/frames")

    # The original script sorted maps by their frame count and exported each.
    # We'll follow same logic. The fields expected on atlas message are 'maps' and 'points'.
    # These are user-defined types produced by ORB_SLAM3 - we assume the structure used in your ROS1 bag.
    maps = list(sorted(last_atlas.maps, key=lambda x: len(x.frames), reverse=True))
    frame_tss = set()
    points_dict = {x.id: x for x in last_atlas.points}
    for map_idx, mp in enumerate(maps, start=1):
        map_dir = model_dir / f"map_{map_idx}"
        map_dir.mkdir(parents=True, exist_ok=True)
        tss = export_map(mp, points_dict, map_dir, known_images)
        frame_tss = frame_tss.union(tss)

    print(f"Need to extract {len(frame_tss)} images")
    # Re-open reader to iterate through images (AnyReader supports re-iterating)
    # export images
    with Reader(bag_path) as reader:
        export_images_from_bag(reader, frame_tss, IMAGE_TOPIC)
        if len(frame_tss) > 0:
            print("missing images: ", frame_tss)
        else:
            print("All images exported.")
    return known_images


if __name__ == "__main__":
    if not cam_file.exists():
        ts_map = create_colmap()
        json.dump(ts_map, open(model_dir / TS_MAP_FILENAME, "w"))
    else:
        print("COLMAP data already exists\n")
    sys.stdout.flush()
    # os.system("docker pull furbrain/cyclops_mvs:latest")
    if not opts.prep_only:
        if opts.refined:
            os.system(f"docker run --user $(id -u):$(id -g) -w /working/map_{opts.submap} "
                      f"-v {model_dir.absolute()}:/working "
                      "furbrain/cyclops_mvs:latest make_refined.sh")
        else:
            os.system(f"docker run --user $(id -u):$(id -g) -w /working/map_{opts.submap} "
                      f"-v {model_dir.absolute()}:/working "
                      "furbrain/cyclops_mvs:latest make_rough.sh")
