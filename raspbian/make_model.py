#!/usr/bin/env python3
"""
colmap_from_bag_ros2.py

Convert a ROS1 or ROS2 bag into files usable by COLMAP (cameras.txt, images.txt, points3D.txt)
and extract images into an images/ directory.

Requires: rosbags (pip install rosbags), cv_bridge, opencv-python (cv2)

It then uses docker to create a rough or refined model
"""

import argparse
import os
import pathlib
import sys
import time
from collections import defaultdict
from typing import Union, Set
from pathlib import Path

import cv_bridge
import cv2

# rosbags high-level reader
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

from ament_index_python import get_package_share_directory

ROOT_DIR = Path('/data/trips/')
IMAGE_TOPIC = "/orb/ORB/keyframes/compressed"
BAG_NAME = "recording"
# Create a typestore and get the string class.
typestore = get_typestore(Stores.LATEST)

bridge = cv_bridge.CvBridge()

parser = argparse.ArgumentParser(description="Create a colmap from a bag (ROS1 or ROS2)")
parser.add_argument('-n', '--name', help="Name of Recording")
parser.add_argument('-r', '--refined', help="Create a refined model, rather than rough", action="store_true")
parser.add_argument('-s', '--submap', help="Submap to create", default=1, type=int)
opts = parser.parse_args()

model_dir = ROOT_DIR / opts.name
cam_filename = "cameras.txt"
image_filename = "images.txt"
points_filename = "points3D.txt"
cam_file = model_dir / cam_filename
images_dir = model_dir / "images"
images_dir.mkdir(parents=True, exist_ok=True)

def as_usec_from_stamp(stamp):
    """
    stamp: a message header stamp-like object or tuple
    we attempt to handle:
      - ROS2: object with .sec and .nanosec
      - ROS1: object with .secs and .nsecs
      - dict-like: {'sec':.., 'nanosec':..} or ('secs','nsecs')
      - integer timestamp in nanoseconds
    """
    # if stamp already integer (nanoseconds)
    if isinstance(stamp, int):
        return stamp // 1000
    # object styles
    sec = None
    nsec = None
    for a in ('sec', 'secs'):
        if hasattr(stamp, a):
            sec = getattr(stamp, a)
            break
        if isinstance(stamp, dict) and a in stamp:
            sec = stamp[a]
            break
    for a in ('nanosec', 'nsecs', 'nsec'):
        if hasattr(stamp, a):
            nsec = getattr(stamp, a)
            break
        if isinstance(stamp, dict) and a in stamp:
            nsec = stamp[a]
            break
    # fallback if stamp is tuple/list (sec, nsec)
    if sec is None and isinstance(stamp, (list, tuple)) and len(stamp) >= 2:
        sec, nsec = stamp[0], stamp[1]
    if sec is None:
        raise ValueError(f"Unsupported stamp type: {type(stamp)} - value: {stamp}")
    if nsec is None:
        nsec = 0
    return int(sec) * 1_000_000 + int(nsec) // 1000

def export_map(mp, points, map_dir: pathlib.Path, known_images: Set[int]):
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
        if not (sparse_dir / cam_filename).exists():
            os.symlink("../../" + cam_filename, sparse_dir / cam_filename)
    except Exception:
        pass

    required_points = defaultdict(list)
    frame_tss = set()
    with open(sparse_dir / image_filename, "w") as image_f:
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
            for idx, pt in enumerate(f.points):
                required_points[pt.point3d_id].append(f"{f.id} {idx}")
                line.append(f"{pt.x} {pt.y} {pt.point3d_id}")
            image_f.write(" ".join(line) + "\n")
    with open(sparse_dir / points_filename, "w") as points_f:
        for pt in points:
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
            msg = typestore.deserialize_cdr(rawdata, "sensor_msgs/msg/CameraInfo")
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
        msg = typestore.deserialize_cdr(rawdata, connection.msgtype)

        # attempt to get header stamp
        header = getattr(msg, "header", None)
        if header is None:
            # some compressed images may have stamp at top-level; try msg.stamp
            stamp = getattr(msg, "stamp", None)
        else:
            stamp = header.stamp
        try:
            stamp_usec = as_usec_from_stamp(stamp)
        except Exception:
            # try using the timestamp from the bag (timestamp provided by reader.messages())
            stamp_usec = int(timestamp) // 1000
            raise
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

def get_available_images(reader: Reader, topic: str) -> Set[int]:
    tss: Set[int] = set()
    for connection, timestamp, rawdata in reader.messages():
        if connection.topic == topic:
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            ts = as_usec_from_stamp(msg.header.stamp)
            tss.add(ts)
    return tss

def create_colmap():
    bag_path = model_dir / BAG_NAME
    if not (bag_path / "metadata.yaml").exists():
        os.system(f"ros2 bag reindex {bag_path}")
    setup_typestore()
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
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                last_atlas = msg
                last_ts = timestamp
        if last_atlas is None:
            raise RuntimeError(f"No messages found on {atlas_topic} — cannot extract maps/points/frames")

        # The original script sorted maps by their frame count and exported each.
        # We'll follow same logic. The fields expected on atlas message are 'maps' and 'points'.
        # These are user-defined types produced by ORB_SLAM3 - we assume the structure used in your ROS1 bag.
        maps = list(sorted(last_atlas.maps, key=lambda x: len(x.frames), reverse=True))
        frame_tss = set()
        for map_idx, mp in enumerate(maps, start=1):
            map_dir = model_dir / f"map_{map_idx}"
            map_dir.mkdir(parents=True, exist_ok=True)
            tss = export_map(mp, last_atlas.points, map_dir, known_images)
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

def find_msg_file(package_name: str, msg_name: str) -> Union[pathlib.Path, None]:
    """
    Try to find the .msg file for package_name/msg_name in several likely places:
      - get_package_share_directory(pkg)/msg/<msg_name>.msg
      - the installed python package dir: <site-packages>/orb_slam3/msg/Atlas.msg
      - inside the python package module path (pkg.__file__) / msg/<msg>.msg
    Returns Path or None.
    """
    candidates = []

    # 1) try package share (standard in ROS 2)
    share = pathlib.Path(get_package_share_directory(package_name))
    candidates.append(share / "msg" / f"{msg_name}.msg")
    candidates.append(share / "msg" / f"{msg_name}.idl")  # sometimes IDL - not common

    # Deduplicate and test existence
    seen = set()
    for c in candidates:
        if c is None:
            continue
        p = pathlib.Path(c)
        if str(p) in seen:
            continue
        seen.add(str(p))
        if p.exists() and p.is_file():
            return p

    # nothing found
    return None

def setup_typestore():
    custom_msgs = {}
    for tp in ('Atlas', 'Map', 'KeyFrame', 'KeyPoint', 'Point3D'):
        msg = find_msg_file("orb_slam3", tp)
        tp_data = get_types_from_msg(msg.read_text(), f"orb_slam3/msg/{tp}")
        custom_msgs.update(tp_data)
    typestore.register(custom_msgs)

if __name__ == "__main__":
    if not cam_file.exists():
        create_colmap()
    else:
        print("COLMAP data already exists\n")
    sys.stdout.flush()
    # os.system("docker pull furbrain/cyclops_mvs:latest")
    if opts.refined:
        os.system(f"docker run --user $(id -u):$(id -g) -w /working/map_{opts.submap} "
                  f"-v {model_dir.absolute()}:/working "
                  "furbrain/cyclops_mvs:latest make_refined.sh")
    else:
        os.system(f"docker run --user $(id -u):$(id -g) -w /working/map_{opts.submap} "
                  f"-v {model_dir.absolute()}:/working "
                  "furbrain/cyclops_mvs:latest make_rough.sh")
