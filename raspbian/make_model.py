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
import platform
import sys
from collections import defaultdict
from pathlib import Path
from typing import Dict, Tuple, Any, Union, List
import numpy as np
import scipy.spatial.transform as sst

# rosbags high-level reader
from rosbags.rosbag2 import Reader

from utils import Camera, get_cameras, export_images_from_bag
from utils import ROOT_DIR, IMAGE_TOPIC, IMAGE_TOPIC_RIGHT, BAG_NAME, CAM_FILENAME, IMAGE_FILENAME, POINTS_FILENAME, \
    TS_MAP_FILENAME, SPARSE_POINTS_FILENAME, as_usec_from_stamp, \
    get_image_from_timestamp, deserialize


parser = argparse.ArgumentParser(description="Create a colmap from a bag (ROS1 or ROS2)")
location = parser.add_mutually_exclusive_group(required=True)
location.add_argument('-n', '--name', help="Name of Recording")
location.add_argument('-d', '--dir', help="Recording directory")
parser.add_argument('-r', '--refined', help="Create a refined model, rather than rough", action="store_true")
parser.add_argument('-s', '--submap', help="Submap to create", default=1, type=int)
parser.add_argument('-p', '--prep-only', help="Just create data for colmap and indices", action="store_true")
parser.add_argument('-a', '--atlas', help="provide an atlas message file to use", default="")
opts = parser.parse_args()

if opts.dir:
    model_dir = pathlib.Path(opts.dir)
else:
    model_dir = ROOT_DIR / opts.name
cam_file = model_dir / CAM_FILENAME
images_dir = model_dir / "images"
images_dir.mkdir(parents=True, exist_ok=True)


def export_map(mp, points: Dict, map_dir: pathlib.Path, known_images: Dict[int, int], camera: Camera):
    sparse_dir = init_map_dir(map_dir)

    required_points = defaultdict(list)
    frame_tss: Dict[int, int] = {}
    sparse_points: Dict[int, Tuple] = {}
    with open(sparse_dir / IMAGE_FILENAME, "w") as image_f:
        for f in mp.frames:
            stamp_usec = as_usec_from_stamp(f.stamp)
            if stamp_usec not in known_images:
                print(f"Image {stamp_usec} not in bag ... skipping")
                continue
            frame_tss[stamp_usec] = f
            o = f.pose.orientation
            t = f.pose.position
            image_f.write(f"{f.id} {o.w} {o.x} {o.y} {o.z} {t.x} {t.y} {t.z} 1 {f.id}.jpg\n")
            line = []
            bag_path = model_dir / BAG_NAME
            with Reader(bag_path) as reader:
                image = get_image_from_timestamp(reader, known_images, stamp_usec)
            for idx, pt in enumerate(f.points):
                #camera.transform_point(pt)
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


def init_map_dir(map_dir: Path) -> Path:
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
    return sparse_dir


def get_available_images(reader: Reader, topic: str) -> Dict[int, int]:
    tss: Dict[int, int] = {}
    for connection, timestamp, rawdata in reader.messages():
        if connection.topic == topic:
            msg = deserialize(rawdata, connection.msgtype)
            ts = as_usec_from_stamp(msg.header.stamp)
            tss[ts] = timestamp
    return tss

def create_colmap_from_atlas(url: str):
    import orb_slam3_py
    binary = url.endswith(".osa")
    bag_path = model_dir / BAG_NAME
    if not (bag_path / "metadata.yaml").exists():
        os.system(f"ros2 bag reindex {bag_path}")
    known_images, known_right_images = get_known_images(bag_path)
    left_camera, right_camera = get_cameras(bag_path, model_dir, cam_file)
    atlas = orb_slam3_py.load_atlas(url, binary=binary)
    #orb_slam3_py.align_atlas(atlas)
    #sort maps
    maps = reversed(sorted(atlas.get_all_maps(), key=lambda x:x.keyframes_in_map()))
    good_ts: Dict[int, orb_slam3_py.KeyFrame] = dict()
    right_ts: Dict[int, orb_slam3_py.KeyFrame] = dict()
    for map_idx, m in enumerate(maps, start=1):
        map_dir = model_dir / f"map_{map_idx}"
        map_dir.mkdir(parents=True, exist_ok=True)
        sparse_dir = init_map_dir(map_dir)
        found_mps: Dict[orb_slam3_py.MapPoint, List[str]] = defaultdict(list)
        right_id_offset = max(x.id for x in m.get_all_keyframes())+1
        with open(sparse_dir / IMAGE_FILENAME, "w") as f:
            for kf in m.get_all_keyframes():
                stamp_usec = as_usec_from_stamp(kf.timestamp)
                if stamp_usec in known_images:
                    good_ts[stamp_usec] = kf
                    pose: sst.RigidTransform = kf.get_pose()
                    o = pose.rotation.as_quat(scalar_first=True)
                    t: np.array = pose.translation
                    f.write(f"{kf.id} {o[0]} {o[1]} {o[2]} {o[3]} {t[0]} {t[1]} {t[2]} 1 {kf.id}.jpg\n")
                    line = []
                    idx = 0
                    for mp, (x, y) in zip(kf.get_map_point_matches(), kf.get_keypoints_undistorted()):
                        if mp:
                            line.append(f"{x} {y} {mp.id}")
                            found_mps[mp].append(f"{kf.id} {idx}")
                            idx += 1
                    f.write(" ".join(line) + "\n")
                if stamp_usec in known_right_images:
                    right_ts[stamp_usec] = kf
                    pose: sst.RigidTransform = kf.get_pose()
                    T_left_right = sst.RigidTransform.from_translation([kf.baseline, 0, 0])
                    pose = pose * T_left_right
                    o = pose.rotation.as_quat(scalar_first=True)
                    t: np.array = pose.translation
                    f.write(f"{kf.id + right_id_offset} {o[0]} {o[1]} {o[2]} {o[3]} {t[0]} {t[1]} {t[2]} 1 {kf.id}r.jpg\n")
                    line = []
                    idx = 0
                    for mp, (_, y), x in zip(kf.get_map_point_matches(), kf.get_keypoints_undistorted(), kf.get_u_right()):
                        if mp and (x>=0):
                            line.append(f"{x} {y} {mp.id}")
                            found_mps[mp].append(f"{kf.id + right_id_offset} {idx}")
                            idx += 1
                    f.write(" ".join(line) + "\n")
        with open(sparse_dir / POINTS_FILENAME, "w") as points_f:
            for mp, track in found_mps.items():
                pt = mp.get_world_pos()
                points_f.write(f"{mp.id} {pt[0]} {pt[1]} {pt[2]} 255 255 255 0 {' '.join(track)}\n")
    with Reader(bag_path) as reader:
        export_images_from_bag(reader, good_ts, IMAGE_TOPIC, left_camera, images_dir)
        export_images_from_bag(reader, right_ts, IMAGE_TOPIC_RIGHT, right_camera, images_dir, right=True)
        if len(good_ts) > 0:
            print("missing images: ", good_ts)
        else:
            print("All images exported.")
        if len(right_ts) > 0:
            print("missing right images: ", right_ts)
        else:
            print("All images exported.")


def create_colmap():
    bag_path = model_dir / BAG_NAME
    if not (bag_path / "metadata.yaml").exists():
        os.system(f"ros2 bag reindex {bag_path}")
    known_images, known_right_images = get_known_images(bag_path)
    left_camera, right_camera = get_cameras(bag_path)
    with Reader(bag_path) as reader:
        if opts.atlas:
            with open(opts.atlas, "rb") as f:
                atlas_data = f.read()
        else:
            atlas_topic = "/orb/ORB/atlas"
            atlas_data = None
            for connection, timestamp, rawdata in reader.messages():
                if connection.topic == atlas_topic:
                    atlas_data = rawdata
            if atlas_data is None:
                raise RuntimeError(f"No messages found on {atlas_topic} — cannot extract maps/points/frames")

        last_atlas = deserialize(atlas_data, 'orb_slam3/msg/Atlas')

    # The original script sorted maps by their frame count and exported each.
    # We'll follow same logic. The fields expected on atlas message are 'maps' and 'points'.
    # These are user-defined types produced by ORB_SLAM3 - we assume the structure used in your ROS1 bag.
    maps = list(sorted(last_atlas.maps, key=lambda x: len(x.frames), reverse=True))
    print(f"found {len(maps)} maps")
    frame_tss = {}
    points_dict = {x.id: x for x in last_atlas.points}
    for map_idx, mp in enumerate(maps, start=1):
        map_dir = model_dir / f"map_{map_idx}"
        map_dir.mkdir(parents=True, exist_ok=True)
        tss = export_map(mp, points_dict, map_dir, known_images, left_camera)
        frame_tss.update(tss)

    print(f"Need to extract {len(frame_tss)} images")
    # Re-open reader to iterate through images (AnyReader supports re-iterating)
    # export images
    with Reader(bag_path) as reader:
        right_frame_tss = frame_tss.copy()
        export_images_from_bag(reader, frame_tss, IMAGE_TOPIC, left_camera, images_dir)
        export_images_from_bag(reader, right_frame_tss, IMAGE_TOPIC_RIGHT, right_camera, images_dir, right=True)
        if len(frame_tss) > 0:
            print("missing images: ", frame_tss)
        else:
            print("All images exported.")
    if len(right_frame_tss) > 0:
        print("missing right images: ", right_frame_tss)
    else:
        print("All right images exported.")
    return known_images


def get_known_images(bag_path: Union[Path, Any]) -> Tuple[Dict[int, int], Dict[int, int]]:
    with Reader(bag_path) as reader:
        known_images = get_available_images(reader, IMAGE_TOPIC)
    with Reader(bag_path) as reader:
        known_right_images = get_available_images(reader, IMAGE_TOPIC_RIGHT)
    return known_images, known_right_images


def get_cuda_args():
    CUDA_RUNTIME_ARGS = "--gpus all -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics "

    CUDA_CONTAINER_SUFFIX = "-cuda"

    # no need to do `xhost +` anymore
    XSOCK = "/tmp/.X11-unix"
    XAUTH = "/tmp/.docker.xauth"
    DISPLAY_ARGS = (f"--volume={XSOCK}:{XSOCK}:rw --volume={XAUTH}:{XAUTH}:rw --env=XAUTHORITY={XAUTH} "
                    f"--env=DISPLAY=unix:1")
    args = CUDA_RUNTIME_ARGS + DISPLAY_ARGS + ' --ipc=host --shm-size=4gb'
    return args, 'openmvs-ubuntu-cuda', '/working/'

def get_pi_args():
    return "--user $(id -u):$(id -g)", "furbrain/cyclops_mvs:latest", ""

if __name__ == "__main__":
    if not cam_file.exists():
        if opts.atlas and (opts.atlas.endswith("osa") or opts.atlas.endswith("txt.gz")):
            create_colmap_from_atlas(opts.atlas)
        else:
            ts_map = create_colmap()
            json.dump(ts_map, open(model_dir / TS_MAP_FILENAME, "w"))
    else:
        print("COLMAP data already exists\n")
    sys.stdout.flush()
    if platform.processor()=="x86_64":
        args, image, prefix = get_cuda_args()
    else:
        args, image, prefix = get_pi_args()
    # os.system("docker pull furbrain/cyclops_mvs:latest")
    if not opts.prep_only:
        if opts.refined:
            os.system(f"docker run {args} -w /working/map_{opts.submap} "
                      f"-v {model_dir.absolute()}:/working "
                      f"{image} {prefix}make_refined.sh")
        else:
            cmd_line = (f"docker run {args} "
                      f"-v {model_dir.absolute()}:/working -w /working/map_{opts.submap} "
                      f"{image} {prefix}make_rough.sh")
            print(cmd_line)
            os.system(cmd_line)
