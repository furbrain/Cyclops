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
from typing import Dict, Tuple, Any, Union, Set, List
import numpy as np
import cv2
import yaml
import scipy.spatial.transform as sst

# rosbags high-level reader
from rosbags.rosbag2 import Reader

from utils import ROOT_DIR, IMAGE_TOPIC, IMAGE_TOPIC_RIGHT, BAG_NAME, CAM_FILENAME, IMAGE_FILENAME, POINTS_FILENAME, \
    TS_MAP_FILENAME, SPARSE_POINTS_FILENAME, bridge, as_usec_from_stamp, \
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
images_right_dir = model_dir / "right"
images_right_dir.mkdir(parents=True, exist_ok=True)



class Camera:
    def __init__(self, camera_info, alpha: float = 0.2):
        self.camera_info = camera_info
        self.K = np.array(camera_info.k).reshape(3, 3)
        self.D = np.array(camera_info.d)
        self.h, self.w = camera_info.height, camera_info.width
        #self.new_K, _ = cv2.getOptimalNewCameraMatrix(self.K, self.D, (self.w, self.h), alpha=alpha)
        self.new_K = self.K
        #print(self.K, self.D, self.h, self.w, self.new_K)
        self.map1, self.map2 = cv2.initUndistortRectifyMap(self.K, self.D[:4], None, self.new_K, (self.w, self.h),
                                                           cv2.CV_16SC2)
        self.point_transform = self.new_K @ np.linalg.inv(self.K)

    def undistort_image(self, image:np.ndarray) -> np.ndarray:
        return cv2.remap(image, self.map1, self.map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    def transform_point(self, point):
        p = self.point_transform @ np.array([point.x, point.y, 1.0])
        point.x = p[0]
        point.y = p[1]

    def get_camera_desc(self):
        return f"PINHOLE {self.w} {self.h} {self.new_K[0,0]} {self.new_K[1,1]} {self.new_K[0,2]} {self.new_K[1,2]}"

def quaternion_to_rotation_matrix(q):
    """Convert quaternion [x, y, z, w] to 3x3 rotation matrix."""
    x, y, z, w = q
    R = np.array([
        [1 - 2*(y**2 + z**2),     2*(x*y - z*w),         2*(x*z + y*w)],
        [    2*(x*y + z*w),     1 - 2*(x**2 + z**2),     2*(y*z - x*w)],
        [    2*(x*z - y*w),         2*(y*z + x*w),     1 - 2*(x**2 + y**2)]
    ])
    return R

def load_stereo_extrinsics(yaml_path):
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    t = data['translation']
    T = np.array([t['x'], t['y'], t['z']])

    r = data['rotation']
    q = np.array([r['x'], r['y'], r['z'], r['w']])  # xyzw convention
    R_mat = quaternion_to_rotation_matrix(q)

    return R_mat, T


def rectify_cam_pair(left:Camera, right: Camera):
    try:
        R, T = load_stereo_extrinsics("/home/pi/.ros/transforms/left_right.yaml")
    except IOError:
        R, T = load_stereo_extrinsics(model_dir / "left_right.yaml")

    img_size = left.w, left.h
    R1, R2, P1, P2, Q, roi_l, roi_r = cv2.stereoRectify(
        left.K, left.D,
        right.K, right.D,
        img_size,
        R, T,
        flags=cv2.CALIB_ZERO_DISPARITY,  # principal points aligned
    )
    left.map1, left.map2 = cv2.initUndistortRectifyMap(left.K, left.D[:4], R1, P1, img_size, cv2.CV_16SC2)
    right.map1, right.map2 = cv2.initUndistortRectifyMap(right.K, right.D[:4], R2, P2, img_size, cv2.CV_16SC2)
    left.new_K = P1[:3, :3]
    right.new_K = P2[:3, :3]


def export_map(mp, points: Dict, map_dir: pathlib.Path, known_images: Dict[int, int], camera:Camera):
    sparse_dir = init_map_dir(map_dir)

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


def get_camera_from_bag(reader, topic_name: str):
    """
    Search for a CameraInfo message in the bag and write cameras.txt
    Uses reader.messages() and reader.deserialize() for convenience.
    """
    for connection, timestamp, rawdata in reader.messages():
        topic = connection.topic
        # accept camera_info topic names ending with camera_info
        if topic==topic_name:
            msg = deserialize(rawdata, "sensor_msgs/msg/CameraInfo")
            cam = Camera(msg)
            return True, cam
    return False, None

def export_images_from_bag(reader: Reader, tss, image_topic, camera:Camera, pth: pathlib.Path = None, right=False):
    """
    reader: AnyReader
    tss: set of expected timestamps in usec
    image_topic: topic string to filter
    """
    # iterate messages on image_topic
    if pth is None:
        pth = images_dir
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
            if right:
                path = str(pth / f"{stamp_usec}r.jpg")
            else:
                path = str(pth / f"{stamp_usec}.jpg")
            print(f"writing image: {path}")
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
                    if ("jpeg" in fmt.lower()) and False:
                        # save raw bytes
                        with open(path, "wb") as f:
                            f.write(bytes(msg.data))
                    else:
                        try:
                            img = bridge.compressed_imgmsg_to_cv2(msg)
                            img = camera.undistort_image(img)
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

def create_colmap_from_atlas(url: str):
    import orb_slam3_py
    bag_path = model_dir / BAG_NAME
    if not (bag_path / "metadata.yaml").exists():
        os.system(f"ros2 bag reindex {bag_path}")
    known_images, known_right_images = get_known_images(bag_path)
    left_camera, right_camera = get_cameras(bag_path)
    atlas = orb_slam3_py.load_atlas(url)
    orb_slam3_py.align_atlas(atlas)
    #sort maps
    maps = reversed(sorted(atlas.get_all_maps(), key=lambda x:x.keyframes_in_map()))
    good_ts: Set[orb_slam3_py.KeyFrame] = set()
    right_ts: Set[orb_slam3_py.KeyFrame] = set()
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
                    good_ts.add(stamp_usec)
                    pose: sst.RigidTransform = kf.get_pose()
                    o = pose.rotation.as_quat(scalar_first=True)
                    t: np.array = pose.translation
                    f.write(f"{kf.id} {o[0]} {o[1]} {o[2]} {o[3]} {t[0]} {t[1]} {t[2]} 1 {stamp_usec}.jpg\n")
                    line = []
                    idx = 0
                    for mp, (x, y) in zip(kf.get_map_point_matches(), kf.get_keypoints_undistorted()):
                        if mp:
                            line.append(f"{x} {y} {mp.id}")
                            found_mps[mp].append(f"{kf.id} {idx}")
                            idx += 1
                    f.write(" ".join(line) + "\n")
                if stamp_usec in known_right_images:
                    right_ts.add(stamp_usec)
                    pose: sst.RigidTransform = kf.get_pose()
                    T_left_right = sst.RigidTransform.from_translation([kf.baseline, 0, 0])
                    pose = pose * T_left_right
                    o = pose.rotation.as_quat(scalar_first=True)
                    t: np.array = pose.translation
                    f.write(f"{kf.id + right_id_offset} {o[0]} {o[1]} {o[2]} {o[3]} {t[0]} {t[1]} {t[2]} 1 {stamp_usec}r.jpg\n")
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
        export_images_from_bag(reader, good_ts, IMAGE_TOPIC, left_camera)
        export_images_from_bag(reader, right_ts, IMAGE_TOPIC_RIGHT, right_camera, right=True)
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
    frame_tss = set()
    points_dict = {x.id: x for x in last_atlas.points}
    for map_idx, mp in enumerate(maps, start=1):
        map_dir = model_dir / f"map_{map_idx}"
        map_dir.mkdir(parents=True, exist_ok=True)
        tss = export_map(mp, points_dict, map_dir, known_images, left_camera)
        frame_tss = frame_tss.union(tss)

    print(f"Need to extract {len(frame_tss)} images")
    # Re-open reader to iterate through images (AnyReader supports re-iterating)
    # export images
    with Reader(bag_path) as reader:
        right_frame_tss = frame_tss.copy()
        export_images_from_bag(reader, frame_tss, IMAGE_TOPIC, left_camera)
        export_images_from_bag(reader, right_frame_tss, IMAGE_TOPIC_RIGHT, right_camera, images_right_dir)
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


def get_cameras(bag_path: Union[Path, Any]) -> Tuple[Any, Any]:
    with Reader(bag_path) as reader:
        # 1) try to find camera info and write cameras.txt
        ok, left_camera = get_camera_from_bag(reader, "/left/camera_info")
        if not ok:
            print("Warning: camera_info not found in bag. cameras.txt will not be created or may be incomplete.")
        ok, right_camera = get_camera_from_bag(reader, "/right/camera_info")
        if not ok:
            print("Warning: camera_info not found in bag. cameras.txt will not be created or may be incomplete.")
        rectify_cam_pair(left_camera, right_camera)  # rectify cameras
        with open(cam_file, "w") as f:
            # PINHOLE requires fx fy cx cy
            # MUST be rectified
            f.write(f"1 {left_camera.get_camera_desc()}\n")
            f.write(f"2 {right_camera.get_camera_desc()}\n")

        # 2) find the last atlas message on /orb_slam3/atlas (mimics original script behavior)
        # We'll find all messages on that topic then pick last
    return left_camera, right_camera


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
        if opts.atlas and opts.atlas.endswith("osa"):
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
            os.system(f"docker run {args} -w /working/map_{opts.submap} "
                      f"-v {model_dir.absolute()}:/working "
                      f"{image} {prefix}make_rough.sh")

