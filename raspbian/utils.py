import json
import os
import pathlib
from pathlib import Path
from typing import Union, Dict, Optional, Any, Tuple

import cv2
import numpy as np
import yaml
from rosbags.interfaces import Connection
from rosbags.rosbag2 import Reader
from rosbags.typesys import get_typestore, Stores, get_types_from_msg
from rosbags.typesys.store import Typestore

import cv_bridge


ROOT_DIR = Path('/data/trips/')
IMAGE_TOPIC = "/orb/ORB/keyframes/compressed"
IMAGE_TOPIC_RIGHT = "/orb/ORB/keyframes_right/compressed"
BAG_NAME = "recording"
CAM_FILENAME = "cameras.txt"
IMAGE_FILENAME = "images.txt"
POINTS_FILENAME = "points3D.txt"
TS_MAP_FILENAME = "ts_map.json"
SPARSE_POINTS_FILENAME = "sparse_points.json"
_typestore: Optional[Typestore] = None
bridge = cv_bridge.CvBridge()


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
    if isinstance(stamp, float):
        return int(stamp * 1_000_000)
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
    try:
        from ament_index_python import get_package_share_directory
        from ament_index_python.packages import PackageNotFoundError
        try:
            share = pathlib.Path(get_package_share_directory(package_name))
        except PackageNotFoundError:
            raise IOError(f"Package {package_name} not found")
        candidates.append(share / "msg" / f"{msg_name}.msg")
        candidates.append(share / "msg" / f"{msg_name}.idl")  # sometimes IDL - not common
    except (ImportError, IOError):
        p = Path(__file__).parent.parent / "ROS2"/ "orb_slam3_ros" / "msg"/ f"{msg_name}.msg"
        print(p)
        if p.exists():
            return p
        else:
            return None
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
    global _typestore
    _typestore = get_typestore(Stores.LATEST)
    custom_msgs = {}
    for tp in ('Atlas', 'Map', 'KeyFrame', 'KeyPoint', 'Point3D'):
        msg = find_msg_file("orb_slam3", tp)
        tp_data = get_types_from_msg(msg.read_text(), f"orb_slam3/msg/{tp}")
        custom_msgs.update(tp_data)
    _typestore.register(custom_msgs)


def get_image_from_timestamp(reader: Reader, ts_map: Dict[int, int], timestamp) -> np.ndarray:
    data = get_raw_image_from_timestamp(reader, ts_map, timestamp)
    img_data = bridge.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")
    return img_data


def get_raw_image_from_timestamp(reader: Reader, ts_map: Dict[int, int], timestamp):
    bag_ts = ts_map.get(timestamp)
    if bag_ts is None:
        raise ValueError(f"Timestamp {timestamp} not found in map")
    connections = [x for x in reader.connections if x.topic == IMAGE_TOPIC]
    for connection, msg_timestamp, rawdata in reader.messages(connections=connections, start=bag_ts - 1000):
        data = deserialize(rawdata, connection.msgtype)
        data_ts = as_usec_from_stamp(data.header.stamp)
        if timestamp == data_ts:
            return data
        elif data_ts > timestamp + 1_000_000:
            raise ValueError(f"Mismatch between timestamp {timestamp} and {data_ts}")
    raise ValueError(f"No images found")

def load_ts_map(model: str) -> Dict[int, int]:

    ts_map_path = ROOT_DIR / model / TS_MAP_FILENAME
    if not ts_map_path.exists():
        os.system(f"/home/pi/Cyclops/raspbian/make_model.py -p -n {model}")
    with open(ts_map_path) as f:
        tmp_dict = json.load(f)
    frame_dict = {int(x):y for x,y in tmp_dict.items()}
    return frame_dict

def deserialize(rawdata: bytes, msg_type: str) -> object:
    if _typestore is None:
        setup_typestore()
    return _typestore.deserialize_cdr(rawdata, msg_type)


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


def rectify_cam_pair(left:Camera, right: Camera, model_dir: Path):
    try:
        R, T = load_stereo_extrinsics("/home/pi/.ros/transforms/left_right.yaml")
    except IOError:
        R, T = load_stereo_extrinsics(model_dir / "left_right.yaml")

    # note the following is swapped - cv2 expects rotations and translations as Trl, not Tlr
    img_size = left.w, left.h
    R2, R1, P2, P1, Q, roi_r, roi_l = cv2.stereoRectify(
        right.K, right.D,
        left.K, left.D,
        img_size,
        R, T,
        flags=cv2.CALIB_ZERO_DISPARITY,  # principal points aligned
    )
    left.map1, left.map2 = cv2.initUndistortRectifyMap(left.K, left.D, R1, P1, img_size, cv2.CV_16SC2)
    right.map1, right.map2 = cv2.initUndistortRectifyMap(right.K, right.D, R2, P2, img_size, cv2.CV_16SC2)
    left.new_K = P1[:3, :3]
    right.new_K = P2[:3, :3]


def get_cameras(bag_path: Union[Path, Any], model_dir: Path, cam_file: Path) -> Tuple[Any, Any]:
    with Reader(bag_path) as reader:
        # 1) try to find camera info and write cameras.txt
        ok, left_camera = get_camera_from_bag(reader, "/left/camera_info")
        if not ok:
            print("Warning: camera_info not found in bag. cameras.txt will not be created or may be incomplete.")
        ok, right_camera = get_camera_from_bag(reader, "/right/camera_info")
        if not ok:
            print("Warning: camera_info not found in bag. cameras.txt will not be created or may be incomplete.")
        rectify_cam_pair(left_camera, right_camera, model_dir)  # rectify cameras
        if cam_file:
            with open(cam_file, "w") as f:
                # PINHOLE requires fx fy cx cy
                # MUST be rectified
                f.write(f"1 {left_camera.get_camera_desc()}\n")
                f.write(f"2 {right_camera.get_camera_desc()}\n")

        # 2) find the last atlas message on /orb_slam3/atlas (mimics original script behavior)
        # We'll find all messages on that topic then pick last
    return left_camera, right_camera


def export_images_from_bag(reader: Reader, tss: Dict[int, Any], image_topic, camera: Camera, pth: pathlib.Path,
                           right=False):
    """
    reader: AnyReader
    tss: set of expected timestamps in usec
    image_topic: topic string to filter
    """
    # iterate messages on image_topic
    pth.mkdir(parents=True, exist_ok=True)
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
                path = str(pth / f"{tss[stamp_usec].id}r.jpg")
            else:
                path = str(pth / f"{tss[stamp_usec].id}.jpg")
            print(f"writing image: {path}")
            # remove from set so we don't write duplicates
            # Should be sensor_msgs/CompressedImage
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
