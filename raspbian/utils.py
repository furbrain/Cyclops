import json
import os
import pathlib
from pathlib import Path
from typing import Union, Dict, Optional

import numpy as np
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
