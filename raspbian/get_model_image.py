import argparse
import json
from pathlib import Path
from typing import Union, Set, Dict

from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
from ament_index_python import get_package_share_directory


typestore = get_typestore(Stores.LATEST)

ROOT_DIR = Path('/data/trips/')
BAG_NAME = "recording"




def find_msg_file(package_name: str, msg_name: str) -> Union[Path, None]:
    """
    Try to find the .msg file for package_name/msg_name in several likely places:
      - get_package_share_directory(pkg)/msg/<msg_name>.msg
      - the installed python package dir: <site-packages>/orb_slam3/msg/Atlas.msg
      - inside the python package module path (pkg.__file__) / msg/<msg>.msg
    Returns Path or None.
    """
    candidates = []

    # 1) try package share (standard in ROS 2)
    share = Path(get_package_share_directory(package_name))
    candidates.append(share / "msg" / f"{msg_name}.msg")
    candidates.append(share / "msg" / f"{msg_name}.idl")  # sometimes IDL - not common

    # Deduplicate and test existence
    seen = set()
    for c in candidates:
        if c is None:
            continue
        p = Path(c)
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


parser = argparse.ArgumentParser(description="Create a colmap from a bag (ROS1 or ROS2)")
parser.add_argument('-n', '--name', help="Name of Recording")
opts = parser.parse_args()

model_dir = ROOT_DIR / opts.name

def get_last_atlas():
    with Reader(model_dir / BAG_NAME) as reader:
        atlas_topic = "/orb/ORB/atlas"
        last_atlas = None
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == atlas_topic:
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                last_atlas = msg
                last_ts = timestamp
        if last_atlas is None:
            raise RuntimeError(f"No messages found on {atlas_topic} — cannot extract maps/points/frames")
    return last_atlas

setup_typestore()
atlas = get_last_atlas()
all_point_idxs: Dict[int, Set[int]] = {}
points = {x.id: x for x in atlas.points}

for i, map in enumerate(atlas.maps):
    all_point_idxs[i] = set()
    for kf in map.frames:
        all_point_idxs[i].update(x.point3d_id for x in kf.points)
best = max(all_point_idxs, key=lambda p: len(all_point_idxs[p]))
seen_points = set()
output_points = []
for frame in atlas.maps[best].frames:
    for kp in frame.points:
        if kp.point3d_id not in seen_points:
            seen_points.add(kp.point3d_id)
            output_points.append((points[kp.point3d_id].x, points[kp.point3d_id].y, points[kp.point3d_id].z))


