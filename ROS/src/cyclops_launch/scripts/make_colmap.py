#!/usr/bin/env python3
import argparse
import rosbag
import rospy
import os

import pathlib
from collections import defaultdict
import cv_bridge
import cv2

bridge  = cv_bridge.CvBridge()

parser = argparse.ArgumentParser(description="A program to create a colmap from a bag")
parser.add_argument('-b', '--bag', help="Bag file to search for images", required=True)
parser.add_argument('-o', '--output_dir', help="Location to put colmap models", default=pathlib.Path.cwd())
parser.add_argument('-r', '--rectified', help="Set to True if images are already rectified", default=False)
parser.add_argument('-i', '--image_topic', help="The image topics to save", default="/camera/image_rect")
opts = parser.parse_args()


root_dir = pathlib.Path(opts.output_dir)
cam_filename = "cameras.txt"
image_filename  = "images.txt"
points_filename = "points3D.txt"
cam_file = root_dir / cam_filename
images_dir = root_dir / "images"
images_dir.mkdir(parents=True, exist_ok=True)

def as_usec(tm: rospy.Time):
    nsec = tm.to_nsec()
    return nsec // 1000

def export_map(mp, points, map_dir):
    sparse_dir = map_dir / "sparse"
    sparse_dir.mkdir(parents=True, exist_ok=True)
    os.symlink("../images", map_dir / "images")
    os.symlink("../../"+ cam_filename, sparse_dir / cam_filename)
    required_points = defaultdict(list)
    frame_tss = set()
    with open(sparse_dir / image_filename, "w") as image_f: 
        for f in mp.frames:
            frame_tss.add(as_usec(f.stamp))
            o = f.pose.orientation
            t = f.pose.position
            image_f.write(f"{f.id} {o.w} {o.x} {o.y} {o.z} {t.x} {t.y} {t.z} 1 {as_usec(f.stamp)}.jpg\n")
            line = []
            for idx, pt in enumerate(f.points):
                required_points[pt.point3d_id].append(f"{f.id} {idx}")
                line.append(f"{pt.x} {pt.y} {pt.point3d_id}")
            image_f.write(" ".join(line)+"\n")
    with open(sparse_dir / points_filename, "w") as points_f:
        for pt in points:
            if pt.id in required_points:
                track = " ".join(required_points[pt.id])
                points_f.write(f"{pt.id} {pt.x} {pt.y} {pt.z} 255 255 255 0 {track}\n")
    return frame_tss  
        
def add_camera(bag):
    message_reader = bag.read_messages()
    frame_ts_set = set()
    for topic, msg, t in message_reader:
        if topic.endswith("camera_info"):
            with open(cam_file, "w") as f:
                if opts.rectified:
                    f.write(f"1 PINHOLE {msg.width} {msg.height} {msg.K[0]} {msg.K[4]} {msg.K[2]} {msg.K[5]}")
                else:
                    f.write(f"1 RADIAL {msg.width} {msg.height} {msg.K[0]} {msg.K[4]} {msg.K[2]} {msg.K[5]} {msg.D[0]} {msg.D[1]} {msg.D[2]} {msg.D[3]}")
            return

def export_images(bag, tss):
    message_reader = bag.read_messages([opts.image_topic])
    for _, msg, _ in message_reader:
        if as_usec(msg.header.stamp) in tss:
            path = str(images_dir/ f"{as_usec(msg.header.stamp)}.jpg")
            tss.remove(as_usec(msg.header.stamp))
            if msg._type=='sensor_msgs/Image':
                img = bridge.imgmsg_to_cv2(msg)
                cv2.imwrite(path, img)
            elif msg._type=='sensor_msgs/CompressedImage':
                if msg.format=="jpeg": #don't extract to image and then re-encode to jpeg - just save
                    with open(path, "wb") as f:
                        f.write(msg.data)
                else:
                    img = bridge.compressed_imgmsg_to_cv2(msg)
                    cv2.imwrite(path, img)                    
            else:
                raise ValueError(f"Unknown image type: {msg._type}")

with rosbag.Bag(opts.bag,"r") as in_bag:
    #find last time
    add_camera(in_bag)
    message_reader = in_bag.read_messages(["/orb_slam3/atlas"], raw=True)
    for _, _, t in message_reader:
        last_t = t
    message_reader = in_bag.read_messages(["/orb_slam3/atlas"], start_time=last_t)
    atlas = None
    for _, msg, _ in message_reader:
        atlas = msg

    frame_tss = set()
    maps = list(sorted(msg.maps, key=lambda x: len(x.frames), reverse=True))
    for map_idx, mp in enumerate(maps, start=1):
        map_dir = root_dir / f"map_{map_idx}"
        tss = export_map(mp, msg.points, map_dir)
        frame_tss = frame_tss.union(tss)
    print(len(frame_tss))
    export_images(in_bag, frame_tss)
    print(len(frame_tss))
    print("missing images: ", frame_tss)

