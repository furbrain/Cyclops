from itertools import product, combinations
from pathlib import Path

import numpy as np
import open3d as o3d
import cv2
from scipy.spatial.transform import Rotation as R

from SLAM.frames import Frame, FrameSet
from SLAM.keyframes import get_keyframes_from_source
from reader.LensCalibration import Lens
from TOF.depth_reader import DepthReader
from TOF.extractor import FILENAME, homography,START, FINISH

DIR = Path("/home/phil/footage/sofa")
POSE_FILE = DIR / "fragments" / "fragment.json"

LENS = Lens.load("/home/phil/Projects/Cyclops/reader/lens_params.npz")

def frame_generator():
    count = 0
    with DepthReader(FILENAME, homography) as reader:
        for tm, frame, image in reader:
            if frame is not None:
                if START < tm < FINISH:
                    if count % 2 == 0:
                        yield count // 2, LENS.undistort_iamge(image)
                        # frame = Frame(DIR, image)
                        # frame.compute_kps()
                        # if last_frame is not None:
                        #     new, old, distance = frame.compare(last_frame)
                        #     E, mask  = cv2.findEssentialMat(new, old,lens.new_K)
                        #     results = cv2.recoverPose(E, new, old, lens.new_K, distanceThresh=100)
                        #     print(results)
                        #     break
                        # last_frame = frame
                    count += 1

def get_projection(node: o3d.pipelines.registration.PoseGraphNode, lens:Lens):
    return lens.new_K @ np.linalg.inv(node.pose)[:3,:]

def get_relative_rotation(a: np.ndarray, b:np.ndarray):
    # b = x @ a
    # x = b @ a.inv
    a = a[:3,:3]
    b = b[:3,:3]
    return b @ np.linalg.inv(a)


#key_frames = get_keyframes_from_source(frame_generator(),True, DIR)
key_frames = FrameSet(DIR)
key_frames.find_frames()
key_frames.load_images()
key_frames.load_descs()
pose_graph = o3d.io.read_pose_graph(str(POSE_FILE))
indices = list(key_frames.frames.keys())
projections = {x: get_projection(pose_graph.nodes[x], LENS) for x in indices}
pts_3d = []
colors = []
for (a_tm,a_frm), (b_tm, b_frm) in combinations(key_frames.frames.items(),2):
    a_pts, b_pts, distance = a_frm.compare(b_frm)
    if len(a_pts)>15:
        mat, mask = cv2.findEssentialMat(a_pts, b_pts, LENS.new_K)
        retvl, rot, t, mask = cv2.recoverPose(mat, a_pts, b_pts)
        rel = get_relative_rotation(pose_graph.nodes[a_tm].pose, pose_graph.nodes[b_tm].pose)
        rel = np.rad2deg(R.from_matrix(rel).as_euler("XYZ"))
        rot = np.rad2deg(R.from_matrix(rot).as_euler("XYZ"))
        print(f"{a_tm} -> {b_tm}: {len(a_pts)}, {a_pts.shape}, {b_pts.shape}")
        print(f"{a_tm} -> {b_tm}: Matrix: {rel}\nSecond:\n{rot}")
        pts_4d = cv2.triangulatePoints(projections[a_tm], projections[b_tm], a_pts.T, b_pts.T)
        pts_3d.extend(np.squeeze(cv2.convertPointsFromHomogeneous(pts_4d.T)))
        ix = a_pts.astype("int").T
        colors.extend(a_frm.orig_img[ix[1],ix[0]])
pts_3d = np.array(pts_3d)
colors = np.array(colors) / 255
norms = np.linalg.norm(pts_3d, axis=1)
#print(norms)
#print(pts_3d)
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(pts_3d[norms<3])
pcd.colors = o3d.utility.Vector3dVector(colors[norms<3])

o3d.visualization.draw_geometries([pcd])
