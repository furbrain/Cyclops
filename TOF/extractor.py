import open3d as o3d
import cv2
import numpy as np
import math as maths

from TOF.depth_reader import DepthReader, TOF_INTRINSIC

FILENAME = "/home/phil/comb2.mkv"

START = 3.0
FINISH = 9

homography = np.load("homography.npy")


import math
import multiprocessing
import os, sys
import numpy as np
import open3d as o3d

from optimize_posegraph import optimize_posegraph_for_fragment

from opencv_pose_estimation import pose_estimation


def register_one_rgbd_pair(s, t, images, intrinsic,
                           with_opencv, config):
    source_rgbd_image = images[s]
    target_rgbd_image = images[t]

    option = o3d.pipelines.odometry.OdometryOption()
    option.depth_diff_max = config["depth_diff_max"]
    if abs(s - t) != 1:
        if with_opencv:
            success_5pt, odo_init = pose_estimation(source_rgbd_image,
                                                    target_rgbd_image,
                                                    intrinsic, False)
            if success_5pt:
                [success, trans, info
                ] = o3d.pipelines.odometry.compute_rgbd_odometry(
                    source_rgbd_image, target_rgbd_image, intrinsic, odo_init,
                    o3d.pipelines.odometry.RGBDOdometryJacobianFromHybridTerm(),
                    option)
                return [success, trans, info]
        return [False, np.identity(4), np.identity(6)]
    else:
        odo_init = np.identity(4)
        [success, trans, info] = o3d.pipelines.odometry.compute_rgbd_odometry(
            source_rgbd_image, target_rgbd_image, intrinsic, odo_init,
            o3d.pipelines.odometry.RGBDOdometryJacobianFromHybridTerm(), option)
        return [success, trans, info]


def make_posegraph_for_fragment(path_dataset, sid, eid, images,
                                intrinsic, with_opencv, config):
    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)
    pose_graph = o3d.pipelines.registration.PoseGraph()
    trans_odometry = np.identity(4)
    pose_graph.nodes.append(
        o3d.pipelines.registration.PoseGraphNode(trans_odometry))
    for s in range(sid, eid):
        for t in range(s + 1, eid):
            # odometry
            if t == s + 1:
                print(
                    "Fragment %03d / %03d :: RGBD matching between frame : %d and %d"
                    % (1, 1, s, t))
                [success, trans,
                 info] = register_one_rgbd_pair(s, t, images,
                                                intrinsic, with_opencv, config)
                trans_odometry = np.dot(trans, trans_odometry)
                trans_odometry_inv = np.linalg.inv(trans_odometry)
                pose_graph.nodes.append(
                    o3d.pipelines.registration.PoseGraphNode(
                        trans_odometry_inv))
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(s - sid,
                                                             t - sid,
                                                             trans,
                                                             info,
                                                             uncertain=False))

    o3d.io.write_pose_graph(
        os.path.join(path_dataset, config["template_fragment_posegraph"]),
        pose_graph)


def integrate_rgb_frames_for_fragment(images, pose_graph_name, intrinsic,
                                      config):
    pose_graph = o3d.io.read_pose_graph(pose_graph_name)
    volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=config["tsdf_cubic_size"] / 512.0,
        sdf_trunc=0.04,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)
    for i in range(len(pose_graph.nodes)):
        rgbd = images[i]
        pose = pose_graph.nodes[i].pose
        volume.integrate(rgbd, intrinsic, np.linalg.inv(pose))
    mesh = volume.extract_triangle_mesh()
    mesh.compute_vertex_normals()
    return mesh


def make_pointcloud_for_fragment(path_dataset, images,
                                  intrinsic, config):
    mesh = integrate_rgb_frames_for_fragment(
        images,
        os.path.join(path_dataset,
             config["template_fragment_posegraph_optimized"]),
        intrinsic, config)
    pcd = o3d.geometry.PointCloud()
    pcd.points = mesh.vertices
    pcd.colors = mesh.vertex_colors
    pcd_name = os.path.join(path_dataset,
                    config["template_fragment_pointcloud"])
    o3d.io.write_point_cloud(pcd_name,
                             pcd,
                             format='auto',
                             write_ascii=False,
                             compressed=True)


def process_single_fragment(images, config):
    intrinsic = TOF_INTRINSIC
    sid = 0
    eid = sid + len(images)

    make_posegraph_for_fragment(config["path_dataset"], sid, eid, images,
                                intrinsic, True, config)
    optimize_posegraph_for_fragment(config["path_dataset"], config)
    make_pointcloud_for_fragment(config["path_dataset"], images,
                                 intrinsic, config)

if __name__=="__main__":
    count = 0
    rgbd_images = []
    with DepthReader(FILENAME, homography) as reader:
        for tm, frame, _ in reader:
            if frame is not None:
                if START < tm < FINISH:
                    if count % 2 == 0:
                        rgbd_images.append(frame)
                    #pcd = o3d.geometry.PointCloud.create_from_rgbd_image(frame, TOF_INTRINSIC)
                    #o3d.visualization.draw_geometries([pcd])
                    #depth = np.asarray(frame.depth)
                    #cv2.imshow("main", cv2.normalize(depth,None,0,255,cv2.NORM_MINMAX,dtype=cv2.CV_8U))
                    color = np.asarray(frame.color)
                    cv2.imshow("color", color)
                    cv2.waitKey(15)
                    count += 1

    config = {
        "depth_diff_max": 1.0,
        "template_fragment_posegraph": "fragments/fragment.json",
        "tsdf_cubic_size": 3.0,
        "preference_loop_closure_odometry": 0.1,
        "path_dataset":"/home/phil/footage/sofa",
        "template_fragment_pointcloud": "fragments/fragment.ply",
        "template_fragment_posegraph_optimized": "fragments/fragment_optimised.json"
    }

    process_single_fragment(rgbd_images, config)