import open3d as o3d
import cv2
import numpy as np
import math as maths

from TOF.depth_reader import DepthReader, TOF_INTRINSIC

FILENAME = "/home/phil/combined.mkv"

START = 8.45
FINISH = 11.45

homography = np.load("homography.npy")
count = 0
with DepthReader(FILENAME, homography) as reader:
    for tm, frame in reader:
        if START < tm < FINISH:
            pcd = o3d.geometry.PointCloud.create_from_rgbd_image(frame, TOF_INTRINSIC)
            o3d.visualization.draw_geometries([pcd])
            #cv2.imshow("main", np.asarray(frame.color))
            #cv2.waitKey(33)
            #count+=1
print(count)

