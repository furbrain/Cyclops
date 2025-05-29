#!/usr/bin/env python3
import argparse
import numpy as np
from sensor_msgs.msg import CameraInfo, Image
import message_filters
import rospy
from cv_bridge import CvBridge
import cv2
import yaml
yaml.add_multi_constructor('tag:', lambda loader, suffix, node: None, Loader=yaml.SafeLoader)

class RemapNode:
    def __init__(self):
        print("initialising")
        self.sub = rospy.Subscriber("~in", Image, queue_size=5, callback=self.callback)
        self.pub = rospy.Publisher("~out", Image, queue_size=5)


        cal_file = rospy.get_param("~chain_file", "/home/phil/bags/sipeed_cal_norm-camchain.yaml")
        self.bridge = CvBridge()

        # self.map = self.get_fancy_homography_mapping(cal_file)
        self.map = self.get_mapping(cal_file)
        print("initialistaion complete")

    def get_homography_mapping(self, cal_file):
        with open(cal_file) as f:
            cams_config = yaml.safe_load(f)
        rgb = cams_config["cam0"]
        tof = cams_config["cam1"]
        matrix = np.load("/home/phil/.ros/camera_info/sipeed_h.npy")
        grid = np.mgrid[0:rgb['resolution'][1],0:rgb['resolution'][0]]
        grid_shape = grid[0].shape
        print(grid_shape)
        grid = np.dstack((grid[1],grid[0]))
        grid = grid.reshape((-1,1,2)).astype(np.float64)
        grid = cv2.perspectiveTransform(grid,matrix)
        remapx = grid[:,:,0].reshape(grid_shape).astype(np.float32)
        remapy = grid[:,:,1].reshape(grid_shape).astype(np.float32)
        remap, _ = cv2.convertMaps(remapx, remapy, cv2.CV_16SC2, nninterpolation=True)
        print(remap)
        return remap

    def get_fancy_homography_mapping(self, cal_file):
        with open(cal_file) as f:
            cams_config = yaml.safe_load(f)
        rgb = cams_config["cam0"]
        grid = np.mgrid[0:rgb['resolution'][1],0:rgb['resolution'][0]]
        grid_shape = grid[0].shape
        grid = np.dstack((grid[1],grid[0]))
        grid = grid.reshape((-1,2)).astype(np.float64)
        X = grid[:,0:1]
        Y = grid[:,1:2]
        ones = np.ones_like(X)
        XY = X*Y
        X2 = X*X
        Y2 = Y*Y
        M = np.hstack((ones,X,Y,XY,X2,Y2))
        parms = np.load("/home/phil/.ros/camera_info/sipeed_fh.npy")
        out = M @ parms
        remapx = out[:,0].reshape(grid_shape).astype(np.float32)
        remapy = out[:,1].reshape(grid_shape).astype(np.float32)
        remap, _ = cv2.convertMaps(remapx, remapy, cv2.CV_16SC2, nninterpolation=True)
        print(remap)
        return remap


    def get_mapping(self, cal_file):
        with open(cal_file) as f:
            cams_config = yaml.safe_load(f)
        rgb = cams_config["cam0"]
        tof = cams_config["cam1"]
        R = np.array(cams_config["cam1"]["T_cn_cnm1"])[:3, :3]
        #R = np.linalg.inv(R)
        # create intrinsic matrices
        for cam in rgb, tof:
            i = cam['intrinsics']
            K = np.array([[i[0], 0, i[2]], [0, i[1], i[3]], [0, 0, 1]])
            cam['K'] = K
            cam['D'] = np.array(cam['distortion_coeffs'])
        #generate points
        grid = np.mgrid[0:rgb['resolution'][1],0:rgb['resolution'][0]]
        grid_shape = grid[0].shape
        print(grid_shape)
        grid = np.dstack((grid[1],grid[0]))
        grid = grid.reshape((-1,2)).astype(np.float64)
        #undistort and rotate
        world_points = cv2.undistortPoints(grid, rgb['K'], rgb['D'],R=R)
        world_points = np.dstack((world_points,np.ones(world_points.shape[0:2])))
        #rotate and redistort
        new_points, _ = cv2.projectPoints(world_points,np.eye(3),np.array([[0.0,0,0]]), tof['K'], tof['D'])
        remapx = new_points[:,:,0].reshape(grid_shape).astype(np.float32)
        remapy = new_points[:,:,1].reshape(grid_shape).astype(np.float32)
        remap, _ = cv2.convertMaps(remapx, remapy, cv2.CV_16SC2, nninterpolation=True)
        print(remap)
        return remap

    def callback(self, img_msg: Image):
        img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
        img = cv2.remap(img, self.map,None, cv2.INTER_NEAREST)
        img = img.astype(np.float32)/1000
        msg = self.bridge.cv2_to_imgmsg(img.astype(np.float32), encoding='32FC1')
        msg.header = img_msg.header
        self.pub.publish(msg)
        
if __name__=="__main__":
    rospy.init_node("remap_node")
    RemapNode()
    rospy.spin()
