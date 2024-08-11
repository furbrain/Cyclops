import cv2
import numpy as np


class Lens:
    def __init__(self):
        self.K = None
        self.D = None
        self.new_K = None
        self.map1 = None
        self.map2 = None

    def calibrate(self, images, show_results: bool = False):
        CHECKERBOARD = (6, 9)
        subpix_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
        calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_CHECK_COND + cv2.fisheye.CALIB_FIX_SKEW
        objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
        objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
        objpoints = []
        imgpoints = []
        for frame in images:
            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(frame, CHECKERBOARD,
                                                     cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
            # If found, add object points, image points (after refining them)
            if ret == True:
                objpoints.append(objp)
                cv2.cornerSubPix(frame, corners, (3, 3), (-1, -1), subpix_criteria)
                imgpoints.append(corners)
        K = np.zeros((3, 3))
        D = np.zeros((4, 1))
        retval, self.K, self.D, rvecs, tvecs = cv2.fisheye.calibrate(
            objpoints,
            imgpoints,
            frame.shape[::-1],
            K,
            D,
            flags=calibration_flags,
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6))
        self.new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, frame.shape[::-1], np.eye(3),
                                                                              balance=0.5)
        self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(self.K, self.D, np.eye(3), self.new_K, frame.shape[::-1], cv2.CV_32FC1)
        # and then remap:
        if show_results:
            results = self.undistort(images)
            for frame in results:
                cv2.imshow("undistort", frame)
                cv2.waitKey(1000)

    def save(self, fname: str):
        np.savez(fname, K=self.K, new_K=self.new_K, D=self.D, map1=self.map1, map2=self.map2)

    @classmethod
    def load(cls, fname: str):
        obj = cls()
        with np.load(fname) as data:
            for key, item in data.items():
                setattr(obj,key,item)
        return obj

    def undistort(self, images):
        if self.map1 is None:
            raise ValueError("Lens must be either calibrated or have a calibration loaded")
        results = []
        for frame in images:
            undistorted_img = cv2.remap(frame, self.map1, self.map2, interpolation=cv2.INTER_LINEAR,
                                        borderMode=cv2.BORDER_CONSTANT)
            results.append(undistorted_img)
        return results
