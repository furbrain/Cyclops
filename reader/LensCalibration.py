from dataclasses import dataclass, field
from typing import Tuple, Any, Dict, Optional, Sequence

import cv2
from dataclasses_json import config, DataClassJsonMixin
import numpy as np

CHECKERBOARD = (9, 6)
CB_SQUARE_SIZE = 23

def npfield(conf_args: Dict[str,Any]=None, **kwargs):
    if conf_args is None:
        conf_args = {}
    return field(metadata=config(decoder=np.asarray, **conf_args), **kwargs)


def get_objpoints(imgpoints):
    objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 1, 3), np.float32)
    objp[:, 0, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) * CB_SQUARE_SIZE
    # objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    # objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    objpoints = [objp] * len(imgpoints)
    return objpoints


@dataclass
class Lens(DataClassJsonMixin):
    K: np.ndarray = npfield(default=None)
    D: np.ndarray = npfield(default=None)
    frame_shape: Optional[Tuple[int,int]] = None
    fisheye: bool = False
    undistort_balance: float = 0.5
    new_K: np.ndarray = npfield(default=None, conf_args={"exclude": lambda x: True})
    map1: np.ndarray = npfield(default=None, repr=False, conf_args={"exclude": lambda x: True})
    map2: np.ndarray = npfield(default=None, repr=False, conf_args={"exclude": lambda x: True})


    def __post_init__(self):
        if self.K is not None and self.D is not None:
            self.create_undistort_map()

    def calibrate_from_images(self, images):
        imgpoints = []
        for frame in images:
            # Find the chess board corners
            self.frame_shape = frame.shape[::-1]
            corners = self.find_cb_corners(frame)
            # If found, add object points, image points (after refining them)
            if corners:
                imgpoints.append(corners)
        self.calibrate_from_points(imgpoints)

    @staticmethod
    def find_cb_corners(frame: np.ndarray) -> Optional[np.ndarray]:
        cb_flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
        subpix_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
        ret, corners = cv2.findChessboardCorners(frame, CHECKERBOARD, flags=cb_flags)
        if ret:
            cv2.cornerSubPix(frame, corners, (3, 3), (-1, -1), subpix_criteria)
            return corners
        return None

    def calibrate_from_points(self, imgpoints):
        objpoints = get_objpoints(imgpoints)
        K = np.zeros((3, 3))
        D = np.zeros((4, 1))
        if self.fisheye:
            calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_CHECK_COND + cv2.fisheye.CALIB_FIX_SKEW
            retval, self.K, self.D, rvecs, tvecs = cv2.fisheye.calibrate(
                objpoints,
                imgpoints,
                self.frame_shape,
                K,
                D,
                flags=calibration_flags,
                criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6))
        else:
            retval, self.K, self.D, rvecs, tvecs = cv2.calibrateCamera(
                objpoints,
                imgpoints,
                self.frame_shape,
                K,
                D,
                criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6))

    def create_undistort_map(self, balance=None):
        if balance is None:
            balance = self.undistort_balance
        else:
            self.undistort_balance = balance
        if self.fisheye:
            self.new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(self.K,
                                                                                self.D,
                                                                                self.frame_shape,
                                                                                np.eye(3),
                                                                                balance=balance)
            self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(self.K, self.D, np.eye(3), self.new_K,
                                                                       self.frame_shape, cv2.CV_32FC1)
        else:
            self.new_K, _ = cv2.getOptimalNewCameraMatrix(self.K,
                                                          self.D,
                                                          self.frame_shape,
                                                          alpha=balance)
            self.map1, self.map2 = cv2.initUndistortRectifyMap(self.K,
                                                               self.D,
                                                               np.eye(3),
                                                               self.new_K,
                                                               self.frame_shape,
                                                               cv2.CV_32FC1)



    def undistort_multiple(self, images):
        if self.map1 is None:
            raise ValueError("Lens must be either calibrated or have a calibration loaded")
        results = []
        for frame in images:
            undistorted_img = self.undistort_image(frame)
            results.append(undistorted_img)
        return results

    def undistort_image(self, img):
        return cv2.remap(img, self.map1, self.map2, interpolation=cv2.INTER_LINEAR,
                                    borderMode=cv2.BORDER_CONSTANT)

    def undistort_points(self, points):
        if self.fisheye:
            return cv2.fisheye.undistortPoints(points, self.K, self.D, P=self.new_K)
        else:
            return cv2.undistortPoints(points, self.K, self.D, P=self.new_K)

    def save(self, fname: str):
        with open(fname,"w") as f:
            f.write(self.to_json())

    @classmethod
    def load(cls, fname: str):
        with open(fname) as f:
            return cls.from_json(f.read())

    @classmethod
    def from_images(cls, images: Sequence[np.ndarray], balance=0.5, fisheye=False, show_results=False):
        instance = cls(fisheye=fisheye)
        instance.calibrate_from_images(images)
        instance.create_undistort_map(balance)
        if show_results:
            results = instance.undistort_multiple(images)
            for frame in results:
                cv2.imshow("undistort", frame)
                cv2.waitKey(1000)
        return instance

    @classmethod
    def from_cb_points(cls, points:Sequence[np.ndarray], frame_shape:Tuple[int,int], balance=0.5, fisheye=False):
        instance = cls(frame_shape=frame_shape, fisheye=fisheye)
        instance.calibrate_from_points(points)
        instance.create_undistort_map(balance)
        return instance

    def undistort_and_crop(self, img: np.ndarray, tl: Tuple[float,float], wh: Tuple[float,float]) -> np.ndarray:
        """
        Undistort and crop the image
        :param img: image to modify
        :param tl: coordinates of top-left corner, in fractions (ie 0.25,0.75 is one quarter of width from left edge
        and three quarters of height from top
        :param wh: width and height in fractions
        :return: undistorted and cropped image
        """
        left = int(tl[0] * self.frame_shape[0])
        top = int(tl[1] * self.frame_shape[1])
        right = int(left + self.frame_shape[0] * wh[0])
        bottom = int(top + self.frame_shape[1] * wh[1])
        undistorted = self.undistort_image(img)
        return undistorted[top:bottom, left:right]

    def cropped_camera_matrix(self, tl: Tuple[float,float]):
        """
        Get the camera matrix for a cropped image. Note that the width and height are not needed.
        :param tl: coordinates of top-left corner, in fractions (ie 0.25,0.75 is one quarter of width from left edge
        and three quarters of height from top
        :return: Camera matrix for a cropped image
        """
        matr = self.new_K.copy()
        matr[0,2] -= tl[0]*self.frame_shape[0]
        matr[1,2] -= tl[1]*self.frame_shape[1]
        return matr


@dataclass
class DualCams(DataClassJsonMixin):
    L1: Lens
    L2: Lens
    R: np.ndarray = npfield(default=None)
    T: np.ndarray = npfield(default=None)
    E: np.ndarray = npfield(default=None)
    F: np.ndarray = npfield(default=None)

    def calibrate(self, L1Points: Sequence[np.ndarray], L2Points: Sequence[np.ndarray]):
        objpoints = get_objpoints(L1Points)
        if self.L1.fisheye and self.L2.fisheye:
            retval, _, _, _, _, self.R, self.T, _, _ = cv2.fisheye.stereoCalibrate(objpoints, L1Points, L2Points,
                                                                                   self.L1.K, self.L1.D,
                                                                                   self.L2.K, self.L2.D,
                                                                                   self.L1.frame_shape)
        elif self.L1.fisheye or self.L2.fisheye:
            l1_points = [self.L1.undistort_points(x) for x in L1Points]
            l2_points = [self.L2.undistort_points(x) for x in L2Points]
            D = np.zeros((4,1))
            retval, _, _, _, _, self.R, self.T, self.E, self.F = cv2.stereoCalibrate(objpoints, l1_points, l2_points,
                                                                                     self.L1.new_K, D,
                                                                                     self.L2.new_K, D,
                                                                                     self.L1.frame_shape)
        else:
            retval, _, _, _, _, self.R, self.T, self.E, self.F = cv2.stereoCalibrate(objpoints, L1Points, L2Points,
                                                                                     self.L1.K, self.L1.D,
                                                                                     self.L2.K, self.L2.D,
                                                                                     self.L1.frame_shape)

    def save(self, fname: str):
        with open(fname,"w") as f:
            f.write(self.to_json(indent=2))

    @classmethod
    def load(cls, fname: str):
        with open(fname) as f:
            return cls.from_json(f.read())
