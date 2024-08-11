import cv2
import mag_cal
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation
import scipy.optimize
from reader.LensCalibration import Lens


def iterative(sensor_data: Rotation, camera_data: Rotation):
    res = scipy.optimize.minimize(value_function, (1.0, 1.0, 1.0), args=(sensor_data, camera_data))
    rot = Rotation.from_euler("ZXY", np.deg2rad(res.x))
    return rot

def value_function(angles, s: Rotation, c: Rotation):
    q = Rotation.from_euler("ZXY", np.deg2rad(angles))
    output_rots = q.inv() * s[0].inv() * s[1:] * q * c[1:].inv()
    return np.rad2deg(np.sqrt(np.mean(output_rots.magnitude() ** 2)))

def get_image_rotation(a, b):
    orb_detector = cv2.ORB.create()
    a_corners, a_desc = orb_detector.detectAndCompute(a, None)
    b_corners, b_desc = orb_detector.detectAndCompute(b, None)
    bf = cv2.BFMatcher()
    matches = bf.knnMatch(a_desc, b_desc, k=2)
    # Apply ratio test
    good = []
    a_indexes = []
    b_indexes = []
    for m, n in matches:
        if m.distance < 0.75 * n.distance:
            good.append(m)
            a_indexes.append(m.queryIdx)
            b_indexes.append(m.trainIdx)
    good_a = cv2.KeyPoint.convert(a_corners, a_indexes)
    good_b = cv2.KeyPoint.convert(b_corners, b_indexes)
    tr, _ = cv2.estimateAffinePartial2D(good_b, good_a)
    return tr


def get_rotation_from_transform(tr: np.ndarray, cam_matrix: np.ndarray):
    y = np.arctan2(tr[0, 1], tr[0, 0])
    scale = np.linalg.norm(tr[0:2,0])
    xz = np.dot(tr,(240,320, 1)) - np.array((240,320))
    print(xz)
    x = np.arctan2(xz[0], cam_matrix[0, 0])
    z = np.arctan2(xz[1], cam_matrix[1, 1])
    return Rotation.from_euler("ZXY",(z, -x, -y))


class IMUCalibration:
    def __init__(self):
        self.imu_intrinsic = mag_cal.Calibration(mag_axes="+Y+X-Z", grav_axes="+Y+X-Z")
        self.rotation = None

    def calibrate(self, snapshots: pd.DataFrame, lens:Lens):
        # do ellipsoid calibration first
        mag_data = snapshots[['Mx', 'My', 'Mz']].to_numpy()
        grav_data = snapshots[['Ax', 'Ay', 'Az']].to_numpy()
        self.imu_intrinsic.calibrate(mag_data, grav_data, routine=mag_cal.Calibration.ELLIPSOID)
        runs = self.imu_intrinsic.find_similar_shots(mag_data,grav_data)
        print(runs)
        #calculate image rotations
        for start, finish in runs:
            imu_matrices = (self.imu_intrinsic.get_orientation_matrix(mag_data, grav_data)[start:finish])
            imu_rots = Rotation.from_matrix(imu_matrices)
            frames = snapshots['undistorted'].iloc[start:finish]
            cam_rots = []
            for rot, frame in zip(imu_rots,frames):

                tr = get_image_rotation(frames.iloc[0], frame)
                print("angles: ", np.rad2deg((imu_rots[0].inv() * rot).as_euler("ZXY")))
                cv2.imshow("original", frames.iloc[0])
                cam_rot = get_rotation_from_transform(tr, lens.new_K)
                print("angles_2: ", np.rad2deg(cam_rot.as_euler("ZXY")))
                cam_rots.append(cam_rot)
                cv2.imshow("current", frame)
                cv2.waitKey(0)
            cam_rots = Rotation.concatenate(cam_rots)
            offset = iterative(sensor_data=imu_rots, camera_data=cam_rots)
            print(np.rad2deg(offset.as_euler("ZXY")))
            print(value_function(np.rad2deg(offset.as_euler("ZXY")), imu_rots, cam_rots))
    def save(self, fname):
        ...

    @classmethod
    def load(cls, fname):
        ...
