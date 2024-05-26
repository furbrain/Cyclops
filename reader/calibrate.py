import collections
from typing import Callable, Optional, Sequence

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import cv2

from gstreader import IMUReader, VidReader
Period = collections.namedtuple("Period", ["start", "end"])

class CyclopsReader:
    """A reader for files created with cyclops. Can read video and IMU readings"""
    def __init__(self, filename: str, truncate_at:Optional[float] = None):
        self.filename = filename
        self.truncate_at = truncate_at
        self.imu_spec = f"""filesrc location="{filename}" name=fsrc ! matroskademux name=demux !  
                            video/x-raw ! appsink name=telemetry sync=false"""
        self.vid_spec = f"""filesrc location="{filename}" ! matroskademux ! video/x-h264 ! decodebin ! 
                            videoconvert ! video/x-raw,format=GRAY8 ! appsink sync=false"""

    def telemetry_generator(self):
        with IMUReader(self.imu_spec) as imu:
            while True:
                tm, floats = imu.get_frame()
                if tm is None or self.truncate_at is not None and tm > self.truncate_at:
                    break
                else:
                    yield tm, floats

    def video_generator(self):
        with VidReader(self.vid_spec) as vid:
            while True:
                tm, frame = vid.get_frame()
                if self.truncate_at is not None and tm > self.truncate_at:
                    break
                if tm is None:
                    break
                else:
                    yield tm, frame

    @staticmethod
    def get_max_abs_gyro(data):
        return np.max(np.abs(data))

    def get_all_telemetry_as_dataframe(self):
        df = pd.DataFrame(columns=["Mx", "My", "Mz", "Ax", "Ay", "Az", "Gx", "Gy", "Gz"])
        for tm, floats in self.telemetry_generator():
            df.at[tm, :] = floats
        df['GMax'] = df[['Gx', 'Gy', 'Gz']].apply(self.get_max_abs_gyro, axis=1, raw=True) * 20
        df['static'] = df['GMax'] <= 6
        return df

    @staticmethod
    def get_static_periods(series, min_time=0.3):
        first_true = None
        last_true = None
        recent_true = False
        periods = []
        for ts, static in series.items():
            if recent_true:
                if static:
                    last_true = ts
                else:
                    recent_true = False
                    if last_true - first_true > min_time:
                        periods.append(Period(first_true, last_true))
            else:
                if static:
                    first_true = ts
                    last_true = ts
                    recent_true = True
        if recent_true:
            if last_true - first_true > min_time:
                periods.append(Period(first_true, last_true))
        return periods

    @staticmethod
    def opticalFlow(a: np.ndarray, b: np.ndarray) -> Optional[float]:
        feature_params = dict(maxCorners=100,
            qualityLevel=0.3,
            minDistance=7,
            blockSize=7)
        lk_params = dict(winSize=(15, 15),
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
        a_corners = cv2.goodFeaturesToTrack(a, mask=None, **feature_params)
        b_corners, st, err = cv2.calcOpticalFlowPyrLK(a, b, a_corners, None, **lk_params)
        if b_corners is not None:
            good_a = b_corners[st == 1]
            good_b = a_corners[st == 1]
            distance2 = np.sqrt(np.sum((good_a - good_b) ** 2, axis=1))
            return np.mean(distance2)
        return None

    @staticmethod
    def congruent_periods(a: Period, bs: Sequence[Period]):
        results = []
        for b in bs:
            if b.end < a.start:
                continue
            if b.start > a.end:
                continue
            results.append(b)
        return results

    def get_agreed_periods(self, xs, ys):
        while True:
            xs = [x for x in xs if len(self.congruent_periods(x, ys)) == 1]
            ys = [y for y in ys if len(self.congruent_periods(y, xs)) == 1]
            if len(xs) == len(ys):
                break
        results = [Period(max(x.start, y.start), min(x.end, y.end)) for x,y in zip(xs, ys)]
        return results

    def read_video(self, measure_func: Callable[[np.ndarray, np.ndarray], Optional[float]], window_size: int = 5):
        outputs = []
        Frame = collections.namedtuple("Frame", ["tm", "frame"])
        dq = collections.deque(maxlen = window_size)
        for f in (Frame(tm, frame) for tm, frame in self.video_generator()):
            if self.truncate_at:
                if f.tm > self.truncate_at:
                    break
            dq.append(f)
            if len(dq)==window_size:
                a = dq[0].frame
                metrics = pd.Series((measure_func(a, x.frame) for x in list(dq)[1:]))
                tms = pd.Series(x.tm for x in dq)
                outputs.append((tms.mean(),metrics.mean()))
        return outputs

    def get_snaphots(self, periods: Sequence[Period], imu: pd.DataFrame):
        frames = pd.Series(dtype="object", name="Frame")
        with VidReader(self.vid_spec) as vid:
            for tm in ((p.start + p.end) / 2 for p in periods):
                vid.seek(tm - 1)
                while True:
                    new_tm, frame = vid.get_frame()
                    if new_tm > tm:
                        break
                frames[new_tm] = frame
        print(frames)
        imu_indexes = imu.index.get_indexer(frames.index, method="nearest")
        imu_data = imu.iloc[imu_indexes]
        print(imu_data)
        result = pd.merge_asof(frames, imu_data, left_index=True, right_index=True, direction="nearest")
        print(result)
        return result

    def get_stable_images(self):
        print("Get frame diffs")
        frame_diffs = np.array(self.read_video(self.opticalFlow, 5))
        df = pd.DataFrame({"motion": frame_diffs[:, 1]}, index=frame_diffs[:, 0])
        df['static'] = df['motion'] < 8.0
        print("getting statics")
        periods_cam = self.get_static_periods(df['static'])

        print("Reading IMU data")
        tel = self.get_all_telemetry_as_dataframe()
        periods_imu = self.get_static_periods(tel['static'])
        periods = self.get_agreed_periods(periods_cam, periods_imu)
        snaps = self.get_snaphots(periods, tel)
        return snaps




def make_interrupted_lines(xs: Sequence[Period], y:float=-5):
    result_x = []
    result_y = []
    for x in xs:
        result_x.extend([x.start, x.end, None])
        result_y.extend([y,y,None])
    return result_x, result_y


if __name__=="__main__":
    CHECKERBOARD = (6, 9)
    subpix_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
    calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_CHECK_COND + cv2.fisheye.CALIB_FIX_SKEW
    objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    objpoints = []
    imgpoints = []
    cyclops = CyclopsReader("../footage/vid0.mkv")
    results = cyclops.get_stable_images()
    for frame in results['Frame']:
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(frame, CHECKERBOARD,
                                                 cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
        # If found, add object points, image points (after refining them)
        if ret == True:
            print("chessboard found")
            objpoints.append(objp)
            cv2.cornerSubPix(frame, corners, (3, 3), (-1, -1), subpix_criteria)
            imgpoints.append(corners)
        cv2.imshow("main", frame)
        cv2.waitKey(500)
    K = np.zeros((3, 3))
    D = np.zeros((4, 1))
    retval, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
        objpoints,
        imgpoints,
        frame.shape[::-1],
        K,
        D,
        flags=calibration_flags,
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6))
    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, frame.shape[::-1], np.eye(3), balance=0.5)
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), new_K, frame.shape[::-1], cv2.CV_32FC1)
    # and then remap:
    for frame in results['Frame']:
        undistorted_img = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        cv2.imshow("undistort", undistorted_img)
        cv2.waitKey(1000)
