import collections
from typing import Optional, Sequence, Callable

import cv2
import numpy as np
import pandas as pd

from gstreader import IMUReader, VidReader
from reader.IMUCalibration import IMUCalibration
from reader.LensCalibration import Lens

Period = collections.namedtuple("Period", ["start", "end"])


def get_optical_flow_points(a, b):
    feature_params = dict(maxCorners=100,
                          qualityLevel=0.3,
                          minDistance=7,
                          blockSize=7)
    lk_params = dict(winSize=(15, 15),
                     maxLevel=2,
                     criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

    a_points = cv2.goodFeaturesToTrack(a, mask = None, **feature_params)
    b_points, st, err = cv2.calcOpticalFlowPyrLK(a, b, a_points, None, **lk_params)
    if b_points is not None:
        good_new = a_points[st == 1]
        good_old = b_points[st == 1]
        return good_new, good_old
    return None, None


class CyclopsReader:
    """A reader for files created with cyclops. Can read video and IMU readings"""
    def __init__(self, filename: str, lens_cal_file: str = "", truncate_at:Optional[float] = None):
        self.filename = filename
        self.truncate_at = truncate_at
        if lens_cal_file:
            self.lens = Lens.load(lens_cal_file)
            print("cam: ", self.lens.K)
        else:
            self.lens = Lens()
        self.imu_cal = IMUCalibration()

    def telemetry_generator(self):
        with IMUReader.from_filename(self.filename) as imu:
            for tm, floats in imu:
                if self.truncate_at is not None and tm > self.truncate_at:
                    break
                else:
                    yield tm, floats

    def video_generator(self):
        with VidReader.from_filename(self.filename, gray=True) as vid:
            for tm, frame in vid:
                if self.truncate_at is not None and tm > self.truncate_at:
                    break
                yield tm, frame

    @staticmethod
    def get_max_abs_gyro(data):
        return np.max(np.abs(data))

    def get_all_telemetry_as_dataframe(self):
        df = pd.DataFrame(columns=["Mx", "My", "Mz", "Ax", "Ay", "Az", "Gx", "Gy", "Gz"])
        for tm, floats in self.telemetry_generator():
            df.loc[tm] = floats
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
    def opticalFlowPoints(a: np.ndarray, b: np.ndarray) -> Optional[float]:
        good_a, good_b = get_optical_flow_points(a, b)
        if good_a is not None:
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

    def get_snaphots(self, periods: Sequence[Period], imu: pd.DataFrame) -> pd.DataFrame:
        frames = pd.Series(dtype="object", name="Frame")
        with VidReader.from_filename(self.filename, gray=True) as vid:
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
        try:
            return pd.read_pickle(f"{self.filename}.stable.pkl")
        except IOError:
            pass
        print("Reading IMU data")
        tel = self.get_all_telemetry_as_dataframe()
        periods_imu = self.get_static_periods(tel['static'])
        #print("Get frame diffs")
        #frame_diffs = np.array(self.read_video(self.opticalFlowPoints, 5))
        #df = pd.DataFrame({"motion": frame_diffs[:, 1]}, index=frame_diffs[:, 0])
        #df['static'] = df['motion'] < 8.0
        #print("getting statics")
        #periods_cam = self.get_static_periods(df['static'])
        #print(periods_cam)
        #periods = self.get_agreed_periods(periods_cam, periods_imu)
        periods = periods_imu
        snaps = self.get_snaphots(periods, tel)
        snaps.to_pickle(f"{self.filename}.stable.pkl")
        return snaps

    def calibrate_lens(self, show_results: bool = False):
        results = self.get_stable_images()
        l = len(results['Frame'])
        if l == 0:
            raise ValueError("No Stable Frames found")
        if l < 6:
            raise ValueError(f"Only {l} stable frames found, need at least 6")
        self.lens.calibrate(results['Frame'], show_results)

    def calibrate_imu(self):
        snapshots = self.get_stable_images()
        snapshots['undistorted'] = self.lens.undistort(snapshots['Frame'])
        self.imu_cal.calibrate(snapshots, self.lens)