from typing import Tuple

import filterpy
import mag_cal.utils
import numpy as np
import pandas as pd
import scipy
from scipy.spatial.transform import Rotation

from reader.gstreader import IMUReader

FNAME = '/home/phil/footage/storrs/vid.mkv'

def get_dataframe(fname: str) -> pd.DataFrame:
    with IMUReader.from_filename(fname) as reader:
        data = []
        for frame in reader:
            data.append((frame[0], *frame[1]))
    frame = pd.DataFrame(data=data, columns=["Time",
                                             "Mx","My","Mz",
                                             "Ax","Ay","Az",
                                             "Gx","Gy","Gz",])
    return frame

def make_pickle_fname(fname: str) -> str:
    return fname + ".imu.pkl"

def load_df(fname: str) -> pd.DataFrame:
    return pd.read_pickle(make_pickle_fname(fname))

def save_df(fname:str, df: pd.DataFrame):
    df.to_pickle(make_pickle_fname(fname))

def add_magnitudes(df: pd.DataFrame):
    for f in "GAM":
        norm = np.linalg.norm(df[[f + 'x', f + 'y', f + 'z']].to_numpy(), axis=1)
        print(norm)
        df[f"|{f}|"] = norm

def dirty_ellipsoid_fit(data: np.array) -> Tuple[np.array, np.array]:
    def gen_transform(x: np.array) -> np.array:
        rotation = Rotation.from_euler("zyx",x[:3])
        scale = np.diag(x[3:6])
        transform = rotation.as_matrix() @ scale @ rotation.inv().as_matrix()
        return transform

    def error_func(x: np.array, values:np.array) -> float:
        transformed = (gen_transform(x) @ (values - x[6:9]).T).T

        return np.sum((1-np.linalg.norm(transformed,axis=1))**2)

    x0 = np.array([1,1,1,00.02,0.02,0.02,10,10,10])
    result = scipy.optimize.minimize(error_func, x0, data)
    print(result)
    print(gen_transform(result.x), result.x[6:9])

if __name__=="__main__":
    import matplotlib.pyplot as plt
    from mag_cal import Sensor
    import json
    df = load_df(FNAME)
    #with open("/home/phil/Projects/Cyclops/gstreamer/calibration.json") as f:
    #     s = Sensor.from_dict(json.load(f))
    #print(s)
    s = Sensor()
    mag_data = df[["Mx", "My", "Mz"]].to_numpy()
    dirty_ellipsoid_fit(mag_data)
    res = s.fit_ellipsoid(mag_data[0::43])
    print(res)
    add_magnitudes(df)
    mag_calib = s.apply(mag_data)
    df[["M'x","M'y","M'z"]] = mag_calib
    df["|M'|"] = np.linalg.norm(df[["M'x","M'y","M'z"]],axis=1)*56
    grav_calib = df[['Ax','Ay','Az']]
    df['Dip'] = np.rad2deg(np.arccos((mag_calib * grav_calib).sum(axis=1)/(np.linalg.norm(mag_calib,axis=1)*df[
        '|A|'])))
    print(df['|A|'].describe())
    print(df["|M'|"].describe())
    print(df['|A|'].describe())
    print(df["Dip"].describe())
    df['|G|'] = np.rad2deg(df['|G|'])
    df.plot(x='Time',y=['|M|',"|M'|"])
    plt.show()
