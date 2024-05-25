import time
import gi
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import cv2
from scipy.spatial.distance import cdist

spec_simple = """filesrc location="../footage/vid1.mkv" name=fsrc ! matroskademux name=demux !  
                 video/x-raw ! appsink name=telemetry sync=false"""

spec_video = """filesrc location="../footage/vid1.mkv" ! matroskademux ! video/x-h264 ! decodebin ! videoconvert ! video/x-raw,format=GRAY8 ! appsink sync=false"""

from gstreader import IMUReader, VidReader

    

def get_telemetry():
    imu = IMUReader(spec_simple)
    df = pd.DataFrame(columns=["Mx", "My", "Mz", "Ax", "Ay", "Az", "Gx", "Gy", "Gz"])
    while True:
        tm, floats = imu.get_frame()
        if tm is None:
            break
        else:
            df.at[tm, :] = floats
    print("video read complete")
    print(len(df))
    df['GMax'] = df[['Gx','Gy','Gz']].apply(get_max_abs_gyro,axis=1,raw=True)
    df['static'] = df['GMax'] <= 0.3
    return df

def get_max_abs_gyro(data):
    return np.max(np.abs(data))

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
                    periods.append((first_true, last_true))
        else:
            if static:
                first_true = ts
                last_true = ts
                recent_true = True
    if recent_true:
        if last_true - first_true > min_time:
            periods.append((first_true, last_true))
    return periods

def read_video():
    #need to add timestamps to outputs
    feature_params = dict( maxCorners = 100,
        qualityLevel = 0.3,
        minDistance = 7,
        blockSize = 7 )
    lk_params = dict( winSize = (15, 15),
        maxLevel = 2,
        criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
    outputs = []
    last_corners = None
    cap = cv2.VideoCapture("../footage/vid1.mkv")
    with VidReader(spec_video) as vid:
        while True:
            tm, new_gray = vid.get_frame()
            if tm is None:
                print("bad frame/EOS")
                break
            if last_corners is not None:
                p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray,new_gray, last_corners, None, **lk_params)
                if p1 is not None:
                    good_new = p1[st==1]
                    good_old = last_corners[st==1]
                    distance2 = np.sqrt(np.sum((good_new - good_old) ** 2, axis=1))
                    outputs.append((tm,np.mean(distance2)))
            old_gray = new_gray
            
            last_corners = cv2.goodFeaturesToTrack(old_gray, mask=None, **feature_params)
    return outputs

#need to graph outputs and threshold            
frame_diffs = np.array(read_video())
print(frame_diffs.shape)
df = pd.DataFrame({"motion":frame_diffs[:,1]}, index=frame_diffs[:,0])

df['static'] = df['motion'] < 8.0

print("camera bits")
periods = get_static_periods(df['static'])
print(periods)

print("IMU bits")
tel = get_telemetry()
periods = get_static_periods(tel['static'])
print(periods)

ax = df.plot.line()
(tel['GMax']*5).plot.line(ax=ax)
plt.show()

