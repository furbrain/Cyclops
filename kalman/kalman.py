import json

import filterpy.kalman
import numpy as np
import scipy.linalg
from filterpy.common import Saver, Q_continuous_white_noise, Q_discrete_white_noise
from filterpy.kalman import MerweScaledSigmaPoints, JulierSigmaPoints
from mag_cal import Sensor
from scipy.spatial.transform import Rotation

from analyse import load_df, FNAME

#this will be our filter
# there are 9 state variables:
#   3 accel(world coords),
#   3 orientation (in axis vector format),
#   3 angular rate (device coords, radians)
# there are also 9 measurement vars - 3 accel, 3 mag, 3 gyro
DIP = np.deg2rad(60)
ACCEL_SLICE = slice(0,3)
ORIENTATION_SLICE = slice(3,6)
MAG_SLICE = slice(3,6)
GYRO_SLICE = slice(6,9)


def state_transition(x, dt):
    accel = x[ACCEL_SLICE]
    orientation = Rotation.from_rotvec(x[ORIENTATION_SLICE])
    gyros = x[GYRO_SLICE]
    rotation = Rotation.from_euler("xyz", gyros*dt/100, degrees=False)**100
    orientation = orientation * rotation
    output = np.zeros((9,))
    output[ACCEL_SLICE] = accel
    output[ORIENTATION_SLICE] = orientation.as_rotvec()
    output[GYRO_SLICE] = gyros
    return output

def measurement_transform(x):
    accel = x[ACCEL_SLICE] + [0, 0, 9.81] # FIXME may need to change this to 1 depending on calibration outputs
    orientation = Rotation.from_rotvec(x[ORIENTATION_SLICE])
    gyros = x[GYRO_SLICE]
    accel = orientation.apply(accel, inverse=True)
    mag = np.array([0, np.sin(DIP), np.cos(DIP)])
    mag = orientation.apply(mag, inverse=True)
    measurements = np.zeros((9,))
    measurements[ACCEL_SLICE] = accel
    measurements[MAG_SLICE] = mag
    measurements[GYRO_SLICE] = gyros
    return measurements

def state_mean(sigmas, Wm):
    results = np.average(sigmas, weights=Wm, axis=0)
    rotations = Rotation.from_rotvec(sigmas[:,ORIENTATION_SLICE])
    #invert first rotation as Wm[0] is negative...
    rotations[0] = rotations[0].inv()
    Wm = Wm.copy()
    Wm[0]  = -Wm[0]
    mean = rotations.mean(Wm)
    results[ORIENTATION_SLICE] = mean.as_rotvec()
    return results

def state_residual(x,y):
    results = x-y
    x_rot = Rotation.from_rotvec(x[ORIENTATION_SLICE])
    y_rot = Rotation.from_rotvec(y[ORIENTATION_SLICE])
    results[ORIENTATION_SLICE] = (x_rot * y_rot.inv()).as_rotvec()
    return results

points = MerweScaledSigmaPoints(9, alpha=.1, beta=2., kappa=-1, subtract=state_residual, sqrt_method=scipy.linalg.sqrtm)
#points = JulierSigmaPoints(9, kappa=-1,subtract=state_residual)
dt = 0.02
UKF = filterpy.kalman.UnscentedKalmanFilter(dim_x=9, dim_z=9, dt=dt,
                                            hx=measurement_transform,
                                            fx=state_transition,
                                            points=points,
                                            x_mean_fn=state_mean,
                                            residual_x=state_residual,
                                            sqrt_fn=scipy.linalg.sqrtm)

#Set confidences...
R = np.zeros(9)
R[MAG_SLICE] = [0.1]*3
R[ACCEL_SLICE] = [0.01]*3
R[GYRO_SLICE] = np.deg2rad([0.1]*3)
UKF.R = np.diag(R)

q = np.zeros((9,9))
q[ACCEL_SLICE,ACCEL_SLICE] = np.diag([0.8]*3) # variance for change in accel in test data is 0.4
q_g = Q_discrete_white_noise(2,var=0.1, block_size=3, order_by_dim=False) # variance for change in gyros is 0.02
q[3:6,6:9] = q_g[0,3]
q[6:9,3:6] = q_g[0,3]
np.fill_diagonal(q[3:9,3:9],np.diag(q_g))
UKF.P *= 10
UKF.Q = q
saver = Saver(UKF)
data = load_df(FNAME)
with open("/home/phil/Projects/Cyclops/gstreamer/calibration.json") as f:
    s = Sensor.from_dict(json.load(f))
data[["M'x","M'y","M'z"]] = s.apply(data[["Mx","My","Mz"]])
print(data.columns)

UKF.batch_filter(zs = data[['Ax', 'Ay', 'Az', "M'x", "M'y", "M'z", 'Gx', 'Gy', 'Gz']].to_numpy(), saver=saver)
saver.to_array()
all_x = saver.x
all_P = saver.P
all_z = saver.z
np.savez("kalman.npz",x=saver.x, P=saver.P, z=saver.z)