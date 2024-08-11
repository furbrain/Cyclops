from typing import Tuple, Callable, Sequence

import numpy as np
import pandas
import scipy.optimize
from scipy.spatial.transform import Rotation, Slerp
import matplotlib.pyplot as plt

### this file is to test my calibration routine
# angle system is yaw, pitch, roll (xzy)
# we want to solve q' * C_x * q * S_0 = S_x
# which is also S_x' * q' * C_x * q * S_0 = 1
# S_x is sensor orientation measured directly, assumed to be true
# q is unknown and value to find - is rotation from sensor frame to camera frame
# D_x = q * S_x
# C_0 is 1
# C_x is rotation of camera image relative to C_0
# yaw = 10
# pitch = 15



def diagonalize(A: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    #get eigens
    eigenValues, eigenVectors = np.linalg.eig(A)
    # sort descending
    idx = eigenValues.argsort()[::-1]
    eigenValues = eigenValues[idx]
    eigenVectors = eigenVectors[:, idx]
    return eigenVectors, np.diag(eigenValues)


def get_test_sensor_orientations(count, yaw, pitch):
    offsets = np.random.uniform(-10, 10, (count, 3))
    rolls = np.linspace(0, 360, count, endpoint=False)
    yaws = np.full(count, yaw)
    pitches = np.full(count, pitch)
    angles = np.vstack((yaws, pitches, rolls)).T
    angles += offsets
    return Rotation.from_euler("ZXY", np.deg2rad(angles))


def get_camera_orientations(rots: Rotation, offset_transform: Rotation, error_size:float) -> Tuple[Rotation, Rotation]:
    reals: Rotation = rots * offset_transform

    offsets = np.random.uniform(-error_size, error_size, (len(reals), 3))
    offsets = Rotation.from_euler("ZXY", np.deg2rad(offsets))
    reals = reals * offsets
    relatives = reals[0].inv() * reals
    return reals, relatives


def iterative(sensor_data: Rotation, camera_data: Rotation):
    res = scipy.optimize.minimize(value_function, (1.0, 1.0, 1.0), args=(sensor_data, camera_data))
    rot = Rotation.from_euler("ZXY", np.deg2rad(res.x))
    return rot, None, None


def value_function(angles, s: Rotation, c: Rotation):
    q = Rotation.from_euler("ZXY", np.deg2rad(angles))
    output_rots = q.inv() * s[0].inv() * s[1:] * q * c[1:].inv()
    return np.rad2deg(np.sqrt(np.mean(output_rots.magnitude() ** 2)))


def rotvecs(sensor_data: Rotation, camera_data: Rotation):
    # annoyingly, this works...
    s = sensor_data[0].inv() * sensor_data[1:]
    c = camera_data[1:]
    sr = s.as_rotvec()
    cr = c.as_rotvec()
    rot, resid = Rotation.align_vectors(sr, cr)
    diffs = rot.apply(cr) - sr
    norms = np.linalg.norm(diffs, axis=1)
    rssd_calculated = np.mean(norms)
    return rot, rssd_calculated, resid


def rotvecs_normed(sensor_data: Rotation, camera_data: Rotation):
    # annoyingly, this works...
    s = sensor_data[0].inv() * sensor_data[1:]
    c = camera_data[1:]
    sr = s.as_rotvec()
    norm_sr = sr / np.atleast_2d(np.linalg.norm(sr, axis=1)).T
    norm_sr[norm_sr[:,1]<-0.5] *= -1
    cr = c.as_rotvec()
    norm_cr = cr / np.atleast_2d(np.linalg.norm(cr, axis=1)).T
    norm_cr[norm_cr[:,1]<-0.5] *= -1
    rot, resid = Rotation.align_vectors(norm_sr, norm_cr)
    diffs = rot.apply(norm_cr) - norm_sr
    norms = np.linalg.norm(diffs, axis=1)
    rssd_calculated = np.mean(norms)
    return rot, rssd_calculated, resid


def tester(yaw: float, pitch: float, count: int, offset_transform: Rotation,
           functions: Sequence[Callable[[Rotation, Rotation], Rotation]]):
    s = get_test_sensor_orientations(count, yaw, pitch)
    g, c = get_camera_orientations(s, offset_transform, 1.0)
    errors = []
    for function in functions:
        rot, _ = function(s, c)
        degs = np.rad2deg(rot.as_euler("ZXY"))
        #print(offset_transform.as_euler("ZXY"), rot.as_euler("ZXY"))
        true_error = np.rad2deg((offset_transform * rot.inv()).magnitude())
        errors.append(true_error)
    return errors

def plotter(yaw: float, pitch: float, count: int, offset_transform: Rotation, error_size):
    s = get_test_sensor_orientations(count, yaw, pitch)
    g, c = get_camera_orientations(s, offset_transform, error_size)
    errors = []
    for function in (rotvecs, rotvecs_normed, iterative):
        fname = function.__name__
        rot, rssd, resid = function(s, c)
        true_error = np.rad2deg((offset_transform * rot.inv()).magnitude())
        computed_error = value_function(np.rad2deg(rot.as_euler("ZXY")), s, c)
        l = locals()
        errors.append({x: l[x] for x in "fname true_error computed_error resid error_size count rssd".split()})
    return errors

def curve_fn(x, a, pow):
    return a * x[:,0] * (x[:, 1] ** pow)


def generate_data():
    all_errors = []
    for _ in range(10000):
        yaw = np.random.randint(0, 360)
        pitch = np.random.randint(-30, 30)
        offset_yaw = np.random.randint(-10, 10)
        offset_pitch = np.random.randint(-10, 10)
        offset_roll = np.random.randint(-10, 10)
        count = np.random.randint(5, 20)
        # count = 30
        offset_transform = Rotation.from_euler("ZXY", np.deg2rad((offset_yaw, offset_pitch, offset_roll)))

        error_size = np.random.uniform(0, 6.0)
        # print(f"{offset_yaw}, {offset_pitch}, {offset_roll} x {count}")
        errors = plotter(yaw, pitch, count, offset_transform, error_size)
        all_errors.extend(errors)
    return pandas.DataFrame(all_errors)


#data = generate_data()
data = pandas.read_csv("test_data.csv")
data.groupby("fname").plot.scatter("error_size","true_error")
plt.show()
for f in data['fname'].unique():
    # plot mean of accuracy against a) count and b) error_size
    subdata = data[data['fname']==f]
    print(f)
    print(subdata.groupby("count")['true_error'].mean())





