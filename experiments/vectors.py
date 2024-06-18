from typing import Tuple, Callable, Sequence

import numpy as np
import scipy.optimize
from scipy.spatial.transform import Rotation, Slerp

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


def get_camera_orientations(rots: Rotation, offset_transform: Rotation) -> Tuple[Rotation, Rotation]:
    reals: Rotation = rots * offset_transform

    offsets = np.random.uniform(-1.0, 1.0, (len(reals), 3))
    offsets = Rotation.from_euler("ZXY", np.deg2rad(offsets))
    reals = offsets * reals
    relatives = reals[0].inv() * reals
    return reals, relatives


def solve_my_problem_bad(sensor_data: Rotation, camera_data: Rotation):
    s = sensor_data[0].inv() * sensor_data
    s = np.hstack(s[1:].as_matrix()).T
    c = np.hstack(camera_data[1:].as_matrix()).T
    r, fit = Rotation.align_vectors(s, c)
    print(np.rad2deg(r.as_euler("ZXY")))


def solve_my_problem_iterative(sensor_data: Rotation, camera_data: Rotation):
    res = scipy.optimize.minimize(value_function, (1.0, 1.0, 1.0), args=(sensor_data, camera_data))
    return Rotation.from_euler("ZXY", np.deg2rad(res.x))


def value_function(angles, s: Rotation, c: Rotation):
    q = Rotation.from_euler("ZXY", np.deg2rad(angles))
    output_rots = q.inv() * s[0].inv() * s[1:] * q * c[1:].inv()
    return np.rad2deg(np.sqrt(np.mean(output_rots.magnitude() ** 2)))


def solve_my_problem_with_complex_eigenvectors(sensor_data: Rotation, camera_data: Rotation):
    s = sensor_data[0].inv() * sensor_data[1:]
    c = camera_data[1:]
    Q_all = []
    for sx, cx in zip(s, c):
        Rs, Ds = diagonalize(sx.as_matrix())
        Rc, Dc = diagonalize(cx.as_matrix())
        print("Rs: ", Ds)
        print("Rc: ", Dc)
        Qm = Rs @ np.linalg.inv(Rc)
        print("Qm: ", Qm)
        Qr = Rotation.from_matrix(Qm)
        print(np.rad2deg(Qr.as_euler("ZXY")))
        Q_all.append(Qr)
    Q = Rotation.concatenate(Q_all)
    return Q.mean()


def solve_with_kabsch_on_rotvecs(sensor_data: Rotation, camera_data: Rotation):
    # annoyingly, this works...
    s = sensor_data[0].inv() * sensor_data[1:]
    c = camera_data[1:]
    sr = s.as_rotvec()
    #sr = sr / np.atleast_2d(np.linalg.norm(sr, axis=1)).T
    cr = c.as_rotvec()
    #cr = cr / np.atleast_2d(np.linalg.norm(cr, axis=1)).T
    rot, resid = Rotation.align_vectors(sr, cr)
    return rot


def solve_with_kabsch_on_rotvecs_normed(sensor_data: Rotation, camera_data: Rotation):
    # annoyingly, this works...
    s = sensor_data[0].inv() * sensor_data[1:]
    c = camera_data[1:]
    sr = s.as_rotvec()
    sr = sr / np.atleast_2d(np.linalg.norm(sr, axis=1)).T
    cr = c.as_rotvec()
    cr = cr / np.atleast_2d(np.linalg.norm(cr, axis=1)).T
    rot, resid = Rotation.align_vectors(sr, cr)
    return rot


def tester(yaw: float, pitch: float, count: int, offset_transform: Rotation,
           functions: Sequence[Callable[[Rotation, Rotation], Rotation]]):
    s = get_test_sensor_orientations(count, yaw, pitch)
    g, c = get_camera_orientations(s, offset_transform)
    errors = []
    for function in functions:
        rot = function(s, c)
        degs = np.rad2deg(rot.as_euler("ZXY"))
        true_error = np.rad2deg((offset_transform * rot.inv()).magnitude())
        #print(f"{function.__name__}: {degs}, {value_function(degs, s, c)}, {true_error}")
        errors.append(true_error)
    return errors


all_errors = []
for _ in range(300):
    yaw = np.random.randint(0, 360)
    pitch = np.random.randint(-90, 90)
    count = 100
    offset_yaw = np.random.randint(0, 360)
    offset_pitch = np.random.randint(-90, 90)
    offset_roll = np.random.randint(0, 360)

    offset_transform = Rotation.from_euler("ZXY", np.deg2rad((offset_yaw, offset_pitch, offset_roll)))

    #print(f"{offset_yaw}, {offset_pitch}, {offset_roll} x {count}")
    errors = tester(yaw, pitch, count, offset_transform, [solve_my_problem_iterative,
                                                 solve_with_kabsch_on_rotvecs,
                                                 solve_with_kabsch_on_rotvecs_normed])
    all_errors.append(errors)
all_errors = np.array(all_errors)
print(np.mean(all_errors, axis=0))