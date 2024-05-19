#from skinematics import rotmat
#from skinematics.sensors.manual import MyOwnSensor

import imu
import numpy as np
np.set_printoptions(suppress=True)
RATE=100

data = np.loadtxt("telemetry.csv", delimiter=",", skiprows=1)
timestamps = data[:,0]
imu.plot_trajectory(data[:,1:])
exit()

mag = data[:,1:4]
accel = data[:,4:7]
gyro = data[:,7:10]
init_frames = RATE // 3
def tidy_gyro(x):
    gn = np.mean(x[:init_frames], axis=0)
    return x - gn

def tidy_accel(x):
    an = np.mean(x[:init_frames], axis=0)
    magnitude = np.linalg.norm(an)
    print("before", an)
    r_x = rotmat.R('x',angle=np.rad2deg(np.arctan2(an[1],an[2])))
    an1 = np.dot(an,r_x.T)
    r_y = rotmat.R('y',angle=np.rad2deg(np.arctan2(an1[0],an1[2])))
    r = np.dot(r_x.T,r_y)
    an = np.dot(an,r)
    print("after", an)
    x = np.dot(x,r)
    x *= (9.81 / magnitude)
    return x

gyro = tidy_gyro(gyro)
accel = tidy_accel(accel)

sensor = MyOwnSensor(in_file="test", q_type="analytical", in_data={"rate": RATE,
                                                                 "mag": mag,
                                                                 "acc": accel,
                                                                 "omega": gyro})
print(sensor.vel[0:-1:10])
