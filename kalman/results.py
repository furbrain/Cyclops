import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

data = np.load("kalman.npz")
x = data['x']
P = data['P']
z = data['z']
rots = Rotation.from_rotvec(x[:,3:6])
eulers = rots.as_euler("xyz", degrees=True)
plt.plot(eulers[:,2])
plt.show()