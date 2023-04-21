import numpy as np
import matplotlib.pyplot as plt

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise


f = KalmanFilter (dim_x=6, dim_z=6, dim_u=2)

A = np.loadtxt('../lqr_params/A.txt', delimiter=',')
B = np.loadtxt('../lqr_params/B.txt', delimiter=',')

print(A.shape, B.shape)
        
f.F = A
f.B = B
f.H = np.eye(6)
f.P = np.eye(6)*0.06
f.R = np.diag([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
f.Q = np.eye(6)*0.1

measurements = np.loadtxt('../states.txt')
torques = np.loadtxt('../torques.txt')

f.x = measurements[0]
fxs = []
for z, u in zip(measurements[1:], torques[:-1]):
    f.predict(u)
    f.update(z)
    fxs.append(f.x)
print(np.diag(f.P))
fxs = np.stack(fxs)


fig, axs = plt.subplots(nrows=6, ncols=1, figsize=(100, 30))
for i in range(6):
    axs[i].plot(measurements[1:,i], label='measured state')
    axs[i].plot(fxs[:,i], label='estimated state')
    axs[i].legend()
plt.savefig('kalman.png')
