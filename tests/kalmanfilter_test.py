import os
import numpy as np
import matplotlib.pyplot as plt

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise


measurements = np.loadtxt('../states.txt')
torques = np.loadtxt('../torques.txt')

N = 6
dt = 0.005

# Kalman filter
f = KalmanFilter(dim_x=N, dim_z=N, dim_u=2)

A  = np.loadtxt('../lqr_params/A.txt', delimiter=',')[:N,:N]
B = np.loadtxt('../lqr_params/B.txt', delimiter=',')[:N]
C = np.eye(N)

Q_cov = np.loadtxt('Q_cov.txt')
# P = np.eye(N) * 1e5
P = np.loadtxt('P.txt') if os.path.exists('P.txt') else np.eye(6) * 1e-6

f.F = A*dt + np.eye(N)
f.B = B*dt
f.H = C
f.P = P
f.R = np.diag([1, 1e4, 1, 1e5, 1, 1e7]) * 1e-7
f.Q = Q_cov[:N,:N]

f.x = measurements[0,:N]
fxs = []
for z, u in zip(measurements[1:], torques[:-1]):
    f.predict(u)
    f.update(z)
    fxs.append(f.x)

np.savetxt('P.txt', f.P)
fxs = np.stack(fxs)

fig, axs = plt.subplots(nrows=N, ncols=1, figsize=(50, 15))
for i, name in enumerate(['Position', 'Velocity', 'Pitch', 'Pitch rate', 'Yaw', 'Yaw rate']):
    axs[i].plot(measurements[:, i], label=f'Measured {name}')
    axs[i].plot(fxs[:, i], label=f'Predicted {name}')
    axs[i].set_ylim([np.min(measurements[:, i]), np.max(measurements[:, i])])
    axs[i].set_xticks([])
    axs[i].legend()
plt.tight_layout()
plt.savefig('comparison.png')
# plt.show()
