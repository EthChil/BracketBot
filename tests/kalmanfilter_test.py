import os
import numpy as np
import matplotlib.pyplot as plt
import time
from numba import jit

from filterpy.kalman import KalmanFilter#, predict, update
from filterpy.common import Q_discrete_white_noise

@jit(nopython=True)
def predict(x, P, F, Q, u, G):
    x = F @ x + G @ u
    P = (F @ P @ F.T) + Q
    return x, P

@jit(nopython=True)
def update(x, P, z, R, H):
    y = z - H @ x
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)
    x = x + K @ y
    KH = K @ H
    I_KH = np.eye(KH.shape[0]) - KH
    P = I_KH @ P @ I_KH.T + K @ R @ K.T
    return x, P


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

F = A*dt + np.eye(N)
G = B*dt
H = C
R = np.diag([1, 1e4, 1, 1e5, 1, 1e7]) * 1e-7
Q = Q_cov[:N,:N]

x = measurements[0,:N]
fxs = []
last_time = time.time()
for z, u in zip(measurements[1:], torques[:-1]):
    print(time.time()-last_time)
    last_time = time.time()
    x, P = predict(x, P, F, Q, u, G)
    x, P = update(x, P, z, R, H)
    fxs.append(x)

np.savetxt('P.txt', P)
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
