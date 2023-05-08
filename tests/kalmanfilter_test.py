import numpy as np
import matplotlib.pyplot as plt
import time
from numba import jit

from filterpy.kalman import KalmanFilter#, predict, update

# @jit(nopython=True)
def predict(x, P, F, Q, u, G):
    x = F @ x + G @ u
    P = (F @ P @ F.T) + Q
    return x, P

# @jit(nopython=True)
def update(x, P, z, R, H):
    y = z - H @ x
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)
    x = x + K @ y
    KH = K @ H
    I_KH = np.eye(KH.shape[0]) - KH
    P = I_KH @ P @ I_KH.T + K @ R @ K.T
    return x, P

log_dir = '../MASTER_LOGS-forivan/'
params_dir = '../lqr_params/'
measurements = np.loadtxt(log_dir + 'states.txt')
torques = np.loadtxt(log_dir + 'torques.txt')
dts = np.loadtxt(log_dir + 'dts.txt')[::2]

# print(measurements.shape, torques.shape, dts.shape)

N = 6

# Kalman filter
f = KalmanFilter(dim_x=N, dim_z=N, dim_u=2)

A  = np.loadtxt(params_dir + 'A.txt', delimiter=',')[:N,:N]
B = np.loadtxt(params_dir + 'B.txt', delimiter=',')[:N]
C = np.eye(N)

Q_cov = np.loadtxt(params_dir + 'Q_cov.txt')
# P = np.eye(N) * 1e5
try:
    P = np.loadtxt(params_dir + 'P.txt')
except FileNotFoundError:
    P = np.eye(6) * 1e-6

H = C
R = np.diag([1e-7, 5e-4, 4e-5, 5e-2, 1e-7, 1e-1])
Q = Q_cov[:N,:N]

x = measurements[0,:N]
fxs = []
last_time = time.time()
for z, u, dt in zip(measurements[1:], torques[:-1], dts):
    F = A*dt + np.eye(N)
    G = B*dt
    x, P = predict(x, P, F, Q, u, G)
    x, P = update(x, P, z, R, H)
    fxs.append(x)

np.savetxt(params_dir + 'P.txt', P)
np.savetxt(params_dir + 'R_cov.txt', R)
fxs = np.stack(fxs)

fig, axs = plt.subplots(nrows=N+1, ncols=1, figsize=(50, 15))
axs[0].plot(dts, label='Time Deltas')
axs[0].set_xticks([])
axs[0].legend()
for i, name in enumerate(['Position', 'Velocity', 'Pitch', 'Pitch rate', 'Yaw', 'Yaw rate']):
    axs[i+1].plot(measurements[:, i], label=f'Measured {name}')
    axs[i+1].plot(fxs[:, i], label=f'Predicted {name}')
    axs[i+1].set_ylim([np.min(measurements[:, i]), np.max(measurements[:, i])])
    axs[i+1].set_xticks([])
    axs[i+1].legend()
plt.tight_layout()
plt.savefig('comparison.png')
plt.show()