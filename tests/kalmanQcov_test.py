import numpy as np
import matplotlib.pyplot as plt

# Assuming you have state and control input data:
states = np.loadtxt('../states.txt')      # Load state data
control_inputs = np.loadtxt('../torques.txt')  # Load control input data

n = states.shape[1]  # Number of states
N = states.shape[0]  # Number of data points
residuals = np.zeros((N - 1, n))
preds = np.zeros((N - 1, n))

A = np.loadtxt('../lqr_params/A.txt', delimiter=',')
B = np.loadtxt('../lqr_params/B.txt', delimiter=',')
dt = 0.006

for t in range(N - 1):
    x_t = states[t]
    x_t1_true = states[t + 1]
    u_t = control_inputs[t]

    # Predict next state using dynamic model
    x_t1_pred = x_t + (A @ x_t + B @ u_t) * dt
    preds[t] = x_t1_pred 

    # Calculate residual (difference between true and predicted state)
    residual = x_t1_true - x_t1_pred
    residuals[t] = residual

# Estimate Q by calculating the covariance of residuals
Q_estimated = np.cov(residuals, rowvar=False)
np.savetxt("Q_cov.txt", Q_estimated)


fig, axs = plt.subplots(nrows=6, ncols=1, figsize=(100, 30))
for i, name in enumerate(['Position', 'Velocity', 'Pitch', 'Pitch rate', 'Yaw', 'Yaw rate']):
    axs[i].set_title(name)
    axs[i].plot(residuals[:, i], label=f'Residual {name}')
    axs[i].plot(states[:, i], label=f'Measured {name}')
    axs[i].set_ylim([np.min(states[:, i]), np.max(states[:, i])])
    axs[i].set_xticks([])
    axs[i].legend()
plt.tight_layout()
plt.savefig('residuals.png')


print(np.sum(residuals**2) / np.sum((states[1:]-states[:-1])**2))