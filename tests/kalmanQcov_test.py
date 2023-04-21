import numpy as np

# Assuming you have state and control input data:
states = np.loadtxt('states.txt')      # Load state data
control_inputs = np.loadtxt('torques.txt')  # Load control input data

n = states.shape[1]  # Number of states
N = states.shape[0]  # Number of data points
residuals = np.zeros((N - 1, n))

A = np.loadtxt('lqr_params/A.txt', delimiter=',')
B = np.loadtxt('lqr_params/B.txt', delimiter=',')

for t in range(N - 1):
    x_t = states[t]
    x_t1_true = states[t + 1]
    u_t = control_inputs[t]

    # Predict next state using dynamic model
    x_t1_pred = A @ x_t + B @ u_t

    # Calculate residual (difference between true and predicted state)
    residual = x_t1_true - x_t1_pred
    residuals[t] = residual

# Estimate Q by calculating the covariance of residuals
Q_estimated = np.cov(residuals, rowvar=False)
