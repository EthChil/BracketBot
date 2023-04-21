import numpy as np
import matplotlib.pyplot as plt

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from scipy.signal import butter, lfilter, filtfilt, savgol_filter, gaussian, convolve

def moving_average_filter(data, window_size):
    cumsum = np.cumsum(np.insert(data, 0, 0))
    return (cumsum[window_size:] - cumsum[:-window_size]) / window_size

def exponential_moving_average_filter(data, alpha):
    filtered_data = np.zeros_like(data)
    filtered_data[0] = data[0]
    for i in range(1, len(data)):
        filtered_data[i] = alpha * data[i] + (1 - alpha) * filtered_data[i - 1]
    return filtered_data

def butter_lowpass_filter_online(data, zi, cutoff, fs, order):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    y, zf = lfilter(b, a, data, zi=zi)
    return y, zf

def gaussian_filter(data, window_size, std_dev):
    window = gaussian(window_size, std=std_dev)
    return convolve(data, window / window.sum(), mode='same')

measurements = np.loadtxt('../states.txt')
torques = np.loadtxt('../torques.txt')

# Select pitch_rate and yaw_rate data
pitch_rate_data = measurements[1:, 3]
yaw_rate_data = measurements[1:, 5]

# Moving average filter
window_size = 5
ma_filtered_pitch_rate = moving_average_filter(pitch_rate_data, window_size)
ma_filtered_yaw_rate = moving_average_filter(yaw_rate_data, window_size)

# Exponential moving average filter
alpha = 0.1
ema_filtered_pitch_rate = exponential_moving_average_filter(pitch_rate_data, alpha)
ema_filtered_yaw_rate = exponential_moving_average_filter(yaw_rate_data, alpha)

# Gaussian filter
gaussian_window_size = 5
std_dev = 4
gaussian_filtered_pitch_rate = gaussian_filter(pitch_rate_data, gaussian_window_size, std_dev)
gaussian_filtered_yaw_rate = gaussian_filter(yaw_rate_data, gaussian_window_size, std_dev)

# Online Butterworth low-pass filter
cutoff = 5  # Choose a suitable cutoff frequency (Hz)
fs = 100  # Sampling frequency (Hz)
order = 12  # Choose a suitable filter order
b, a = butter(order, cutoff / (0.5 * fs), btype='low', analog=False)
zi = np.zeros((order, len(pitch_rate_data) - 1))
butter_filtered_pitch_rate_online = np.zeros_like(pitch_rate_data)
butter_filtered_yaw_rate_online = np.zeros_like(yaw_rate_data)

for i, (pitch_rate, yaw_rate) in enumerate(zip(pitch_rate_data, yaw_rate_data)):
    if i == 0:
        continue
    butter_filtered_pitch_rate_online[i - 1], zi[:, i - 1] = butter_lowpass_filter_online(np.array([pitch_rate]), zi[:, i - 2], cutoff, fs, order)
    butter_filtered_yaw_rate_online[i - 1], zi[:, i - 1] = butter_lowpass_filter_online(np.array([yaw_rate]), zi[:, i - 2], cutoff, fs, order)
    
# Savitzky-Golay filter
window_length = 21  # Must be odd
polyorder = 11  # Polynomial order
sg_filtered_pitch_rate = savgol_filter(pitch_rate_data, window_length, polyorder)
sg_filtered_yaw_rate = savgol_filter(yaw_rate_data, window_length, polyorder)

# Kalman filter
f = KalmanFilter(dim_x=6, dim_z=6, dim_u=2)

A  = np.loadtxt('../lqr_params/A.txt', delimiter=',')
B = np.loadtxt('../lqr_params/B.txt', delimiter=',')

Q_cov = np.loadtxt('Q_cov.txt')

f.F = A
f.B = B
f.H = np.eye(6)
f.P = np.eye(6)*1000
f.R = np.diag([1e-6, 1e-6, 1e-4, 0.1, 1e-4, 0.1])
f.Q = Q_cov

f.x = measurements[0]
fxs = []
for z, u in zip(measurements[1:], torques[:-1]):
    f.predict(u)
    f.update(z)
    fxs.append(f.x)
fxs = np.stack(fxs)

# Extract filtered pitch rate and yaw rate from Kalman filter results
kalman_filtered_pitch_rate = fxs[:, 3]
kalman_filtered_yaw_rate = fxs[:, 5]

# Plot pitch rate and yaw rate with different filters
fig, axs = plt.subplots(nrows=2, ncols=1, figsize=(100, 30))

axs[0].plot(pitch_rate_data, label='Measured Pitch Rate')
axs[0].plot(kalman_filtered_pitch_rate, label='Kalman Filter')
axs[0].plot(ma_filtered_pitch_rate, label='Moving Average Filter')
axs[0].plot(ema_filtered_pitch_rate, label='Exponential Moving Average Filter')
axs[0].plot(gaussian_filtered_pitch_rate, label='Gaussian Filter')
axs[0].plot(butter_filtered_pitch_rate_online, label='Online Butterworth Low-pass Filter')
axs[0].plot(sg_filtered_pitch_rate, label='Savitzky-Golay Filter')
axs[0].set_title('Pitch Rate')
axs[0].legend()

axs[1].plot(yaw_rate_data, label='Measured Yaw Rate')
axs[1].plot(kalman_filtered_yaw_rate, label='Kalman Filter')
axs[1].plot(ma_filtered_yaw_rate, label='Moving Average Filter')
axs[1].plot(ema_filtered_yaw_rate, label='Exponential Moving Average Filter')
axs[1].plot(gaussian_filtered_yaw_rate, label='Gaussian Filter')
axs[1].plot(butter_filtered_yaw_rate_online, label='Online Butterworth Low-pass Filter')
axs[1].plot(sg_filtered_yaw_rate, label='Savitzky-Golay Filter')
axs[1].set_title('Yaw Rate')
axs[1].legend()

plt.tight_layout()
plt.savefig('comparison.png')
# plt.show()
