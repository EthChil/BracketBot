import numpy as np
import matplotlib.pyplot as plt
import os

# Load the NumPy files
x_data = np.load(os.getcwd()+"/orbslam_independant_files/ego_xs.npy")
y_data = np.load(os.getcwd()+"/orbslam_independant_files/ego_ys.npy")

# Separate values and timestamps
x_values, x_timestamps = x_data[:, 0], x_data[:, 1]
y_values, y_timestamps = y_data[:, 0], y_data[:, 1]

fig, axs = plt.subplots(nrows=1, ncols=1, figsize=(8, 16))

# Plot 1
axs.plot(y_values, x_values, label='ego')
axs.set_title("dts")
plt.tight_layout()
plt.savefig("ego.png")
plt.clf()

