import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt("data.txt", delimiter=",")

pitch = data[:, 0]
yaw = data[:, 1]
roll = data[:, 2]

plt.plot(pitch)
plt.plot(yaw)
plt.plot(roll)

plt.xlabel("X-axis")
plt.ylabel("Y-axis")
plt.title("filtered_data")
plt.savefig('filtered_data.png')
