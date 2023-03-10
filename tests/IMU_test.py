import sys
sys.path.append('../')
import IMU
import IMU
import time
import matplotlib.pyplot as plt
import math


IMU = IMU.IMU(0, 40)
IMU.setupIMU()

values = []
times = []
valuesRaw1 = []
valuesRaw2 = []

plot_dir = './plots/'

plt.switch_backend('Agg')

stamp = time.time()
while (time.time() - stamp < 20):
    time.sleep(0.005)
    # print(IMU.getAngleCompRads())
    # print(f'{IMU.getWogma()[0]:.3f} | {IMU.getWogma()[1]:.3f} | {IMU.getWogma()[2]:.3f}')
    # print(IMU.getWogma()[0], )

    # angle = IMU.getAngle()
    # wogma = IMU.getWogmaPitch()
    # wogma_yaw = IMU.getWogmaYaw()
    angleYaw = IMU.getYaw()



    # print(angle)
    print(angleYaw)

    # values.append(angle[0])
    # valuesRaw1.append(angle[1])
    # valuesRaw2.append(angle[2])
    # times.append(time.time() - stamp)

fig, axs = plt.subplots(nrows=3, ncols=1, figsize=(8, 16))

# Plot 1
axs[0].plot(times, values, label='ang')
axs[0].legend()
axs[0].set_title("ang")

axs[1].plot(times, valuesRaw1, label='Y RAW')
axs[1].legend()
axs[1].set_title("Y RAW")

axs[2].plot(times, valuesRaw2, label='Z RAW')
axs[2].legend()
axs[2].set_title("Z RAW")

plt.tight_layout()
plt.savefig(plot_dir+ "IMU_PLOT.png")
plt.clf()

