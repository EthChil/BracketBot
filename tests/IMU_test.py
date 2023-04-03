import sys
sys.path.append('../')
import IMU
import time
import matplotlib.pyplot as plt
import math


IMU = IMU.IMU_BNO085() # 1, 0, 0 for top mounting on front/back of extrusion, 0, 0, 1 for conventional mount
IMU.setupIMU()
IMU.restoreCalibrationConstants([0, 0, 85, 0, 1, 0, 166, 1, 77, 1, 176, 1, 1, 0, 0, 0, 0, 0, 232, 3, 178, 1])

plot_dir = './plots/'

plt.switch_backend('Agg')

times = []
pitchAngles = []
pitchRates = []

stamp = time.time()
while (time.time() - stamp < 200):
    time.sleep(0.1)
    # print(IMU.getAngleCompRads())
    # print(f'{IMU.getWogma()[0]:.3f} | {IMU.getWogma()[1]:.3f} | {IMU.getWogma()[2]:.3f}')
    # print(IMU.getWogma()[0], )

    # angle = IMU.getAngle()
    wogma = IMU.getPitchRate()
    wogma_yaw = IMU.getYawRate()
    angleYaw = IMU.getYawAngle()
    anglePitch = IMU.getPitchAngle()

    # print(angleYaw)
    # print(IMU.getCalibStatus())
    times.append(time.time() - stamp)
    pitchAngles.append(anglePitch)
    pitchRates.append(wogma)

    print(math.degrees(anglePitch))
    

fig, axs = plt.subplots(nrows=2, ncols=1, figsize=(8, 16))
axs[0].plot(times, pitchAngles, label='Pitch Angles')
axs[0].legend()
axs[0].set_title("Pitch Angles")
axs[1].plot(times, pitchRates, label='Pitch Rates')
axs[1].legend()
axs[1].set_title("Pitch Rates")


plt.tight_layout()
plt.savefig(plot_dir + "IMU_Plot.png")
plt.clf()

