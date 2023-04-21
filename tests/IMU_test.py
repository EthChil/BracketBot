import sys
sys.path.append('../')
import IMU
import time
import matplotlib.pyplot as plt
import math


IMU1 = IMU.IMU_BNO085() # 1, 0, 0 for top mounting on front/back of extrusion, 0, 0, 1 for conventional mount
IMU1.setupIMU()
IMU1.restoreCalibrationConstants([0, 0, 85, 0, 1, 0, 166, 1, 77, 1, 176, 1, 1, 0, 0, 0, 0, 0, 232, 3, 178, 1])

IMU2 = IMU.IMU_BNO055(0, 40) # 1, 0, 0 for top mounting on front/back of extrusion, 0, 0, 1 for conventional mount
IMU2.setupIMU()
IMU2.restoreCalibrationConstants([0, 0, 85, 0, 1, 0, 166, 1, 77, 1, 176, 1, 1, 0, 0, 0, 0, 0, 232, 3, 178, 1])

plot_dir = './plots/'

plt.switch_backend('Agg')

times = []
pitchAngles1 = []
pitchRates1 = []
yawAngles1 = []
yawRates1 = []

pitchAngles2 = []
pitchRates2 = []
yawAngles2 = []
yawRates2 = []

stamp = time.time()
while (time.time() - stamp < 100):
    time.sleep(0.05)

    #bno085
    wogmaPitch1 = -IMU1.getPitchRate()
    wogmaYaw1 = IMU1.getYawRate()
    angleYaw1 = IMU1.getYawAngle()
    anglePitch1 = -IMU1.getPitchAngle()
    
    pitchAngles1.append(anglePitch1)
    pitchRates1.append(wogmaPitch1)
    yawAngles1.append(angleYaw1)
    yawRates1.append(wogmaYaw1)
    
    #bno055
    wogmaPitch2 = IMU2.getPitchRate()
    wogmaYaw2 = IMU2.getYawRate()
    angleYaw2 = IMU2.getYawAngle()
    anglePitch2 = IMU2.getPitchAngle()
    
    pitchAngles2.append(anglePitch2)
    pitchRates2.append(wogmaPitch2)
    yawAngles2.append(angleYaw2)
    yawRates2.append(wogmaYaw2)
    
    print(math.degrees(anglePitch1), math.degrees(anglePitch2), "pitch")
    
    
    times.append(time.time() - stamp)


fig, axs = plt.subplots(nrows=2, ncols=1, figsize=(8, 16))
axs[0].plot(times, pitchAngles1, label='Pitch Angles 85')
axs[0].plot(times, pitchAngles2, label='Pitch Angles 55')
axs[0].legend()
axs[0].set_title("Pitch Angles")

axs[1].plot(times, pitchRates1, label='Pitch Angle Rates 85')
axs[1].plot(times, pitchRates2, label='Pitch Angle Rates 55')
axs[1].legend()
axs[1].set_title("Pitch Rates")



plt.tight_layout()
plt.savefig(plot_dir + "IMU_Plot.png")
plt.clf()

