import sys
sys.path.append('../')
import IMU
import time
import matplotlib.pyplot as plt
import math


IMU1 = IMU.IMU_BNO085() # 1, 0, 0 for top mounting on front/back of extrusion, 0, 0, 1 for conventional mount
IMU1.setupIMU()

plot_dir = './plots/'

plt.switch_backend('Agg')

times = []
dts = []
pitchAngles1 = []
pitchRates1 = []
yawAngles1 = []
yawRates1 = []

start_time = time.time()
cur_time = 0
prev_time = 0
while (time.time() - start_time < 10):
    cur_time = time.time() - start_time # relative time starts at 0
    dt = cur_time - prev_time

    #bno085
    pitch_rate1, yaw_rate1 = IMU1.getRates()
    # pitch_rate1, yaw_rate1 = [0,0]
    pitch_angle1, yaw_angle1 = IMU1.getAngles()
    # pitch_angle1, yaw_angle1 = [0,0]
    
    pitchAngles1.append(-pitch_angle1)
    pitchRates1.append(pitch_rate1)
    yawAngles1.append(-yaw_angle1)
    yawRates1.append(-yaw_rate1)
    
    prev_time = cur_time
    times.append(cur_time)
    dts.append(dt)


fig, axs = plt.subplots(nrows=3, ncols=1, figsize=(100, 20))
axs[0].plot(times, pitchAngles1, label='Pitch Angles 85')
axs[0].legend()
axs[0].set_title("Pitch Angles")

axs[1].plot(times, pitchRates1, label='Pitch Angle Rates 85')
axs[1].legend()
axs[1].set_title("Pitch Rates")

axs[2].plot(times, dts, label='dt')
axs[2].legend()
axs[2].set_title("DTs")



plt.tight_layout()
plt.savefig(plot_dir + "IMU_speed_test.png")
plt.clf()

