
import odrive
from odrive.enums import *
import time
import matplotlib.pyplot as plt
import numpy as np
import IMU

from odriveDriver import Axis
plt.switch_backend('Agg')

plot_dir = './plots/'

IMU = IMU.IMU(0, 40)
IMU.setupIMU()

odrv0 = odrive.find_any()

ref_time = time.time()
t2m = 0.528 #turns to meters



K = np.array([-3.1623,  -4.4213, -28.1290,  -8.7088]) #New dynamimcs ones





def LQR(axis0, axis1):
    # read initial values to offset
    x_init = axis0.get_pos_turns() * t2m

    pitch_angle=IMU.getPitchAngle() 
    yaw_angle=IMU.getYawAngle()
    pitch_rate=IMU.getPitchRate()
    yaw_rate=IMU.getYawRate()

    Xf = np.array([0.0, 0.0, 0, 0])

    start_time = time.time()
    cur_time = time.time() - start_time
    prev_time = time.time() - start_time

    # SAVE THINGS TO PLOT
    times = []
    dts = []
    xs = []
    vs = []
    dxdts = []

    pitchAngles = []
    yawAngles = []
    pitchRates = []
    yawRates = []

    torque_commands = []

    while cur_time < 30:
        time.sleep(0.001)

        # if cur_time > 3:
        #     Xf = np.array([0.0 + (cur_time-3)*0.2, 0.0, 0, 0])

        #stop the program if the vel gets super high
        if abs(axis0.get_vel()) > 2:
            break

        cur_time = time.time() - start_time # relative time starts at 0
        dt = cur_time - prev_time

        x = axis0.get_pos_turns() * t2m - x_init
        v = axis0.get_vel() * t2m

        pitch_angle=IMU.getPitchAngle() 
        yaw_angle=IMU.getYawAngle()
        pitch_rate=-IMU.getPitchRate()
        yaw_rate=IMU.getYawRate()

        X = np.array([x, v, pitch_angle, pitch_rate])

        U = -K @ (X - Xf)

        torque_command = U / 2

        axis0.set_trq(torque_command)
        axis1.set_trq(torque_command)

        times.append(cur_time)
        dts.append(dt)
        xs.append(x)
        vs.append(v)

        pitchAngles.append(pitch_angle)
        yawAngles.append(yaw_angle)
        pitchRates.append(pitch_rate)
        yawRates.append(yaw_rate)

        torque_commands.append(torque_command)

        prev_time = cur_time

    brake_both_motors(a0, a1)

    fig, axs = plt.subplots(nrows=9, ncols=1, figsize=(8, 16))

    # Plot 1
    axs[0].plot(times, dts, label='dts')
    axs[0].legend()
    axs[0].set_title("dts")

    # Plot 2
    axs[1].plot(times, xs, label='xs')
    axs[1].legend()
    axs[1].set_title("xs")

    # Plot 3
    axs[2].plot(times, vs, label='vs')
    axs[2].legend()
    axs[2].set_title("vs")

    # Plot 5
    axs[4].plot(times, pitchAngles, label='Pitch Angles')
    axs[4].legend()
    axs[4].set_title("Pitch Angles")

    # Plot 6
    axs[5].plot(times, pitchRates, label='pitch rates')
    axs[5].legend()
    axs[5].set_title("pitch rates")

    # Plot 7
    axs[6].plot(times, yawAngles, label='Yaw Angles')
    axs[6].legend()
    axs[6].set_title("Yaw Angles")

    # Plot 7
    axs[7].plot(times, yawRates, label='Yaw Rates')
    axs[7].legend()
    axs[7].set_title("Yaw Rates")

    axs[8].plot(times, torque_commands, label='torque')
    axs[8].legend()
    axs[8].set_title("Balance Torque")

    plt.tight_layout()
    plt.savefig(plot_dir+ "balance_plots.png")
    plt.clf()



def brake_both_motors(axis0, axis1):

    while abs(axis0.get_vel()) > 0.05 and abs(axis1.get_vel()) > 0.05:
        axis0.set_trq(-0.5 if axis0.get_vel() > 0 else 0.5)
        axis1.set_trq(-0.5 if axis1.get_vel() > 0 else 0.5)
        
    axis0.set_trq(0)
    axis1.set_trq(0)

        
a0 = Axis(odrv0.axis0, dir=1)
a1 = Axis(odrv0.axis1, dir=-1)

a0.setup()
a1.setup()

LQR(a0, a1)

brake_both_motors(a0, a1)




