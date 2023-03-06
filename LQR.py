
import odrive
from odrive.enums import *
import time
import matplotlib.pyplot as plt
import numpy as np
import IMU_uncool


from odriveDriver import Axis
plt.switch_backend('Agg')

plot_dir = './plots/'

IMU = IMU_uncool.IMU(0, 40)
IMU.setupIMU()

odrv0 = odrive.find_any()

ref_time = time.time()
t2m = 0.528 #turns to meters

# K = np.array([-31.6228, -30.5921, -92.5864, -32.4303])
# K = np.array([-10.0000, -11.3648, -43.2605, -15.2819]) #working ones
# K = np.array([-31.6228, -30.5921, -92.5864, -32.4303])
# K = np.array([-1.0000,  -2.0552, -17.2178,  -6.2506]) #smaller torques
K = np.array([-3.1623,  -4.5965, -24.7847,  -8.8673]) #medium torques higher position objective


def LQR(axis0, axis1):
    # read initial values to offset
    x_init = axis0.get_pos_turns() * t2m
    cur_theta = IMU.getAngle()
    prev_theta = cur_theta
    wogma = 0
    prev_x = 0

    # Initialize running weighted average filter for wogma
    alpha = 1 # Weight of the new measurement (0 < alpha < 1)
    wogma_filtered = wogma

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
    cur_thetas = []
    wogmas = []
    filtered_wogmas = []
    wogma_from_IMUs = []
    torque_commands = []

    while cur_time < 10:
        # if cur_time > 3:
        #     Xf = np.array([0.0 + (cur_time-3)*0.2, 0.0, 0, 0])

        time.sleep(0.001)
        cur_time = time.time() - start_time # relative time starts at 0
        dt = cur_time - prev_time

        x = axis0.get_pos_turns() * t2m - x_init
        v = axis0.get_vel() * t2m
        cur_theta = IMU.getAngle()
        wogma = (cur_theta - prev_theta) / dt
        wogma_from_IMU = IMU.getWogma()
        
        wogma_filtered = alpha * wogma + (1 - alpha) * wogma_filtered

        X = np.array([x, v, cur_theta, wogma_from_IMU])

        U = -K @ (X - Xf)

        torque_command = U / 2 # if (U / 2) < 3 else 3

        axis0.set_trq(torque_command)
        axis1.set_trq(torque_command)

        times.append(cur_time)
        dts.append(dt)
        xs.append(x)
        vs.append(v)
        dxdts.append((x - prev_x) / dt)
        cur_thetas.append(cur_theta)
        wogmas.append(wogma)
        filtered_wogmas.append(wogma_filtered)
        wogma_from_IMUs.append(wogma_from_IMU)
    
        torque_commands.append(torque_command)

        prev_theta = cur_theta
        prev_time = cur_time
        prev_x = x

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

    # Plot 4
    axs[3].plot(times, dxdts, label='dxdts')
    axs[3].legend()
    axs[3].set_title("dxdts")

    # Plot 5
    axs[4].plot(times, cur_thetas, label='cur_thetas')
    axs[4].legend()
    axs[4].set_title("cur_thetas")

    # Plot 6
    axs[5].plot(times, wogmas, label='wogmas')
    axs[5].legend()
    axs[5].set_title("wogmas")

    # Plot 7
    axs[6].plot(times, filtered_wogmas, label='filtered_wogmas')
    axs[6].legend()
    axs[6].set_title("filtered_wogmas")

    # Plot 7
    axs[7].plot(times, wogma_from_IMUs, label='IMU wogma')
    axs[7].legend()
    axs[7].set_title("IMU wogmas")

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

        
a0 = Axis(odrv0.axis0)
a1 = Axis(odrv0.axis1, dir=-1)

a0.setup()
a1.setup()

LQR(a0, a1)

brake_both_motors(a0, a1)




