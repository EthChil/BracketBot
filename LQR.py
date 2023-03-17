
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
IMU.restoreCalibrationConstants([252, 255, 21, 0, 53, 0, 135, 1, 54, 1, 253, 1, 0, 0, 254, 255, 0, 0, 232, 3, 249])

odrv0 = odrive.find_any()

ref_time = time.time()
t2m = 0.528 #turns to meters



# K = np.array([[-3.1623,  -4.4213, -28.1290,  -8.7088, 0, 0], [0,0,0,0,1,1.2749]]) #New dynamimcs ones
# K = np.array([[-1.0000, -2.0620, -21.0476, -6.6199, -0.0000, 0.0000], [0.0000, 0.0000, 0.0000, 0.0000, 1.0000, 1.2749]]) #swapped Q's
# K = np.array([[-10.0000, -10.6250, -45.9842, -13.8316, -0.0000, -0.0000], [0.0000, 0.0000, 0.0000, 0.0000, 3.1623, 2.4132]]) #stronger R's set to 1
# K = np.array([[-4.4721, -5.7066, -31.9805, -9.8143, -0.0000, -0.0000], [0.0000, 0.0000, 0.0000, 0.0000, 1.4142, 1.5353]]) #strogner R's  set to 5
K = np.array([[-5.77, -6.92, -35.52, -10.83, -0.00, 0.00],[-0.00, -0.00, -0.00, 0.00, 1.83, 1.77]]) #stronger R's set to 3



def LQR(axis0, axis1):
    # read initial values to offset
    x_init = axis0.get_pos_turns() * t2m

    pitch_angle=IMU.getPitchAngle() 
    yaw_angle=IMU.getYawAngle()
    pitch_rate=IMU.getPitchRate()
    yaw_rate=IMU.getYawRate()

    Xf = np.array([0, 0, 0, 0, 0, 0])

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

    Cl_commands = []
    Cr_commands = []

    while cur_time < 60:
        time.sleep(0.001)

        # if cur_time > 3:
        #     Xf = np.array([0.0 + (cur_time-3)*0.2, 0.0, 0, 0])

        #stop the program if the torque or vel gets super high
        if abs(axis0.get_torque_input()) > 10:
            print("torque too high: ",axis0.get_torque_input(),"Nm")
            break
        if abs(axis0.get_vel()) > 3:
            print("velocity too high: ",axis0.get_vel(),"turns/s")
            break

        cur_time = time.time() - start_time # relative time starts at 0
        dt = cur_time - prev_time

        x = axis0.get_pos_turns() * t2m - x_init
        v = axis0.get_vel() * t2m
        print("raw x: ", axis0.get_pos_turns())
        print("raw vel: ", axis0.get_vel())
        print("calc x: ", x)
        print("calc v: ", v)

        pitch_angle= IMU.getPitchAngle() 
        yaw_angle=IMU.getYawAngle()
        pitch_rate= IMU.getPitchRate()
        yaw_rate=-IMU.getYawRate()

        X = np.array([x, v, pitch_angle, pitch_rate, yaw_angle, yaw_rate])

        # U = -K @ (X - Xf)

        D = np.array([[0.5, 0.5],[0.5, -0.5]])
        # print("K mat: ", K)
        # print("X mat: ", X)
        # print("Xf mat: ", Xf)
        # print("D mat:", D)
        Cl, Cr = D @ (-K @ (X - Xf))
        # print("Cl: ", Cl)
        # print("Cr: ", Cr)


        axis0.set_trq(Cl)
        axis1.set_trq(Cr)

        times.append(cur_time)
        dts.append(dt)
        xs.append(x)
        vs.append(v)

        pitchAngles.append(pitch_angle)
        yawAngles.append(yaw_angle)
        pitchRates.append(pitch_rate)
        yawRates.append(yaw_rate)

        Cl_commands.append(Cl)
        Cr_commands.append(Cr)

        prev_time = cur_time

    brake_both_motors(a0, a1)

    fig, axs = plt.subplots(nrows=8, ncols=1, figsize=(8, 16))

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
    axs[3].plot(times, pitchAngles, label='Pitch Angles')
    axs[3].legend()
    axs[3].set_title("Pitch Angles")

    # Plot 6
    axs[4].plot(times, pitchRates, label='pitch rates')
    axs[4].legend()
    axs[4].set_title("pitch rates")

    # Plot 7
    axs[5].plot(times, yawAngles, label='Yaw Angles')
    axs[5].legend()
    axs[5].set_title("Yaw Angles")

    # Plot 7
    axs[6].plot(times, yawRates, label='Yaw Rates')
    axs[6].legend()
    axs[6].set_title("Yaw Rates")

    axs[7].plot(times, Cl_commands, label='torque right')
    axs[7].plot(times, Cr_commands, label='torque left')
    axs[7].legend()
    axs[7].set_title("Balance Torques")

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
