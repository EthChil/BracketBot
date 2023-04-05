
import odrive
from odrive.enums import *
import time
import matplotlib.pyplot as plt
import numpy as np
import IMU
import math

from odriveDriver import Axis
plt.switch_backend('Agg')

plot_dir = './plots/'

IMU1 = IMU.IMU_BNO085()
# IMU.restoreCalibrationConstants([0, 0, 85, 0, 1, 0, 166, 1, 77, 1, 176, 1, 1, 0, 0, 0, 0, 0, 232, 3, 178, 1]) only for bno55
IMU1.setupIMU()

IMU2 = IMU.IMU_BNO055(0, 40)
IMU2.restoreCalibrationConstants([0, 0, 85, 0, 1, 0, 166, 1, 77, 1, 176, 1, 1, 0, 0, 0, 0, 0, 232, 3, 178, 1]) #only for bno55
IMU2.setupIMU()

odrv0 = odrive.find_any()

ref_time = time.time()
t2m = 0.528 #turns to meters


#with ivans fix
#Q = diag([100 1 10 1 10 1]); % 'x', 'v', 'θ', 'ω', 'δ', "δ'
#R = diag([3 3]); % Torque cost Cθ,Cδ
K = np.array([[-5.77, -7.74, -42.86, -16.89, 0.00, 0.00],[-0.00, -0.00, -0.00, -0.00, 1.83, 1.77]] )


#not care about pos
#Q = diag([1 1 10 1 10 1]); % 'x', 'v', 'θ', 'ω', 'δ', "δ'
#R = diag([3 3]); % Torque cost Cθ,Cδ
# K = np.array([[-0.58, -2.57, -26.38, -9.95, 0.00, 0.00],[0.00, 0.00, 0.00, 0.00, 1.83, 1.77]] )


# K = np.array([[-10.00, -13.15, -64.11, -25.50, 0.00, 0.00],[-0.00, -0.00, -0.00, -0.00, 3.16, 2.41]] )





def LQR(axis0, axis1):
    # read initial values to offset
    x_init = (axis0.get_pos_turns() * t2m + axis1.get_pos_turns() * t2m)/2
    
    pos_init = (axis0.get_pos_turns() * t2m + axis1.get_pos_turns() * t2m)/2

    pitch_angle=IMU1.getPitchAngle() 
    yaw_angle=IMU1.getYawAngle()
    pitch_rate=IMU1.getPitchRate()
    yaw_rate=IMU1.getYawRate()

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
    
    pitchAngles2 = []
    yawAngles2 = []
    pitchRates2 = []
    yawRates2 = []

    Cl_commands = []
    Cr_commands = []

    while cur_time < 20:
        time.sleep(0.0001)
        
        cur_time = time.time() - start_time # relative time starts at 0
        dt = cur_time - prev_time
        
        #stop the program if the torque or vel gets super high
        if abs(axis0.get_torque_input()) > 12:
            print("torque too high: ",axis0.get_torque_input(),"Nm")
            break
        if abs(axis0.get_vel()) > 4:
            print("velocity too high: ",axis0.get_vel(),"turns/s")
            break
        
        Xf = np.array([0, 0, 0, 0, 0, 0])

        x = (axis0.get_pos_turns() * t2m  + axis1.get_pos_turns() * t2m)/2 - pos_init
        v = (axis0.get_vel() * t2m + axis1.get_vel() * t2m)/2

        
        yaw_angle = -IMU1.getYawAngle() # positive for base mount
        pitch_angle = -IMU1.getPitchAngle() # positive for base mount
        pitch_rate = IMU1.getPitchRate() # negative for base mount
        yaw_rate = -IMU1.getYawRate() # negative for base mount
        
        yaw_angle2 = IMU2.getYawAngle() # positive for base mount
        pitch_angle2 = IMU2.getPitchAngle() # positive for base mount
        pitch_rate2 = -IMU2.getPitchRate() # negative for base mount
        yaw_rate2 = -IMU2.getYawRate() # negative for base mount
        
        

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
        
        pitchAngles2.append(pitch_angle2)
        yawAngles2.append(yaw_angle2)
        pitchRates2.append(pitch_rate2)
        yawRates2.append(yaw_rate2)

        Cl_commands.append(Cl)
        Cr_commands.append(Cr)
    
        
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
    axs[3].plot(times, pitchAngles, label='Pitch Angles 85')
    axs[3].plot(times, pitchAngles2, label='Pitch Angles 55')
    axs[3].legend()
    axs[3].set_title("Pitch Angles")

    # Plot 6
    axs[4].plot(times, pitchRates, label='pitch rates 85')
    axs[4].plot(times, pitchRates2, label='pitch rates 55')
    axs[4].legend()
    axs[4].set_title("pitch rates")

    # Plot 7
    axs[5].plot(times, yawAngles, label='Yaw Angles 85')
    axs[5].plot(times, yawAngles2, label='Yaw Angles 55')
    axs[5].legend()
    axs[5].set_title("Yaw Angles")

    # Plot 7
    axs[6].plot(times, yawRates, label='Yaw Rates 85')
    axs[6].plot(times, yawRates2, label='Yaw Rates 55')
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
