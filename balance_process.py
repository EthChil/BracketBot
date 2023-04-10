
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

ref_time = time.time()
t2m = 0.528 #turns to meters

#with ivans fix
#Q = diag([100 1 10 1 10 1]); % 'x', 'v', 'θ', 'ω', 'δ', "δ'
#R = diag([3 3]); % Torque cost Cθ,Cδ
K = np.array([[-5.77, -7.74, -42.86, -16.89, 0.00, 0.00],[-0.00, -0.00, -0.00, -0.00, 1.83, 1.77]] )

W = 0.567   # Distance between wheels in meters

xs_ego = []
ys_ego = []
thetas_ego = []
thetas_fused = []

x, y, theta = 0, 0, 0 # Initial position and heading


def update_position(x, y, theta, d_left, d_right, W):
    distance_left = d_left
    distance_right = d_right

    distance_avg = (distance_left + distance_right) / 2
    delta_theta = (distance_right - distance_left) / W

    x += distance_avg * math.cos(theta + delta_theta / 2)
    y += distance_avg * math.sin(theta + delta_theta / 2)
    theta += delta_theta

    return x, y, theta



def LQR(axis0, axis1):
    
    IMU1 = IMU.IMU_BNO085()
    IMU2 = IMU.IMU_BNO055(0, 40)

    odrv0 = odrive.find_any()
    axis0 = Axis(odrv0.axis0, dir=1)
    axis1 = Axis(odrv0.axis1, dir=-1)
    
    pos_init_a0 = axis0.get_pos_turns() * t2m
    pos_init_a1 = axis1.get_pos_turns() * t2m

    pos_a0_cur = axis0.get_pos_turns() * t2m - pos_init_a0
    pos_a1_cur = axis1.get_pos_turns() * t2m - pos_init_a1
    pos_a0_prev = pos_a0_cur
    pos_a1_prev = pos_a1_cur
    
    # read initial values to offset
    pos_init = (axis0.get_pos_turns() * t2m + axis1.get_pos_turns() * t2m)/2

    pitch_angle1=IMU1.getPitchAngle() 
    yaw_angle1=IMU1.getYawAngle()
    pitch_rate1=IMU1.getPitchRate()
    yaw_rate1=IMU1.getYawRate()

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

    pitchAngles1 = []
    yawAngles1 = []
    pitchRates1 = []
    yawRates1 = []
    
    pitchAngles2 = []
    yawAngles2 = []
    pitchRates2 = []
    yawRates2 = []

    Cl_commands = []
    Cr_commands = []

    while cur_time < 30:
        cur_time = time.time() - start_time # relative time starts at 0
        dt = cur_time - prev_time
        
        #stop the program if the torque or vel gets super high
        if abs(axis0.get_torque_input()) > 12:
            print("torque too high: ",axis0.get_torque_input(),"Nm")
            break
        if abs(axis0.get_vel()) > 4:
            print("velocity too high: ",axis0.get_vel(),"turns/s")
            break
        
        
        # EGO MOTION
        pos_a0_cur = axis0.get_pos_turns() * t2m - pos_init_a0
        pos_a1_cur = axis1.get_pos_turns() * t2m - pos_init_a1
        
        pos_a0_delta = pos_a0_cur-pos_a0_prev
        pos_a1_delta = pos_a1_cur-pos_a1_prev
        # END EGO
        
        Xf = np.array([0, 0, 0, 0, 0, 0])

        x = (axis0.get_pos_turns() * t2m  + axis1.get_pos_turns() * t2m)/2 - pos_init
        v = (axis0.get_vel() * t2m + axis1.get_vel() * t2m)/2

        
        yaw_angle1 = -IMU1.getYawAngle() # positive for base mount
        pitch_angle1 = -IMU1.getPitchAngle() # positive for base mount
        pitch_rate1 = IMU1.getPitchRate() # negative for base mount
        yaw_rate1 = -IMU1.getYawRate() # negative for base mount
        
        yaw_angle2 = IMU2.getYawAngle() # positive for base mount
        pitch_angle2 = IMU2.getPitchAngle() # positive for base mount
        pitch_rate2 = -IMU2.getPitchRate() # negative for base mount
        yaw_rate2 = -IMU2.getYawRate() # negative for base mount
        
        
        theta_fused = (theta + yaw_angle2)/2
        
        x, y, theta = update_position(x, y, theta_fused, pos_a0_delta, pos_a1_delta, W)
        xs_ego.append(x)
        ys_ego.append(y)
        thetas_ego.append(theta)
        thetas_fused.append(theta_fused)
        
        X = np.array([x, v, pitch_angle1, pitch_rate2, yaw_angle2, yaw_rate2])

        D = np.array([[0.5, 0.5],[0.5, -0.5]])

        Cl, Cr = D @ (-K @ (X - Xf))

        
        # instead of anticogging
        if Cl > 0:
            axis0.set_trq(Cl+0.25)
        else:
            axis0.set_trq(Cl-0.25)
            
        if Cr > 0:
            axis1.set_trq(Cr+0.25)
        else:
            axis1.set_trq(Cr-0.25)
        
        # axis0.set_trq(Cl)
        # axis1.set_trq(Cr_commands)
        

        times.append(cur_time)
        dts.append(dt)
        xs.append(x)
        vs.append(v)

        pitchAngles1.append(pitch_angle1)
        yawAngles1.append(yaw_angle1)
        pitchRates1.append(pitch_rate1)
        yawRates1.append(yaw_rate1)
        
        pitchAngles2.append(pitch_angle2)
        yawAngles2.append(yaw_angle2)
        pitchRates2.append(pitch_rate2)
        yawRates2.append(yaw_rate2)

        Cl_commands.append(Cl)
        Cr_commands.append(Cr)
    
        pos_a0_prev = pos_a0_cur
        pos_a1_prev = pos_a1_cur
        prev_time = cur_time

    # brake_both_motors(a0, a1)

    # fig, axs = plt.subplots(nrows=8, ncols=1, figsize=(8, 16))

    # # Plot 1
    # axs[0].plot(times, dts, label='dts')
    # axs[0].legend()
    # axs[0].set_title("dts")

    # # Plot 2
    # axs[1].plot(times, xs, label='xs')
    # axs[1].legend()
    # axs[1].set_title("xs")

    # # Plot 3
    # axs[2].plot(times, vs, label='vs')
    # axs[2].legend()
    # axs[2].set_title("vs")

    # # Plot 5
    # axs[3].plot(times, pitchAngles1, label='Pitch Angles 85')
    # axs[3].plot(times, pitchAngles2, label='Pitch Angles 55')
    # axs[3].legend()
    # axs[3].set_title("Pitch Angles")

    # # Plot 6
    # axs[4].plot(times, pitchRates1, label='pitch rates 85')
    # axs[4].plot(times, pitchRates2, label='pitch rates 55')
    # axs[4].legend()
    # axs[4].set_title("pitch rates")

    # # Plot 7
    # axs[5].plot(times, yawAngles1, label='Yaw Angles 85')
    # axs[5].plot(times, yawAngles2, label='Yaw Angles 55')
    # axs[5].plot(times, thetas_ego, label='Yaw Angle ego')
    # axs[5].plot(times, thetas_fused, label='Yaw Angle Fused 55 Ego')
    # axs[5].legend()
    # axs[5].set_title("Yaw Angles")

    # # Plot 7
    # axs[6].plot(times, yawRates1, label='Yaw Rates 85')
    # axs[6].plot(times, yawRates2, label='Yaw Rates 55')
    # axs[6].legend()
    # axs[6].set_title("Yaw Rates")

    # axs[7].plot(times, Cl_commands, label='torque right')
    # axs[7].plot(times, Cr_commands, label='torque left')
    # axs[7].legend()
    # axs[7].set_title("Balance Torques")


    # plt.tight_layout()
    # plt.savefig(plot_dir+ "balance_plots.png")
    # plt.clf()
    
    # fig, axs = plt.subplots(nrows=1, ncols=1, figsize=(8, 16))

    # # Plot 1
    # axs.plot(ys_ego, xs_ego, label='ego')
    # axs.set_title("dts")
    # plt.tight_layout()
    # plt.savefig(plot_dir+ "ego.png")
    # plt.clf()
