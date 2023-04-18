import moteusDriver

import time
import matplotlib.pyplot as plt
import numpy as np
import IMU
import math

plt.switch_backend('Agg')

plot_dir = './plots/'

IMU1 = IMU.IMU_BNO085()
# IMU.restoreCalibrationConstants([0, 0, 85, 0, 1, 0, 166, 1, 77, 1, 176, 1, 1, 0, 0, 0, 0, 0, 232, 3, 178, 1]) only for bno55
IMU1.setupIMU()

# IMU2 = IMU.IMU_BNO055(0, 40)
# IMU2.restoreCalibrationConstants([0, 0, 85, 0, 1, 0, 166, 1, 77, 1, 176, 1, 1, 0, 0, 0, 0, 0, 232, 3, 178, 1]) #only for bno55
# IMU2.setupIMU()

moteus1 = moteusDriver.MoteusController(device_id=1, direction=1)
moteus1.stop()

moteus2 = moteusDriver.MoteusController(device_id=2, direction=1)
moteus2.stop()


ref_time = time.time()
t2m = 0.528 #turns to meters
c2m = t2m/8192



#Q = diag([50 30 0.1 0.01 10 1]); % 'x', 'v', 'θ', 'ω', 'δ', "δ'
#R = diag([3 3]); % Torque cost Cθ,Cδ
K = np.array([[-4.08, -8.11, -52.57, -25.27, -0.00, 0.00],[0.00, 0.00, 0.00, 0.00, 1.83, 1.77]]  )



W = 0.567   # Distance between wheels in meters


pos_init_a0 = moteus1.get_position() * t2m
pos_init_a1 = moteus2.get_position() * t2m

pos_a0_cur = moteus1.get_position() * t2m - pos_init_a0
pos_a1_cur = moteus2.get_position() * t2m - pos_init_a1
pos_a0_prev = pos_a0_cur
pos_a1_prev = pos_a1_cur

xs_ego = []
ys_ego = []
thetas_ego = []
thetas_fused = []

x_ego, y, theta = 0, 0, 0 # Initial position and heading


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
    # read initial values to offset
    x_init = (moteus1.get_position() * t2m + moteus2.get_position() * t2m)/2
    pos_init = (moteus1.get_position() * t2m + moteus2.get_position() * t2m)/2
    

    pitch_angle1=IMU1.getPitchAngle() 
    yaw_angle1=IMU1.getYawAngle()
    pitch_rate1=IMU1.getPitchRate()
    yaw_rate1=IMU1.getYawRate()
    
    start_angle = -yaw_angle1
    

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
    
    moteus1.stop()
    moteus2.stop()
    
    while cur_time < 30:
        # time.sleep(0.0001)
        
        cur_time = time.time() - start_time # relative time starts at 0
        dt = cur_time - prev_time
        
        a0_pos_turns = moteus1.get_position()
        a1_pos_turns = moteus2.get_position()
        
        a0_vel_turns = moteus1.get_velocity()
        a1_vel_turns = moteus2.get_velocity()
        # a0_trq_input = axis0.get_torque_input()
        
        #stop the program if the torque or vel gets super high
        # if abs(a0_trq_input) > 12:
        #     print("torque too high: ",axis0.get_torque_input(),"Nm")
        #     break
        # if abs(a0_vel_cts*c2m) > 4:
        #     print("velocity too high: ",axis0.get_vel(),"turns/s")
        #     break
        
        
        # EGO MOTION
        pos_a0_cur = a0_pos_turns * t2m - pos_init_a0
        pos_a1_cur = a1_pos_turns * t2m - pos_init_a1
        
        global pos_a0_prev, pos_a1_prev, x_ego, y, theta
        
        pos_a0_delta = pos_a0_cur-pos_a0_prev
        pos_a1_delta = pos_a1_cur-pos_a1_prev
        # END EGO
        
        Xf = np.array([0, 0, 0, 0, 0, 0])

        x = (a0_pos_turns * t2m  + a1_pos_turns * t2m)/2 - pos_init
        v = (a0_vel_turns * t2m + a1_vel_turns * t2m)/2


        yaw_angle2 =0
        pitch_angle2 = 0
        pitch_rate2 = 0
        yaw_rate2 =0
        
        pitch_rate1, yaw_rate1 = IMU1.getRates()
        pitch_angle1, yaw_angle1 = IMU1.getAngles()
        
        
        theta_fused = (theta + yaw_angle2)/2
        
        x_ego, y, theta = update_position(x_ego, y, theta, pos_a0_delta, pos_a1_delta, W)
        
        xs_ego.append(x_ego)
        ys_ego.append(y)
        thetas_ego.append(theta)
        thetas_fused.append(theta_fused)
        
        X = np.array([x, v, -pitch_angle1, pitch_rate1, -yaw_angle1-start_angle, -yaw_rate1])
        D = np.array([[0.5, 0.5],[0.5, -0.5]])

        Cl, Cr = D @ (-K @ (X - Xf))

            
        # Apply the filtered torque values to the axes
        moteus1.set_torque(Cl)
        moteus2.set_torque(Cr)
            

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

    moteus1.stop()
    moteus2.stop()

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
    axs[3].plot(times, pitchAngles1, label='Pitch Angles 85')
    axs[3].plot(times, pitchAngles2, label='Pitch Angles 55')
    axs[3].legend()
    axs[3].set_title("Pitch Angles")

    # Plot 6
    axs[4].plot(times, pitchRates1, label='pitch rates 85')
    axs[4].plot(times, pitchRates2, label='pitch rates 55')
    axs[4].legend()
    axs[4].set_title("pitch rates")

    # Plot 7
    axs[5].plot(times, yawAngles1, label='Yaw Angles 85')
    axs[5].plot(times, yawAngles2, label='Yaw Angles 55')
    axs[5].plot(times, thetas_ego, label='Yaw Angle ego')
    axs[5].plot(times, thetas_fused, label='Yaw Angle Fused 55 Ego')
    axs[5].legend()
    axs[5].set_title("Yaw Angles")

    # Plot 7
    axs[6].plot(times, yawRates1, label='Yaw Rates 85')
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
    
    fig, axs = plt.subplots(nrows=1, ncols=1, figsize=(8, 16))

    # Plot 1
    axs.plot(ys_ego, xs_ego, label='ego')
    axs.set_title("dts")
    plt.tight_layout()
    plt.savefig(plot_dir+ "ego.png")
    plt.clf()



def brake_both_motors(axis0, axis1):

    while abs(axis0.get_vel()) > 0.05 and abs(axis1.get_vel()) > 0.05:
        axis0.set_trq(-0.5 if axis0.get_vel() > 0 else 0.5)
        axis1.set_trq(-0.5 if axis1.get_vel() > 0 else 0.5)
        
    axis0.set_trq(0)
    axis1.set_trq(0)

        


LQR(a0, a1)

brake_both_motors(a0, a1)
