
            
import time
import matplotlib.pyplot as plt
import numpy as np
import math
# import IMU

import asyncio
import moteus

plt.switch_backend('Agg')
plot_dir = './plots/'

def update_position(x, y, theta, d_left, d_right, W):
        distance_left = d_left
        distance_right = d_right

        distance_avg = (distance_left + distance_right) / 2
        delta_theta = (distance_right - distance_left) / W

        x += distance_avg * math.cos(theta + delta_theta / 2)
        y += distance_avg * math.sin(theta + delta_theta / 2)
        theta += delta_theta

        return x, y, theta

async def control_main(termination_event, imu85_dict):

    fdcanusb = moteus.Fdcanusb(debug_log='fdcanusb_debug.log')
    moteus1 = moteus.Controller(id=1, transport=fdcanusb)
    moteus2 = moteus.Controller(id=2, transport=fdcanusb)

    m1state = await moteus1.set_stop(query=True)
    m2state = await moteus2.set_stop(query=True)
    
    # IMU1 = IMU.IMU_BNO085()
    # IMU1.setupIMU()

    t2m = 0.528 #turns to meters
    W = 0.567   # Distance between wheels in meters
    
    K = np.array([[-3.87, -6.55, -43.01, -20.61, -0.00, -0.00],[0.00, 0.00, 0.00, 0.00, 0.32, 0.76]] )

    pos_init_a0 = -m1state.values[moteus.Register.POSITION] * t2m
    pos_init_a1 = -m2state.values[moteus.Register.POSITION] * t2m
    pos_init = (pos_init_a0 + pos_init_a1)/2
    print(pos_init)
    

    pos_a0_cur = 0
    pos_a1_cur = 0
    pos_a0_prev = 0
    pos_a1_prev = 0

    xs_ego = []
    ys_ego = []
    thetas_ego = []
    thetas_fused = []

    x_ego, y, theta = 0, 0, 0 # Initial position and heading

    
    pitch_angle1 = imu85_dict.get("pitch_angle", 0)
    yaw_angle1 =   imu85_dict.get("yaw_angle", 0) 
    pitch_rate1 =  imu85_dict.get("pitch_rate_ewa", 0)
    yaw_rate1 =    imu85_dict.get("yaw_rate_ewa", 0)

    # pitch_angle1, yaw_angle1 = IMU1.getAngles()
    # pitch_rate1, yaw_rate1 = IMU1.getRates()
    
    prev_pitch = pitch_angle1
    
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
    
    Cl_actual = []
    Cr_actual = []
    
    ewa1 = []
    ewa2 = []
    
    m1state = await moteus1.set_stop(query=True)
    m2state = await moteus2.set_stop(query=True)
    
    
    max_torque_delta = 0.2  # Change this value based on your desired torque ramp rate

    prev_Cl = 0
    prev_Cr = 0



    while cur_time < 10:        
        cur_time = time.time() - start_time # relative time starts at 0
        dt = cur_time - prev_time
        
        a0_pos_turns = -m1state.values[moteus.Register.POSITION]
        a1_pos_turns = -m2state.values[moteus.Register.POSITION]
        
        a0_vel_turns = -m1state.values[moteus.Register.VELOCITY]
        a1_vel_turns = -m2state.values[moteus.Register.VELOCITY]
        
        
        # EGO MOTION
        pos_a0_cur = a0_pos_turns * t2m - pos_init_a0
        pos_a1_cur = a1_pos_turns * t2m - pos_init_a1
        
        pos_a0_delta = pos_a0_cur-pos_a0_prev
        pos_a1_delta = pos_a1_cur-pos_a1_prev
        # END EGO
        
        Xf = np.array([0, 0, 0, 0, 0, 0])
        x = (a0_pos_turns * t2m  + a1_pos_turns * t2m)/2 - pos_init
        v = (a0_vel_turns * t2m + a1_vel_turns * t2m)/2

        yaw_angle1 =0
        pitch_angle1 = 0
        pitch_rate1 = 0
        yaw_rate1 =0
        
        yaw_angle2 =0
        pitch_angle2 = 0
        pitch_rate2 = 0
        yaw_rate2 =0
        
        pitch_angle1 = imu85_dict.get("pitch_angle", 0)
        yaw_angle1 =   imu85_dict.get("yaw_angle", 0) 
        pitch_rate1 =  imu85_dict.get("pitch_rate_ewa", 0)
        yaw_rate1 =    imu85_dict.get("yaw_rate_ewa", 0)
        
        # pitch_angle1, yaw_angle1 = IMU1.getAngles()
        # pitch_rate1_raw, yaw_rate1 = IMU1.getRates()
        
        
        # pitch_rate1 = (prev_pitch - pitch_angle1)/dt
        # v = (pos_a0_delta)/dt
        
        # pitch_rate_ewa = alpha * (-pitch_rate1) + (1 - alpha) * pitch_rate_ewa
        # vel_ewa = alpha *  v + (1 - alpha) * vel_ewa

    
        theta_fused = (theta + yaw_angle2)/2
        
        x_ego, y, theta = update_position(x_ego, y, theta, pos_a0_delta, pos_a1_delta, W)
        
        # print(x, v, -pitch_angle1, -pitch_rate_ewa)
        

        X = np.array([x, v, -pitch_angle1, pitch_rate1, -yaw_angle1, -yaw_rate1 ])
        D = np.array([[0.5, 0.5],[0.5, -0.5]])

        Cl, Cr = D @ (-K @ (X - Xf))
        
        Cl = np.clip(Cl, prev_Cl - max_torque_delta, prev_Cl + max_torque_delta)
        Cr = np.clip(Cr, prev_Cr - max_torque_delta, prev_Cr + max_torque_delta)

        # ...

        # Store previous torque values for the next iteration
        prev_Cl = Cl
        prev_Cr = Cr
        
        m1state = await moteus1.set_position(position=math.nan, velocity=0.0, accel_limit=0.0, feedforward_torque=Cl, kp_scale=0, kd_scale=0, maximum_torque=4, query=True)
        m2state = await moteus2.set_position(position=math.nan, velocity=0.0, accel_limit=0.0, feedforward_torque=Cr, kp_scale=0, kd_scale=0, maximum_torque=4, query=True)
    
        xs_ego.append(x_ego)
        ys_ego.append(y)
        thetas_ego.append(theta)
        thetas_fused.append(theta_fused)
            
        Cl_actual.append(m1state.values[moteus.Register.TORQUE])
        Cr_actual.append(m2state.values[moteus.Register.TORQUE])

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
        
        # ewa1.append(pitch_rate1_raw)
        # ewa2.append(vel_ewa)
    
        pos_a0_prev = pos_a0_cur
        pos_a1_prev = pos_a1_cur
        prev_time = cur_time
        prev_pitch = pitch_angle1
        
        await asyncio.sleep(0)

    m1state = await moteus1.set_stop(query=True)
    m2state = await moteus2.set_stop(query=True)

    fig, axs = plt.subplots(nrows=8, ncols=1, figsize=(100, 50))

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
    # axs[2].plot(times, ewa2, label='ewa')
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
    # axs[4].plot(times, ewa1, label='pitewa')
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

    axs[7].plot(times, Cl_commands, label='torque left')
    axs[7].plot(times, Cr_commands, label='torque right')
    axs[7].plot(times, Cl_actual, label='moteus torque left ')
    axs[7].plot(times, Cr_actual, label='moteus torque right')
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



def run_moteus(termination_event, imu_setup, imu85_dict):
    while not imu_setup.is_set():
        continue
    
    asyncio.run(control_main(termination_event, imu85_dict))
    termination_event.set()



# if __name__ == '__main__':
#     asyncio.run(main())