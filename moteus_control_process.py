import time
import matplotlib.pyplot as plt
import numpy as np
import math
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

async def control_main(termination_event, imu_and_odometry_dict):
    fdcanusb = moteus.Fdcanusb(debug_log='fdcanusb_debug.log')
    moteus1 = moteus.Controller(id=1, transport=fdcanusb)
    moteus2 = moteus.Controller(id=2, transport=fdcanusb)

    m1state = await moteus1.set_stop(query=True)
    m2state = await moteus2.set_stop(query=True)

    t2m = 0.528 #turns to meters
    W = 0.567   # Distance between wheels in meters
    K = np.array([[-3.87, -6.55, -43.01, -20.61, -0.00, -0.00],[0.00, 0.00, 0.00, 0.00, 0.32, 0.76]] )
    Xf = np.array([0, 0, 0, 0, 0, 0])
    max_torque_delta = 0.2
    
    # SET THESE
    moteus1_direction = -1 
    moteus2_direction = -1

    moteus1_initial_position = m1state.values[moteus.Register.POSITION] * t2m * moteus1_direction
    moteus2_initial_position = m2state.values[moteus.Register.POSITION] * t2m * moteus2_direction
    combined_initial_position = (moteus1_initial_position + moteus2_initial_position)/2
    
    moteus1_previous_position = 0
    moteus2_previous_position = 0    
    moteus1_previous_torque_command = 0
    moteus2_previous_torque_command = 0
    x_ego, y_ego, theta_ego = 0, 0, 0
    
    
    pitch_angle85    = imu_and_odometry_dict.get("pitch_angle85", 0)
    yaw_angle85      = imu_and_odometry_dict.get("yaw_angle85", 0) 
    pitch_rate85_ewa = imu_and_odometry_dict.get("pitch_rate85_ewa", 0)
    yaw_rate85_ewa   = imu_and_odometry_dict.get("yaw_rate85_ewa", 0)

    start_time = time.time()
    cur_time = 0
    prev_time = 0

    m1state = await moteus1.set_stop(query=True)
    m2state = await moteus2.set_stop(query=True)

    while cur_time < 2 and not termination_event.is_set():        
        cur_time = time.time() - start_time
        dt = cur_time - prev_time
        
        moteus1_current_position = m1state.values[moteus.Register.POSITION] * t2m * moteus1_direction - moteus1_initial_position
        moteus2_current_position = m2state.values[moteus.Register.POSITION] * t2m * moteus2_direction - moteus2_initial_position
        
        moteus1_current_velocity = m1state.values[moteus.Register.VELOCITY] * t2m * moteus1_direction
        moteus2_current_velocity = m2state.values[moteus.Register.VELOCITY] * t2m * moteus2_direction
        
        moteus1_delta_position = moteus1_current_position - moteus1_previous_position
        moteus2_delta_position = moteus2_current_position - moteus2_previous_position
        
        combined_current_position = (moteus1_current_position + moteus2_current_position)/2
        combined_current_velocity = (moteus1_current_velocity + moteus2_current_velocity)/2
        
        pitch_angle85    = imu_and_odometry_dict.get("pitch_angle85", 0)
        yaw_angle85      = imu_and_odometry_dict.get("yaw_angle85", 0) 
        pitch_rate85_ewa = imu_and_odometry_dict.get("pitch_rate85_ewa", 0)
        yaw_rate85_ewa   = imu_and_odometry_dict.get("yaw_rate85_ewa", 0)
        
        x_ego, y_ego, theta_ego = update_position(x_ego, y_ego, theta_ego, moteus1_delta_position, moteus2_delta_position, W)
        
        X = np.array([
            combined_current_position, 
            combined_current_velocity, 
            -pitch_angle85, 
            pitch_rate85_ewa, 
            -yaw_angle85, 
            -yaw_rate85_ewa 
            ])
        D = np.array([[0.5, 0.5],[0.5, -0.5]])
        Cl, Cr = D @ (-K @ (X - Xf))
        
        #TORQUE RAMPING
        Cl = np.clip(Cl, moteus1_previous_torque_command - max_torque_delta, moteus1_previous_torque_command + max_torque_delta)
        Cr = np.clip(Cr, moteus2_previous_torque_command - max_torque_delta, moteus2_previous_torque_command + max_torque_delta)
        
        m1state = await moteus1.set_position(position=math.nan, velocity=0.0, accel_limit=0.0, feedforward_torque=Cl, kp_scale=0, kd_scale=0, maximum_torque=4, query=True)
        m2state = await moteus2.set_position(position=math.nan, velocity=0.0, accel_limit=0.0, feedforward_torque=Cr, kp_scale=0, kd_scale=0, maximum_torque=4, query=True)
        
        imu_and_odometry_dict['x_ego']                    = x_ego
        imu_and_odometry_dict['y_ego']                    = y_ego
        imu_and_odometry_dict['theta_ego']                = theta_ego
        imu_and_odometry_dict['moteus1_current_position'] = moteus1_current_position
        imu_and_odometry_dict['moteus2_current_position'] = moteus2_current_position
        imu_and_odometry_dict['moteus1_current_velocity'] = moteus1_current_velocity
        imu_and_odometry_dict['moteus2_current_velocity'] = moteus2_current_velocity
        imu_and_odometry_dict['moteus1_torque_reading']   = m1state.values[moteus.Register.TORQUE]
        imu_and_odometry_dict['moteus2_torque_reading']   = m2state.values[moteus.Register.TORQUE]
        imu_and_odometry_dict['moteus1_torque_command']   = Cl
        imu_and_odometry_dict['moteus2_torque_command']   = Cr
        imu_and_odometry_dict['dt_moteus_control']        = dt
    
        moteus1_previous_position = moteus1_current_position
        moteus2_previous_position = moteus2_current_position
        moteus1_previous_torque_command = Cl
        moteus2_previous_torque_command = Cr
        prev_time = cur_time
        
        await asyncio.sleep(0)

    m1state = await moteus1.set_stop(query=True)
    m2state = await moteus2.set_stop(query=True)

    # fig, axs = plt.subplots(nrows=8, ncols=1, figsize=(100, 50))

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
    # # axs[2].plot(times, ewa2, label='ewa')
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
    # # axs[4].plot(times, ewa1, label='pitewa')
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

    # axs[7].plot(times, Cl_commands, label='torque left')
    # axs[7].plot(times, Cr_commands, label='torque right')
    # axs[7].plot(times, Cl_actual, label='moteus torque left ')
    # axs[7].plot(times, Cr_actual, label='moteus torque right')
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



def run_moteus(termination_event, imu_setup, imu85_dict):
    while not imu_setup.is_set():
        continue
    
    asyncio.run(control_main(termination_event, imu85_dict))
    termination_event.set()



# if __name__ == '__main__':
#     asyncio.run(main())