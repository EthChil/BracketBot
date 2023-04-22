import time
import numpy as np
import math
import matplotlib.pyplot as plt

plt.switch_backend('Agg')
plot_dir = './MASTER_LOGS/'



def run_logging_process(termination_event, imu_setup, imu_shared_array, odometry_shared_array, LQR_state_array):
    imu_setup.wait() 
    
    start_time = time.time()
    cur_time = 0
    prev_time = 0
    
    # SAVE THINGS TO PLOT
    times = []
    
    x_ego = []
    y_ego = []
    theta_ego = []
    moteus1_current_position = []
    moteus2_current_position = []
    moteus1_current_velocity = []
    moteus2_current_velocity = []
    moteus1_torque_reading = []
    moteus2_torque_reading = []
    moteus1_torque_command = []
    moteus2_torque_command = []
    dt_moteus_control = []
    pitch_angle85 = []
    yaw_angle85 = []
    pitch_rate85 = []
    yaw_rate85 = []
    pitch_rate85_ewa = []
    yaw_rate85_ewa = []
    dt_imu85_process = []
    dt_logging_process = []
    
    z_position = []
    z_velocity = []
    z_pitch_angle = []
    z_pitch_rate = []
    z_yaw_angle = []
    z_yaw_rate = []
    
    x_position = []
    x_velocity = []
    x_pitch_angle = []
    x_pitch_rate = []
    x_yaw_angle = []
    x_yaw_rate = []
    
    

    
    while not termination_event.is_set():
        
        cur_time = time.time()-start_time
        dt = cur_time - prev_time
        prev_time = cur_time
        
        times.append(cur_time)
        
        with imu_shared_array.get_lock():
            imu_data = imu_shared_array[:]
            
        pitch_angle85.append(imu_data[0])
        yaw_angle85.append(imu_data[1])
        pitch_rate85.append(imu_data[2])
        yaw_rate85.append(imu_data[3])
        dt_imu85_process.append(imu_data[4])
        dt_logging_process.append(dt)
            
        with odometry_shared_array.get_lock():
            odometry_data = odometry_shared_array[:]
            
        x_ego.append(odometry_data[0])
        y_ego.append(odometry_data[1])
        theta_ego.append(odometry_data[2])
        moteus1_current_position.append(odometry_data[3])
        moteus2_current_position.append(odometry_data[4])
        moteus1_current_velocity.append(odometry_data[5])
        moteus2_current_velocity.append(odometry_data[6])
        moteus1_torque_reading.append(odometry_data[7])
        moteus2_torque_reading.append(odometry_data[8])
        moteus1_torque_command.append(odometry_data[9])
        moteus2_torque_command.append(odometry_data[10])
        dt_moteus_control.append(odometry_data[11])
            
        with LQR_state_array.get_lock():
            LQR_data = LQR_state_array[:]
        
        z_position.append(LQR_data[0])
        z_velocity.append(LQR_data[1])
        z_pitch_angle.append(LQR_data[2])
        z_pitch_rate.append(LQR_data[3])
        z_yaw_angle.append(LQR_data[4])
        z_yaw_rate.append(LQR_data[5])
        
        x_position.append(LQR_data[6])
        x_velocity.append(LQR_data[7])
        x_pitch_angle.append(LQR_data[8])
        x_pitch_rate.append(LQR_data[9])
        x_yaw_angle.append(LQR_data[10])
        x_yaw_rate.append(LQR_data[11])

        
        
        time.sleep(0.001)
        
        
    print("Saving Logs and Plots")
    
    fig, axs = plt.subplots(nrows=8, ncols=1, figsize=(40, 30))

    # Plot 1
    axs[0].plot(times, dt_logging_process, label='logging process')
    axs[0].plot(times, dt_imu85_process, label='imu85 process')
    axs[0].plot(times, dt_moteus_control, label='moteus process')
    axs[0].legend()
    axs[0].set_title("Delta times")

    # Plot 2
    axs[1].plot(times, moteus1_current_position, label='Moteus 1 position')
    axs[1].plot(times, moteus2_current_position, label='Moteus 2 position')
    axs[1].plot(times, z_position, label='z state position')
    axs[1].plot(times, x_position, label='x_hat state position')
    axs[1].plot(times, x_ego, label='Ego')
    axs[1].legend()
    axs[1].set_title("Position X")

    # Plot 3
    axs[2].plot(times, moteus1_current_velocity, label='Moteus 1 Velocity')
    axs[2].plot(times, moteus2_current_velocity, label='Moteus 2 Velocity')
    axs[2].plot(times, z_velocity, label='z state velocity')
    axs[2].plot(times, x_velocity, label='x_hat state velocity')
    axs[2].legend()
    axs[2].set_title("Velocity V")

    # Plot 5
    axs[3].plot(times, pitch_angle85, label='Pitch Angles 85')
    axs[3].plot(times, z_pitch_angle, label='z state pitch angle')
    axs[3].plot(times, x_pitch_angle, label='x_hat state pitch angle')
    axs[3].legend()
    axs[3].set_title("Pitch Angles")

    # Plot 6
    axs[4].plot(times, pitch_rate85, label='pitch rates 85')
    axs[4].plot(times, z_pitch_rate, label='z state pitch rate')
    axs[4].plot(times, x_pitch_rate, label='x_hat state pitch rate')
    axs[4].legend()
    axs[4].set_title("pitch rates")

    # Plot 7
    axs[5].plot(times, yaw_angle85, label='Yaw Angles 85')
    axs[5].plot(times, z_yaw_angle, label='z state yaw angle')
    axs[5].plot(times, x_yaw_angle, label='x_hat yaw angle')
    axs[5].plot(times, theta_ego, label='Yaw Angle ego')
    axs[5].legend()
    axs[5].set_title("Yaw Angles")

    # Plot 7
    axs[6].plot(times, yaw_rate85, label='Yaw Rates 85')
    axs[6].plot(times, z_yaw_rate, label='z state yaw rate')
    axs[6].plot(times, x_yaw_rate, label='x_hat state yaw rate')
    axs[6].legend()
    axs[6].set_title("Yaw Rates")

    axs[7].plot(times, moteus1_torque_command, label='Moteus 1 torque command')
    axs[7].plot(times, moteus2_torque_command, label='Moteus 2 torque command')
    axs[7].plot(times, moteus1_torque_reading, label='Moteus 1 torque reading')
    axs[7].plot(times, moteus2_torque_reading, label='Moteus 2 torque reading')
    axs[7].legend()
    axs[7].set_title("Torques")
    
    plt.tight_layout()
    plt.savefig(plot_dir+ "IMU_and_Odometry.png")
    plt.clf()
    
    fig, axs = plt.subplots(nrows=1, ncols=1, figsize=(16, 16))

    # Plot 1
    axs.plot(y_ego, x_ego, label='ego')
    axs.set_title("EGO")
    plt.tight_layout()
    plt.savefig(plot_dir+ "Ego_Estimation_Log.png")
    plt.clf()