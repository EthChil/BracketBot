import time
import numpy as np
import math
import matplotlib.pyplot as plt

plt.switch_backend('Agg')
plot_dir = './MASTER_LOGS/'

def run_logging_process(termination_event, imu_setup, imu_and_odometry_dict):
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
    
    while not termination_event.is_set():
        
        cur_time = time.time()-start_time
        dt = cur_time - prev_time
        times.append(cur_time)
        
        x_ego.append(imu_and_odometry_dict.get("x_ego", 0))
        y_ego.append(imu_and_odometry_dict.get("y_ego", 0))
        theta_ego.append(imu_and_odometry_dict.get("theta_ego", 0))
        moteus1_current_position.append(imu_and_odometry_dict.get("moteus1_current_position", 0))
        moteus2_current_position.append(imu_and_odometry_dict.get("moteus2_current_position", 0))
        moteus1_current_velocity.append(imu_and_odometry_dict.get("moteus1_current_velocity", 0))
        moteus2_current_velocity.append(imu_and_odometry_dict.get("moteus2_current_velocity", 0))
        moteus1_torque_reading.append(imu_and_odometry_dict.get("moteus1_torque_reading", 0))
        moteus2_torque_reading.append(imu_and_odometry_dict.get("moteus2_torque_reading", 0))
        moteus1_torque_command.append(imu_and_odometry_dict.get("moteus1_torque_command", 0))
        moteus2_torque_command.append(imu_and_odometry_dict.get("moteus2_torque_command", 0))
        dt_moteus_control.append(imu_and_odometry_dict.get("dt_moteus_control", 0))
        pitch_angle85.append(imu_and_odometry_dict.get("pitch_angle85", 0))
        yaw_angle85.append(imu_and_odometry_dict.get("yaw_angle85", 0))
        pitch_rate85.append(imu_and_odometry_dict.get("pitch_rate85", 0))
        yaw_rate85.append(imu_and_odometry_dict.get("yaw_rate85", 0))
        pitch_rate85_ewa.append(imu_and_odometry_dict.get("pitch_rate85_ewa", 0))
        yaw_rate85_ewa.append(imu_and_odometry_dict.get("yaw_rate85_ewa", 0))
        dt_imu85_process.append(imu_and_odometry_dict.get("dt_imu85_process", 0))
        dt_logging_process.append(dt)
        
        
        prev_time = cur_time
        time.sleep(0.0001)
        
    print("Saving Logs and Plots")
    
    fig, axs = plt.subplots(nrows=8, ncols=1, figsize=(20, 10))

    # Plot 1
    axs[0].plot(times, dt_logging_process, label='logging process')
    axs[0].plot(times, dt_imu85_process, label='imu85 process')
    axs[0].plot(times, dt_moteus_control, label='moteus process')
    axs[0].legend()
    axs[0].set_title("Delta times")

    # Plot 2
    axs[1].plot(times, moteus1_current_position, label='Moteus 1 position')
    axs[1].plot(times, moteus2_current_position, label='Moteus 2 position')
    axs[1].plot(times, x_ego, label='Ego')
    axs[1].legend()
    axs[1].set_title("Position X")

    # Plot 3
    axs[2].plot(times, moteus1_current_velocity, label='Moteus 1 Velocity')
    axs[2].plot(times, moteus2_current_velocity, label='Moteus 2 Velocity')
    axs[2].legend()
    axs[2].set_title("Velocity V")

    # Plot 5
    axs[3].plot(times, pitch_angle85, label='Pitch Angles 85')
    axs[3].legend()
    axs[3].set_title("Pitch Angles")

    # Plot 6
    axs[4].plot(times, pitch_rate85, label='pitch rates 85')
    axs[4].plot(times, pitch_rate85_ewa, label='pitch rates 85 EWA')
    axs[4].legend()
    axs[4].set_title("pitch rates")

    # Plot 7
    axs[5].plot(times, yaw_angle85, label='Yaw Angles 85')
    axs[5].plot(times, theta_ego, label='Yaw Angle ego')
    axs[5].legend()
    axs[5].set_title("Yaw Angles")

    # Plot 7
    axs[6].plot(times, yaw_rate85, label='Yaw Rates 85')
    axs[6].plot(times, yaw_rate85_ewa, label='Yaw Rates 85 EWA')
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