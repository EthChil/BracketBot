import time
import numpy as np
import math
import matplotlib.pyplot as plt

plt.switch_backend('Agg')
plot_dir = './MASTER_LOGS/'

def logger(mode, imu_setup_done, odrive_setup_done, imu85_dict, imu55_dict, ego_estimation, drive_stats, termination_event):
    
    # WAIT FOR PROCESSES
    imu_setup_done.wait() 
    odrive_setup_done.wait()
    
    start_time = time.time()
    cur_time = time.time() - start_time
    prev_time = time.time() - start_time
    
    # SAVE THINGS TO PLOT
    times = []
    dts = []
    
    xs_lqr = []
    vs_lqr = []
    dxdts = []
    
    xs_ego = []
    ys_ego = []
    thetas_ego = []

    pitchAngles85 = []
    yawAngles85 = []
    pitchRates85 = []
    yawRates85 = []
    
    pitchAngles55 = []
    yawAngles55 = []
    pitchRates55 = []
    yawRates55 = []

    Cl_commands = []
    Cr_commands = []
    
    while not termination_event.is_set():
        
        cur_time = time.time()-start_time
        dt = cur_time - prev_time
        
        pitch_angle85 = imu85_dict.get("pitch_angle", 0)
        yaw_angle85 = imu85_dict.get("yaw_angle", 0)
        pitch_rate85 = imu85_dict.get("pitch_rate", 0)
        yaw_rate85 = imu85_dict.get("yaw_rate", 0)
        
        pitch_angle55 = imu55_dict.get("pitch_angle", 0)
        yaw_angle55 = imu55_dict.get("yaw_angle", 0)
        pitch_rate55 = imu55_dict.get("pitch_rate", 0)
        yaw_rate55 = imu55_dict.get("yaw_rate", 0)
        
        egos = ego_estimation.get("ego", (0, 0, 0))
        drives = drive_stats.get("stats", (0, 0, 0, 0))
        
        
        # SAVE TO ARRAYS
        times.append(cur_time)
        dts.append(dt)
        
        xs_lqr.append(drives[0])
        vs_lqr.append(drives[1])
        Cl_commands.append(drives[2])
        Cr_commands.append(drives[3])
        
        xs_ego.append(egos[0])
        ys_ego.append(egos[1])
        thetas_ego.append(egos[2])
        
        pitchAngles85.append(pitch_angle85)
        yawAngles85.append(yaw_angle85)
        pitchRates85.append(pitch_rate85)
        yawRates85.append(yaw_rate85)
        
        pitchAngles55.append(pitch_angle55)
        yawAngles55.append(yaw_angle55)
        pitchRates55.append(pitch_rate55)
        yawRates55.append(yaw_rate55)
        
        prev_time = cur_time
        time.sleep(0.01)
        
    print("Saving Logs and Plots")
    
    fig, axs = plt.subplots(nrows=8, ncols=1, figsize=(8, 16))

    # Plot 1
    axs[0].plot(times, dts, label='dts')
    axs[0].legend()
    axs[0].set_title("dts")

    # Plot 2
    axs[1].plot(times, xs_lqr, label='xs')
    axs[1].legend()
    axs[1].set_title("xs")

    # Plot 3
    axs[2].plot(times, vs_lqr, label='vs')
    axs[2].legend()
    axs[2].set_title("vs")

    # Plot 5
    axs[3].plot(times, pitchAngles85, label='Pitch Angles 85')
    axs[3].plot(times, pitchAngles55, label='Pitch Angles 55')
    axs[3].legend()
    axs[3].set_title("Pitch Angles")

    # Plot 6
    axs[4].plot(times, pitchRates85, label='pitch rates 85')
    axs[4].plot(times, pitchRates55, label='pitch rates 55')
    axs[4].legend()
    axs[4].set_title("pitch rates")

    # Plot 7
    axs[5].plot(times, yawAngles85, label='Yaw Angles 85')
    axs[5].plot(times, yawAngles55, label='Yaw Angles 55')
    axs[5].plot(times, thetas_ego, label='Yaw Angle ego')
    axs[5].legend()
    axs[5].set_title("Yaw Angles")

    # Plot 7
    axs[6].plot(times, yawRates85, label='Yaw Rates 85')
    axs[6].plot(times, yawRates55, label='Yaw Rates 55')
    axs[6].legend()
    axs[6].set_title("Yaw Rates")

    axs[7].plot(times, Cl_commands, label='torque right')
    axs[7].plot(times, Cr_commands, label='torque left')
    axs[7].legend()
    axs[7].set_title("Balance Torques")


    plt.tight_layout()
    plt.savefig(plot_dir+ "Master_Program_Log.png")
    plt.clf()
    
    fig, axs = plt.subplots(nrows=1, ncols=1, figsize=(8, 16))

    # Plot 1
    axs.plot(ys_ego, xs_ego, label='ego')
    axs.set_title("dts")
    plt.tight_layout()
    plt.savefig(plot_dir+ "Ego_Estimation_Log.png")
    plt.clf()
    