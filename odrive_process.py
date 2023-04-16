import odrive
from odrive.enums import *
from odriveDriver import Axis

import time
import numpy as np
import math

def run_odrive(mode, imu_setup_done, odrive_setup_done, imu85_dict, imu55_dict, ego_estimation, drive_stats, set_points, termination_event):
    
    # WAIT FOR PROCESSES
    imu_setup_done.wait() 
    odrive_setup_done.wait()
    
    # INIT ODRIVE AGAIN
    odrv0 = odrive.find_any()
    axis0 = Axis(odrv0.axis0, dir=1)
    axis1 = Axis(odrv0.axis1, dir=-1)
    
    # INIT VARS
    
    t2m = 0.528 # Meters per turn
    W = 0.567   # Distance between wheels in meters
    
    pos_init_a0 = axis0.get_pos_turns() * t2m
    pos_a0_cur = axis0.get_pos_turns() * t2m - pos_init_a0
    pos_a0_prev = pos_a0_cur
    
    pos_init_a1 = axis1.get_pos_turns() * t2m
    pos_a1_cur = axis1.get_pos_turns() * t2m - pos_init_a1
    pos_a1_prev = pos_a1_cur
    
    x, y, theta = 0, 0, 0
    pos_init = (axis0.get_pos_turns() * t2m + axis1.get_pos_turns() * t2m)/2
    x_lqr = (axis0.get_pos_turns() * t2m  + axis1.get_pos_turns() * t2m)/2 - pos_init
    v_lqr = (axis0.get_vel() * t2m + axis1.get_vel() * t2m)/2
    
    pitch_angle1 = imu85_dict.get("pitch_angle", 0)
    yaw_angle1 = imu85_dict.get("yaw_angle", 0)
    pitch_rate1 = imu85_dict.get("pitch_rate", 0)
    yaw_rate1 = imu85_dict.get("yaw_rate", 0)
    
    pitch_angle2 = imu55_dict.get("pitch_angle", 0)
    yaw_angle2 = imu55_dict.get("yaw_angle", 0)
    pitch_rate2 = imu55_dict.get("pitch_rate", 0)
    yaw_rate2 = imu55_dict.get("yaw_rate", 0)
    
    # LQR stuff
    Xf = np.array([0, 0, 0, 0, 0, 0])
    #Q = diag([100 1 10 1 10 1]); % 'x', 'v', 'θ', 'ω', 'δ', "δ'
    #R = diag([3 3]); % Torque cost Cθ,Cδ
    K = np.array([[-4.08, -8.11, -52.57, -25.27, -0.00, 0.00],[0.00, 0.00, 0.00, 0.00, 1.83, 1.77]] )
    
    start_time = time.time()
    cur_time = time.time() - start_time
    prev_time = time.time() - start_time
    
    #keyboard control
    x_stable = x_lqr
    theta_stable = yaw_angle1
    prev_dir = 0
    
    def update_position(x, y, theta, d_left, d_right, W):
        distance_left = d_left
        distance_right = d_right

        distance_avg = (distance_left + distance_right) / 2
        delta_theta = (distance_right - distance_left) / W

        x += distance_avg * math.cos(theta + delta_theta / 2)
        y += distance_avg * math.sin(theta + delta_theta / 2)
        theta += delta_theta

        return x, y, theta

    
    def brake_both_motors():
        while abs(axis0.get_vel()) > 0.05 and abs(axis1.get_vel()) > 0.05:
            axis0.set_trq(-0.5 if axis0.get_vel() > 0 else 0.5)
            axis1.set_trq(-0.5 if axis1.get_vel() > 0 else 0.5)
            
        axis0.set_trq(0)
        axis1.set_trq(0)
        
    def LQR():
        nonlocal K
        nonlocal Xf
        
        
        x_lqr = (axis0.get_pos_turns() * t2m  + axis1.get_pos_turns() * t2m)/2 - pos_init
        
        v_lqr = (axis0.get_vel() * t2m + axis1.get_vel() * t2m)/2
        
        
        pitch_angle1 = imu85_dict.get("pitch_angle", 0)
        yaw_angle1 = imu85_dict.get("yaw_angle", 0)
        pitch_rate1 = imu85_dict.get("pitch_rate", 0)
        yaw_rate1 = imu85_dict.get("yaw_rate", 0)
        
        X = np.array([x_lqr, v_lqr, pitch_angle1, pitch_rate1, yaw_angle1, yaw_rate1])
        D = np.array([[0.5, 0.5],[0.5, -0.5]])
        Cl, Cr = D @ (-K @ (X - Xf))
        
        drive_stats["stats"] = (x_lqr, v_lqr, Cl, Cr)
        
        
        #stop the program if the torque or vel gets super high
        if abs(axis0.get_torque_input()) > 10:
            print("torque too high: ",axis0.get_torque_input(),"Nm")
            termination_event.set()
            
        if abs(v_lqr) > 3:
            print("velocity too high: ", v_lqr ,"m/s")
            termination_event.set()
            
        print('7')
        

        # axis0.set_trq(Cl)
        # axis1.set_trq(Cr)
            
        
            
    def LQR_keyboard():
        nonlocal x_stable
        nonlocal theta_stable
        nonlocal prev_dir
        nonlocal Xf
        
        x_lqr = (axis0.get_pos_turns() * t2m  + axis1.get_pos_turns() * t2m)/2 - pos_init
        v_lqr = (axis0.get_vel() * t2m + axis1.get_vel() * t2m)/2
        yaw_angle1 = imu85_dict.get("yaw_angle", 0)
        
        #stop the program if the torque or vel gets super high
        if abs(axis0.get_torque_input()) > 10:
            print("torque too high: ",axis0.get_torque_input(),"Nm")
            termination_event.set()
        if abs(v_lqr) > 5:
            print("velocity too high: ", v_lqr ,"m/s")
            termination_event.set()
            
        
        if mode.get("key", "NOPE") == "NONE":
            print("back to 0")
            #0.3 is to allow it to slow down
            Xf = np.array([x_stable+0.3*prev_dir, 0, 0, 0, theta_stable, 0])
            Xf = np.array([0, 0, 0, 0, 0, 0])
            K = np.array([[-4.08, -8.11, -52.57, -25.27, -0.00, 0.00],[0.00, 0.00, 0.00, 0.00, 1.83, 1.77]] )
            
        elif mode.get("key", "NOPE") == "W":
            prev_dir = 1
            x_stable = x_lqr
            theta_stable = yaw_angle1
            Xf = np.array([x_lqr, 0.25, 0.015, 0, yaw_angle1, 0])
            K = np.array([[-0.07, -5.56, -48.67, -23.37, -0.00, -0.00],[0.00, 0.00, 0.00, 0.00, 2.24, 1.98]])
            
        elif mode.get("key", "NOPE") == "S":
            prev_dir = -1
            x_stable = x_lqr
            theta_stable = yaw_angle1
            Xf = np.array([x_lqr, -0.25, -0.015, 0, yaw_angle1, 0])
            K = np.array([[-0.07, -5.56, -48.67, -23.37, -0.00, -0.00],[0.00, 0.00, 0.00, 0.00, 2.24, 1.98]])
            
        elif mode.get("key", "NOPE") == "A":
            prev_dir = 0
            x_stable = x_lqr
            theta_stable = yaw_angle1
            Xf = np.array([x_lqr, 0, 0, 0, yaw_angle1, -0.75])
            K = np.array([[-5.00, -8.21, -50.29, -24.16, 0.00, -0.00],[-0.00, -0.00, -0.00, -0.00, 0.05, 3.88]])
            
        elif mode.get("key", "NOPE") == "D":
            prev_dir = 0
            x_stable = x_lqr
            theta_stable = yaw_angle1
            Xf = np.array([x_lqr, 0, 0, 0, yaw_angle1, 0.75])
            K = np.array([[-5.00, -8.21, -50.29, -24.16, 0.00, -0.00],[-0.00, -0.00, -0.00, -0.00, 0.05, 3.88]])
            

        pitch_angle1 = imu85_dict.get("pitch_angle1", 0)
        yaw_angle1 = imu85_dict.get("yaw_angle1", 0)
        pitch_rate1 = imu85_dict.get("pitch_rate1", 0)
        yaw_rate1 = imu85_dict.get("yaw_rate1", 0)

        
        
        X = np.array([x_lqr, v_lqr, pitch_angle1, pitch_rate1, yaw_angle1, yaw_rate1])
        D = np.array([[0.5, 0.5],[0.5, -0.5]])
        Cl, Cr = D @ (-K @ (X - Xf))
        

        # axis0.set_trq(Cl)
        # axis1.set_trq(Cr)
        
        
        drive_stats["stats"] = (x_lqr, v_lqr, Cl, Cr)
        
        
        
    while not termination_event.is_set():
        
        # RUN EGO ESTIMATION
        time.sleep(0.0001)
        pos_a0_cur = axis0.get_pos_turns() * t2m - pos_init_a0
        pos_a1_cur = axis1.get_pos_turns() * t2m - pos_init_a1
        pos_a0_delta = pos_a0_cur-pos_a0_prev
        pos_a1_delta = pos_a1_cur-pos_a1_prev
        x, y, theta = update_position(x, y, theta, pos_a0_delta, pos_a1_delta, W)
        
        set_points['setpoints'] = Xf
        ego_estimation["ego"] = (x, y, theta, pos_a0_cur, pos_a1_cur)
        
        
        
        if mode.get("mode", "IDLE") == "BALANCE":
            LQR()
            
        elif mode.get("mode", "IDLE") == "KEYBOARD":
            LQR_keyboard()
            
        else:
            brake_both_motors()
            
        # Goes at the end of the loop
        pos_a0_prev = pos_a0_cur
        pos_a1_prev = pos_a1_cur
        
    print("DONE odrive")    
        
    axis0.set_trq(0)
    axis1.set_trq(0)
        
        
    
    
    
    
    