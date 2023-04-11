import odrive
from odrive.enums import *
from odriveDriver import Axis

import time
import numpy as np
import math

def run_odrive(mode, imu_setup_done, odrive_setup_done, imu85_dict, imu55_dict, ego_estimation, drive_stats, termination_event):
    
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
    K = np.array([[-5.77, -7.74, -42.86, -16.89, 0.00, 0.00],[-0.00, -0.00, -0.00, -0.00, 1.83, 1.77]] )
    
    start_time = time.time()
    cur_time = time.time() - start_time
    prev_time = time.time() - start_time
    
    #keyboard control
    x_stable = x_lqr
    theta_stable = yaw_angle2
    
    
    
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
        x_lqr = (axis0.get_pos_turns() * t2m  + axis1.get_pos_turns() * t2m)/2 - pos_init
        v_lqr = (axis0.get_vel() * t2m + axis1.get_vel() * t2m)/2
        pitch_angle1 = imu85_dict.get("pitch_angle", 0)
        yaw_angle2 = imu55_dict.get("yaw_angle", 0)
        pitch_rate2 = imu55_dict.get("pitch_rate", 0)
        yaw_rate2 = imu55_dict.get("yaw_rate", 0)
        
        X = np.array([x_lqr, v_lqr, pitch_angle1, pitch_rate2, yaw_angle2, yaw_rate2])
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
        
        vel_scope = 0.1
        mult_a0 = (vel_scope - abs(axis0.get_vel()))/vel_scope if abs(axis0.get_vel())<0.1 else 0
        mult_a1 = (vel_scope - abs(axis1.get_vel()))/vel_scope if abs(axis1.get_vel())<0.1 else 0
        
        print(mult_a0, mult_a1)
        # instead of anticogging
        if Cl > 0:
            axis0.set_trq(Cl+0.26*mult_a0)
        else:
            axis0.set_trq(Cl-0.25*mult_a0)
            
        if Cr > 0:
            axis1.set_trq(Cr+0.28*mult_a1)
        else:
            axis1.set_trq(Cr-0.23*mult_a1)
            
    def LQR_keyboard(Xf, K):
        nonlocal x_stable
        nonlocal theta_stable
        
        x_lqr = (axis0.get_pos_turns() * t2m  + axis1.get_pos_turns() * t2m)/2 - pos_init
        v_lqr = (axis0.get_vel() * t2m + axis1.get_vel() * t2m)/2
        yaw_angle2 = imu55_dict.get("yaw_angle", 0)
        
        #stop the program if the torque or vel gets super high
        if abs(axis0.get_torque_input()) > 10:
            print("torque too high: ",axis0.get_torque_input(),"Nm")
            termination_event.set()
        if abs(v_lqr) > 3:
            print("velocity too high: ", v_lqr ,"m/s")
            termination_event.set()
            
        
        if mode.get("key", "NOPE") == "NONE":
            print("back to 0")
            Xf = np.array([x_stable, 0, 0, 0, theta_stable, 0])
            K = np.array([[-5.77, -8.02, -44.82, -17.54, 0.00, 0.00],[-0.00, -0.00, -0.00, -0.00, 1.83, 1.77]] )
            
        elif mode.get("key", "NOPE") == "W":
            x_stable = x_lqr
            theta_stable = yaw_angle2
            Xf = np.array([x_lqr, 0.2, 0, 0, yaw_angle2, 0])
            K = np.array([[-0.58, -1.63, -20.61, -7.77, -0.00, 0.00],[0.00, -0.00, -0.00, -0.00, 0.06, 3.18]] )
            
        elif mode.get("key", "NOPE") == "S":
            x_stable = x_lqr
            theta_stable = yaw_angle2
            Xf = np.array([x_lqr, -0.2, 0, 0, yaw_angle2, 0])
            K = np.array([[-0.58, -1.63, -20.61, -7.77, -0.00, 0.00],[0.00, -0.00, -0.00, -0.00, 0.06, 3.18]]  )
            
        elif mode.get("key", "NOPE") == "A":
            x_stable = x_lqr
            theta_stable = yaw_angle2
            Xf = np.array([x_lqr, 0, 0, 0, yaw_angle2, -0.5])
            K = np.array([[-0.71, -1.90, -21.98, -8.30, 0.00, 0.00],[0.00, 0.00, 0.00, 0.00, 0.07, 3.89]] )
            
        elif mode.get("key", "NOPE") == "D":
            x_stable = x_lqr
            theta_stable = yaw_angle2
            Xf = np.array([x_lqr, 0, 0, 0, yaw_angle2, 0.5])
            K = np.array([[-0.71, -1.90, -21.98, -8.30, 0.00, 0.00],[0.00, 0.00, 0.00, 0.00, 0.07, 3.89]] )
            

        pitch_angle1 = imu85_dict.get("pitch_angle", 0)
        pitch_rate2 = imu55_dict.get("pitch_rate", 0)
        yaw_rate2 = imu55_dict.get("yaw_rate", 0)
        
        X = np.array([x_lqr, v_lqr, pitch_angle1, pitch_rate2, yaw_angle2, yaw_rate2])
        D = np.array([[0.5, 0.5],[0.5, -0.5]])
        Cl, Cr = D @ (-K @ (X - Xf))
        
        drive_stats["stats"] = (x_lqr, v_lqr, Cl, Cr)
        
        a0vel = abs(axis0.get_vel())
        a1vel = abs(axis1.get_vel())
        
        vel_scope = 0.05
        
        mult_a0 = max((vel_scope - a0vel)/vel_scope, 1) if a0vel<vel_scope else 0
        mult_a1 = max((vel_scope - a1vel)/vel_scope, 1) if a1vel<vel_scope else 0

        # print(mult_a0, mult_a1, a0vel, a1vel)
        # instead of anticogging
        if Cl > 0:
            axis0.set_trq(Cl+0.26*mult_a0)
        else:
            axis0.set_trq(Cl-0.25*mult_a0)
            
        if Cr > 0:
            axis1.set_trq(Cr+0.28*mult_a1)
        else:
            axis1.set_trq(Cr-0.23*mult_a1)
        
        
        
    while not termination_event.is_set():
        # RUN EGO ESTIMATION
        time.sleep(0.0001)
        pos_a0_cur = axis0.get_pos_turns() * t2m - pos_init_a0
        pos_a1_cur = axis1.get_pos_turns() * t2m - pos_init_a1
        pos_a0_delta = pos_a0_cur-pos_a0_prev
        pos_a1_delta = pos_a1_cur-pos_a1_prev
        x, y, theta = update_position(x, y, theta, pos_a0_delta, pos_a1_delta, W)
        
        ego_estimation["ego"] = (x, y, theta)
        
        # print(mode.get("key", "NOPE"))
        
        if mode.get("mode", "IDLE") == "BALANCE":
            LQR()
            
        if mode.get("mode", "IDLE") == "KEYBOARD":
            LQR_keyboard(Xf, K)
            
        else:
            brake_both_motors()
            
        
        # Goes at the end of the loop
        pos_a0_prev = pos_a0_cur
        pos_a1_prev = pos_a1_cur
        
    axis0.set_trq(0)
    axis1.set_trq(0)
        
        
    
    
    
    
    