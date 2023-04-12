import odrive
from odrive.enums import *
from odriveDriver import Axis

import os
import time
import numpy as np
import math
import curses
import control

import IMU

# SETUP IMU
IMU55_dict = {}
IMU85_dict = {}
IMU1 = IMU.IMU_BNO085()
IMU1.setupIMU()
IMU2 = IMU.IMU_BNO055(0, 40)
IMU2.restoreCalibrationConstants([0, 0, 85, 0, 1, 0, 166, 1, 77, 1, 176, 1, 1, 0, 0, 0, 0, 0, 232, 3, 178, 1]) #only for bno55
IMU2.setupIMU()

# SETUP ODRIVE
odrv0 = odrive.find_any()
axis0 = Axis(odrv0.axis0, dir=1)
axis1 = Axis(odrv0.axis1, dir=-1)

axis0.setup()
axis1.setup()

t2m = 0.528 # Meters per turn
W = 0.567   # Distance between wheels in meters

pos_init = axis0.get_pos_turns()*t2m

def LQR(K):
    pitch_angle1 = -IMU1.getYawRate()
    yaw_angle2 = IMU2.getYawAngle()
    pitch_rate2 = -IMU2.getPitchRate()
    yaw_rate2 = -IMU2.getYawRate()
    
    #stop the program if the torque or vel gets super high
    if abs(axis0.get_torque_input()) > 10:
        print("torque too high: ",axis0.get_torque_input(),"Nm")
        axis0.set_trq(0)
        axis1.set_trq(0)
        exit(0)
    if abs(axis0.get_vel()) > 3:
        print("velocity too high: ", axis0.get_vel() ,"m/s")
        axis0.set_trq(0)
        axis1.set_trq(0)
        exit(0)

    Xf = np.array([0, 0, 0, 0, 0, 0])
    X = np.array([axis0.get_pos_turns()*t2m-pos_init, axis0.get_vel()*t2m, pitch_angle1, pitch_rate2, yaw_angle2, yaw_rate2])
    D = np.array([[0.5, 0.5],[0.5, -0.5]])
    Cl, Cr = D @ (-K @ (X - Xf))
    
    
    # vel_scope = 0.03
    # mult_a0 = max((vel_scope - a0vel)/vel_scope, 1) if a0vel<vel_scope else 0
    # mult_a1 = max((vel_scope - a1vel)/vel_scope, 1) if a1vel<vel_scope else 0
    
    # Cl_modded = Cl
    # Cr_modded = Cr

    # if Cl > 0:
    #     Cl_modded = Cl + 0.26*mult_a0
        
    # else:
    #     Cl_modded = Cl - 0.25*mult_a0
        
    # if Cr > 0:
    #     Cr_modded = Cr + 0.28*mult_a1
    # else:
    #     Cr_modded = Cr - 0.23*mult_a1
        
    axis0.set_trq(Cl)
    axis1.set_trq(Cr)

    # if you want custom anticogging
    # axis0.set_trq(Cl_modded)
    # axis1.set_trq(Cr_modded)


def calculate_gains(A, B, Q, R):
    K, _, _ = control.lqr(A, B, Q, R)
    return K


def update_curses(stdscr, cur_idx, vals, labels, lqr_params):
    params_updated = False
    stop_flag = False

    stdscr.clear()
    curses.curs_set(0)
    height, width = stdscr.getmaxyx()
    x = int((width - 11) / 2)
    y = int((height - 7) / 2)
    for i in range(7):
        stdscr.addstr(y+i, x, f"{labels[i]}: ", curses.A_BOLD)
        stdscr.addstr(vals[i], curses.A_UNDERLINE)

    if cur_idx is not None:
        cursor_y = y + cur_idx
        cursor_x = x + len(labels[cur_idx]) + 2 + len(vals[cur_idx])
        stdscr.move(cursor_y, cursor_x)
        curses.curs_set(1)


    stdscr.refresh()

    key = stdscr.getch()
    # exit the loop if the user presses the 'q' key
    if key == ord('q'):
        stop_flag = True

    elif key == ord('x'):
        curses.curs_set(1)
        cur_idx = 0
    elif key == ord('v'):
        curses.curs_set(1)
        cur_idx = 1
    elif key == ord('p'):
        curses.curs_set(1)
        cur_idx = 2
    elif key == ord('w'):
        curses.curs_set(1)
        cur_idx = 3
    elif key == ord('a'):
        curses.curs_set(1)
        cur_idx = 4
    elif key == ord('y'):
        curses.curs_set(1)
        cur_idx = 5
    elif key == ord('r'):
        curses.curs_set(1)
        cur_idx = 6

    elif key == 10:  # Enter key
        curses.curs_set(0)
        for i in range(7):
            lqr_params[i] = int(vals[i])
        params_updated = True
        cur_idx = None

    elif key == curses.KEY_BACKSPACE or key == 127:  # Backspace key
        if cur_idx is not None:
            vals[cur_idx] = vals[cur_idx][:-1]

    elif cur_idx is not None and chr(key).isnumeric():
        vals[cur_idx] += chr(key)



    return stop_flag, params_updated, cur_idx


def main(stdscr):
    # K = np.array([[-2.93, -6.61, -46.64, -19.96, 0.00, 0.00],[0.00, 0.00, 0.00, 0.00, 1.20, 1.45]])
    # if os.path.exists('lqr_params/K.txt'):
    #     K = np.loadtxt('lqr_params/K.txt')
    A = np.loadtxt('lqr_params/A.txt', delimiter=',')
    B = np.loadtxt('lqr_params/B.txt', delimiter=',')

    Q = np.eye(6)
    R = np.eye(2)
    if os.path.exists('lqr_params/Q.txt'):
        Q = np.loadtxt('lqr_params/Q.txt')
    if os.path.exists('lqr_params/R.txt'):
        R = np.loadtxt('lqr_params/R.txt')
    Q = Q.astype(int)
    R = R.astype(int)
    K = calculate_gains(A, B, Q, R)

    labels = ["Position (x)", "Velocity (v)", "Pitch Angle (p)", "Pitch Rate (w)", "Yaw Angle (a)", "Yaw Rate (y)", "R Gain (r)"]

    cur_idx = None
    lqr_params = [*Q.diagonal(), R[0,0]]
    vals = [str(x) for x in lqr_params] * 7
    
    t0 = time.time()
    while time.time() < t0 + 100:
        stop_flag, params_updated, cur_idx = update_curses(stdscr, cur_idx, vals, labels, lqr_params)
        if stop_flag:
            break
    
        if params_updated:
            Q = np.diag(lqr_params[:6])
            R = np.diag([lqr_params[6]]*2)
            K = calculate_gains(A, B, Q, R)
            print(f"Updated K: {K}")
            print(f"Params: {lqr_params}")
            print("*"*50)

        with open('out.txt', 'a') as f:
            f.write(f'{time.time()}\n')
        # LQR(K)
    np.savetxt('lqr_params/K.txt', K)
    np.savetxt('lqr_params/Q.txt', Q, fmt='%i')
    np.savetxt('lqr_params/R.txt', R, fmt='%i')

curses.wrapper(main)
# main(None)

axis0.set_trq(0)
axis1.set_trq(0)
