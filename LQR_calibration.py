import odrive
from odrive.enums import *
from odriveDriver import Axis

import os
import time
import numpy as np
import math
import curses
import control
import matplotlib.pyplot as plt
from multiprocessing import Process, Manager, Event

import IMU


def LQR(K, devices, pos_init, angle_init, start_time):
    IMU1, IMU2, axis0, axis1 = devices
    
    t2m = 0.528
    c2m = t2m/8192
   # time.sleep(0.0001)
        
    a0_pos_cts = axis0.get_pos_cts()
    a1_pos_cts = axis1.get_pos_cts()
    a0_vel_cts = axis0.get_vel_cts()
    a1_vel_cts = axis1.get_vel_cts()
    a0_trq_input = axis0.get_torque_input()
    
    #stop the program if the torque or vel gets super high
    if abs(a0_trq_input) > 12:
        print("torque too high: ",axis0.get_torque_input(),"Nm")
        stop_event.set()
    
    if abs(a0_vel_cts*c2m) > 4:
        print("velocity too high: ",axis0.get_vel(),"turns/s")
        stop_event.set()
    
    
    Xf = np.array([0, 0, 0, 0, 0, 0])

    x = (a0_pos_cts * c2m  + a1_pos_cts * c2m)/2 - pos_init
    v = (a0_vel_cts * c2m + a1_vel_cts * c2m)/2


    yaw_angle2 =0
    pitch_angle2 = 0
    pitch_rate2 = 0
    yaw_rate2 =0
    
    pitch_rate1, yaw_rate1 = IMU1.getRates()
    pitch_angle1, yaw_angle1 = IMU1.getAngles()
    
    
    X = np.array([x, v, -pitch_angle1, pitch_rate1, -yaw_angle1-angle_init, -yaw_rate1])

    # U = -K @ (X - Xf)

    D = np.array([[0.5, 0.5],[0.5, -0.5]])
    # print("K mat: ", K)
    # print("X mat: ", X)
    # print("Xf mat: ", Xf)
    # print("D mat:", D)
    Cl, Cr = D @ (-K @ (X - Xf))
    # print("Cl: ", Cl)
    # print("Cr: ", Cr)
    


    vel_scope = 5 #counts/s
    mult_a0 = max((vel_scope - abs(a0_vel_cts))/vel_scope, 1) if a0_vel_cts<vel_scope else 0
    mult_a1 = max((vel_scope - abs(a1_vel_cts))/vel_scope, 1) if a1_vel_cts<vel_scope else 0
    
    Cl_modded = Cl
    Cr_modded = Cr

    if Cl > 0:
        Cl_modded = Cl + 0.15*mult_a0
        
    else:
        Cl_modded = Cl - 0.15*mult_a0
        
    if Cr > 0:
        Cr_modded = Cr + 0.15*mult_a1
    else:
        Cr_modded = Cr - 0.15*mult_a1
        
    # Apply the filtered torque values to the axes
    axis0.set_trq(Cl_modded)
    axis1.set_trq(Cr_modded)

    return X#, [t_imu, t_odrive_read, t_odrive_write, t_imureaded1, t_imureaded2, t_total]


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
        print(vals)

    elif key == curses.KEY_BACKSPACE or key == 127:  # Backspace key
        if cur_idx is not None:
            vals[cur_idx] = vals[cur_idx][:-1]

    elif cur_idx is not None and chr(key).isnumeric():
        vals[cur_idx] += chr(key)

    return stop_flag, params_updated, cur_idx


def get_k(stdscr, K_dict, stop_event):
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
    K_dict['K'] = K

    labels = ["Position (x)", "Velocity (v)", "Pitch Angle (p)", "Pitch Rate (w)", "Yaw Angle (a)", "Yaw Rate (y)", "R Gain (r)"]

    cur_idx = None
    lqr_params = [*Q.diagonal(), R[0,0]]
    vals = [str(x) for x in lqr_params] * 7
    
    t0 = time.time()
    while time.time() < t0 + 30:
        # stop_flag, params_updated, cur_idx = update_curses(stdscr, cur_idx, vals, labels, lqr_params)
        # if stop_flag:
        #     stop_event.set()
        #     break
        params_updated = False
    
        if params_updated:
            Q = np.diag(lqr_params[:6])
            R = np.diag([lqr_params[6]]*2)
            K = calculate_gains(A, B, Q, R)
            K_dict['K'] = K
            print(f"Updated K: {K}")
            print(f"Params: {lqr_params}")
            print("*"*50)
    
    stop_event.set()
    np.savetxt('lqr_params/K.txt', K)
    np.savetxt('lqr_params/Q.txt', Q, fmt='%i')
    np.savetxt('lqr_params/R.txt', R, fmt='%i')

def curses_wrapper(K_dict, stop_event):
    curses.wrapper(lambda stdscr: get_k(stdscr, K_dict, stop_event))

def balance(K_dict, stop_event):
    import sys
    # sys.stdout = open(os.devnull, 'w')
    # SETUP IMU
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

    devices = [IMU1, IMU2, axis0, axis1]

    t2m = 0.528
    c2m = t2m/8192
    
    pos_init = (axis0.get_pos_cts() * c2m + axis1.get_pos_cts() * c2m)/2
    
    pitch_angle1=IMU1.getPitchAngle() 
    yaw_angle1=IMU1.getYawAngle()
    pitch_rate1=IMU1.getPitchRate()
    yaw_rate1=IMU1.getYawRate()
    
    start_angle = -yaw_angle1
    
    start_time = time.time()
    
    
    # K_dict['state'] = []
    # K_dict['times'] = []
    while time.time()-start_time < 5:
        state = LQR(K_dict['K'], devices, pos_init, start_angle, start_time)
        # K_dict['state'] = K_dict['state'] + [state]
        # K_dict['times'] = K_dict['times'] + [times]
    
    axis0.set_trq(0)
    axis1.set_trq(0)


if __name__ == "__main__":
    with Manager() as manager:
        K_dict = manager.dict()

        K_dict['K'] = np.array([[-7.56, -12.01, -67.08, -32.35, 0.00, 0.00],[-0.00, -0.00, -0.00, -0.00, 1.69, 1.69]])

        stop_event = Event()
        # curses_thread = Process(target=get_k, args=(None, K_dict, stop_event))
        balance_thread = Process(target=balance, args=(K_dict, stop_event))
        
        # curses_thread.start()
        balance_thread.start()

        # curses_thread.join()
        balance_thread.join()
    

        print(K_dict['K'])
        # dts = np.stack(K_dict['times'])
        # states = np.stack(K_dict['state'])
        
    # fig, axs = plt.subplots(nrows=7, ncols=1, figsize=(8, 100))

    # axs[0].set_title('dt')
    # axs[0].plot(dts[:,0], label='imu time')
    # axs[0].plot(dts[:,1], label='odrive read time')
    # axs[0].plot(dts[:,2], label='odrive write time')
    # axs[0].plot(dts[:,3], label='imuread1')
    # axs[0].plot(dts[:,4], label='imuread2')
    # axs[0].plot(dts[:,5], label='total loop time')
    # axs[0].xaxis.grid(True, linestyle='--', alpha=0.5)  # add x-axis grid
    # axs[0].legend()
    
    # for i, (var, name) in enumerate(zip(states.T, ['x', 'v', 'p', 'w', 'a', 'y'])):
    #     axs[i+1].set_title(name)
    #     axs[i+1].plot(var)
    #     axs[i+1].xaxis.grid(True, linestyle='--', alpha=0.5)  # add x-axis grid

    # plt.savefig(f'lqr_params/plots.png')




