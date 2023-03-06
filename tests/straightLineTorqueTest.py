import sys
sys.path.append('../')
import odrive
from odrive.enums import *
import time
import matplotlib.pyplot as plt
import numpy as np
import IMU
import odriveDriver


plt.switch_backend('Agg')

plot_dir = './plots/'

t2m = 0.528 #turns to meters

odrv0 = odrive.find_any()

def get_slope(x, y1, y2):
    # Fit a straight line to the points using NumPy
    fit1 = np.polyfit(x, y1, 1)
    fit2 = np.polyfit(x, y2, 1)
    # The slope of the line is the acceleration

    avg_slope = (fit1[0] + fit2[0])/2
    return avg_slope, fit1[0], fit2[0]

def get_torque(accel):
    return 0.001 * accel / 0.0846 + 9.31 * accel * 0.0846 / 2

def straight_accel_test(axis0, axis1, torque_set=0):
    axis0.setup()
    axis1.setup()

    axis0.trq_set(torque_set)
    axis1.trq_set(torque_set)

    #turns -> meters constant
    t2m = 0.528

    #get starting position
    start_pos_axis0 = axis0.get_pos_turns() * t2m
    start_pos_axis1 = axis1.get_pos_turns() * t2m

    cur_pos_axis0 = axis0.get_pos_turns() * t2m - start_pos_axis0
    cur_pos_axis1 = axis1.get_pos_turns() * t2m - start_pos_axis1


    # velocities
    cur_vel_axis0 = axis0.get_vel()
    cur_vel_axis1 = axis1.get_vel()

    start_time = time.time() - ref_time
    cur_time = start_time

    times = []
    pos_axis0 = []
    pos_axis1 = []
    vels_axis0 = []
    vels_axis1 = []

    while cur_pos_axis0 < 2:
        print(cur_pos_axis0, cur_pos_axis1)
        cur_time = time.time() - ref_time
        cur_vel_axis0 = axis0.get_vel()
        cur_vel_axis1 = axis1.get_vel()
        cur_pos_axis0 = axis0.get_pos_turns() * t2m - start_pos_axis0
        cur_pos_axis1 = axis1.get_pos_turns() * t2m - start_pos_axis1

        times.append(cur_time)
        pos_axis0.append(cur_pos_axis0)
        pos_axis1.append(cur_pos_axis1)
        vels_axis0.append(cur_vel_axis0)
        vels_axis1.append(cur_vel_axis1)

    brake_both_motors(axis0, axis1)

    plt.plot(times, pos_axis0, label='positions')
    plt.plot(times, pos_axis1)
    plt.savefig(plot_dir+ "position.png")
    plt.clf()

    #generate slope and torque
    slope_avg,slope_axis0,slope_axis1 = get_slope(times, vels_axis0, vels_axis1)
    

    torque_avg = get_torque(slope_avg)
    torque_axis0 = get_torque(slope_axis0)
    torque_axis1 = get_torque(slope_axis1)
    #generate some data to plot the slope
    avg_vel = [slope_avg*x for x in times]

    plt.plot(times, vels_axis0, label='velocity')
    plt.plot(times, vels_axis1)
    plt.plot(times, avg_vel)
    
    plt.title(f"Set Torque:{torque_set:.2f} Nm\n Calced Torque Avg: {torque_avg:.2f} Nm\n  Accel Avg: {slope_avg:.2f} m/s^2\n  Calced Torque Axis0: {torque_axis0:.2f} Nm\n  Accel Axis0: {slope_axis0:.2f} m/s^2\n  Calced Torque Axis1: {torque_axis1:.2f} Nm\n  Accel Axis1: {slope_axis1:.2f} m/s^2,")
    # Adjust spacing to make sure all lines of the title are visible
    plt.subplots_adjust(top=0.7)
    plt.savefig(plot_dir+ "velocities.png")
    plt.clf()

def plot_torques(axis):
    axis.setup()

    axis.trq_set(1.5)

    t2m = 0.528

    start_time = time.time()
    cur_time = time.time() - start_time

    times = []
    iqm_arr = []
    iqs_arr = []
    trqs_arr = []
    trqi_arr = []

    idm_arr = []
    ids_arr = []

    phb_arr = []
    phc_arr = []

    ibus_arr = []

    while (cur_time) < 3:
        iqm_arr.append(axis.get_iq_measured())
        iqs_arr.append(axis.get_iq_setpoint())
        # trqs_arr.append(axis.get_torque_setpoint())
        # trqi_arr.append(axis.get_torque_input())

        ibus_arr.append(axis.get_motor_bus_current())
        # vbus_arr.append(axis.get_bus_voltage())

        idm_arr.append(axis.get_id_measured())
        ids_arr.append(axis.get_id_setpoint())

        phb_arr.append(axis.get_dc_calib_b())
        phc_arr.append(axis.get_dc_calib_c())


        cur_time = time.time() - start_time
        times.append(cur_time)


    plt.plot(times, ibus_arr, label='current')
    plt.savefig(plot_dir+ "motor_bus_current.png")
    plt.clf()

    plt.plot(times, idm_arr, label='id current')
    plt.plot(times, ids_arr)
    plt.savefig(plot_dir+ "id_current.png")
    plt.clf()

    plt.plot(times, iqm_arr, label='iq current')
    plt.plot(times, iqs_arr)
    plt.savefig(plot_dir+ "iq_current.png")
    plt.clf()

    plt.plot(times, iqm_arr, label='iq id current')
    plt.plot(times, idm_arr, label='iq id current')
    plt.plot(times, ids_arr)
    plt.plot(times, iqs_arr)
    plt.plot(times, ibus_arr)
    plt.plot(times, phb_arr)
    plt.plot(times, phc_arr)
    plt.savefig(plot_dir+ "iq_id_current.png")
    plt.clf()

    plt.plot(times, phb_arr)
    plt.plot(times, phc_arr)
    plt.savefig(plot_dir+ "phases.png")
    plt.clf()

    axis.trq_brake()


def brake_both_motors(axis0, axis1):

    while abs(axis0.get_vel()) > 0.05 and abs(axis1.get_vel()) > 0.05:
        axis0.trq_set(-0.5 if axis0.get_vel() > 0 else 0.5)
        axis1.trq_set(-0.5 if axis1.get_vel() > 0 else 0.5)
        
    axis0.trq_set(0)
    axis1.trq_set(0)