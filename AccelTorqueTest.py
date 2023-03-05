
import odrive
from odrive.enums import *
import time
import matplotlib.pyplot as plt
import numpy as np
import IMU
plt.switch_backend('Agg')

IMU = IMU.IMU(0, 40)
IMU.setupIMU()

odrv0 = odrive.find_any()

ref_time = time.time()
t2m = 0.528

# K = np.array([-10.0000, -11.3648, -43.2605, -15.2819])
K = np.array([-31.6228, -30.5921, -92.5864, -32.4303])


def LQR(axis0, axis1):
    #read initial values to offset
    x_init = axis0.get_pos_turns() * t2m
    cur_theta = IMU.getAngleCompRads()
    prev_theta = cur_theta
    wogma = IMU.getAngleCompRads()*1000
    prev_x = 0

    Xf = np.array([0.5, 0, 0, 0])

    start_time = time.time()
    cur_time = time.time() - start_time
    prev_time = time.time() - start_time

    #SAVE THINGS TO PLOT
    times = []
    dts = []
    xs = []
    vs = []
    dxdts = []
    cur_thetas = []
    wogmas = []
    torque_commands = []

    while cur_time < 3:
        time.sleep(0.001)
        cur_time = time.time() - start_time #relative time starts at 0
        dt = cur_time - prev_time

        x = axis0.get_pos_turns()*t2m - x_init
        v = axis0.get_vel()*t2m
        cur_theta = IMU.getAngleCompRads()
        wogma = (cur_theta - prev_theta)/dt

        X = np.array([x, v, cur_theta, wogma])
        U = -K @ (X-Xf)

        torque_command = U/2# if (U/2)<3 else 3

        axis0.trq_set(torque_command)
        axis1.trq_set(torque_command)

        times.append(cur_time)
        dts.append(dt)
        xs.append(x)
        vs.append(v)
        dxdts.append((x-prev_x)/dt)
        cur_thetas.append(cur_theta)
        wogmas.append(wogma)
        torque_commands.append(torque_command)

        # print('cur_time: ', cur_time)
        # print('dt: ', dt)
        # print('x: ', x)
        # print('v: ', v)
        # print('dx/dt: ', (x-prev_x)/dt)
        # print('cur_theta: ', cur_theta)
        # print('wogma: ', wogma)
        # print('----------------------')
        # print('ivans torque: ', U)
        # print('----------------------')

        prev_theta = cur_theta
        prev_time = cur_time
        prev_x = x

    brake_both_motors(a0, a1)

    plt.plot(times, dts, label='dts')
    plt.plot(times, xs, label='xs')
    plt.plot(times, vs, label='vs')
    plt.plot(times, dxdts, label='dxdts')
    plt.plot(times, cur_thetas, label='cur_thetas')
    plt.plot(times, wogmas, label='wogmas')

    plt.legend()
    plt.savefig("balance_params.png")
    plt.clf()

    plt.plot(times, torque_commands, label='torque')
    plt.legend()
    plt.savefig("balance_torque.png")
    plt.clf()

    

def get_slope(x, y1, y2):
    # Fit a straight line to the points using NumPy
    fit1 = np.polyfit(x, y1, 1)
    fit2 = np.polyfit(x, y2, 1)
    # The slope of the line is the acceleration

    avg_slope = (fit1[0] + fit2[0])/2
    return avg_slope, fit1[0], fit2[0]

def get_torque(accel):
    return 0.001 * accel / 0.0846 + 9.31 * accel * 0.0846 / 2

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
    plt.savefig("motor_bus_current.png")
    plt.clf()

    plt.plot(times, idm_arr, label='id current')
    plt.plot(times, ids_arr)
    plt.savefig("id_current.png")
    plt.clf()

    plt.plot(times, iqm_arr, label='iq current')
    plt.plot(times, iqs_arr)
    plt.savefig("iq_current.png")
    plt.clf()

    plt.plot(times, iqm_arr, label='iq id current')
    plt.plot(times, idm_arr, label='iq id current')
    plt.plot(times, ids_arr)
    plt.plot(times, iqs_arr)
    plt.plot(times, ibus_arr)
    plt.plot(times, phb_arr)
    plt.plot(times, phc_arr)
    plt.savefig("iq_id_current.png")
    plt.clf()

    plt.plot(times, phb_arr)
    plt.plot(times, phc_arr)
    plt.savefig("phases.png")
    plt.clf()

    axis.trq_brake()



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
    plt.savefig("position.png")
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
    plt.savefig("velocities.png")
    plt.clf()

def brake_both_motors(axis0, axis1):

    while abs(axis0.get_vel()) > 0.05 and abs(axis1.get_vel()) > 0.05:
        axis0.trq_set(-0.5 if axis0.get_vel() > 0 else 0.5)
        axis1.trq_set(-0.5 if axis1.get_vel() > 0 else 0.5)
        
    axis0.trq_set(0)
    axis1.trq_set(0)


class Axis:
    def __init__(self, axis, dir=1):
        self.axis = axis
        self.dir = dir


    def setup(self):
        if self.axis.encoder.is_ready == False:
            self.axis.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
            while self.axis.current_state != AXIS_STATE_IDLE:
                time.sleep(0.1)

        # print(f"{self.axis} encoder is indexed")
        self.axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.axis.controller.input_vel = 0
        self.axis.controller.input_torque = 0

        self.trq_set()

    

    def trq_brake(self):
        while abs(self.get_vel()) > 0.05:
            self.trq_set(-0.5 if self.get_vel() > 0 else 0.5)

        self.trq_set(0)
           

    def vel_set(self, v=0):
        if self.axis.controller.config.control_mode != CONTROL_MODE_VELOCITY_CONTROL:
            self.axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
            print("changed to vel control")

        self.axis.controller.input_vel = v * self.dir

    def trq_set(self, t=0):
        if self.axis.controller.config.control_mode != CONTROL_MODE_TORQUE_CONTROL:
            self.axis.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
            print("changed to torque control")
        
        self.axis.controller.input_torque = t * self.dir

    def stop(self):
        self.axis.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
        self.axis.controller.input_vel = 0
        self.axis.controller.input_torque = 0

    def get_id_measured(self):
        return self.axis.motor.current_control.Id_measured

    def get_id_setpoint(self):
        return self.axis.motor.current_control.Id_setpoint

    def get_iq_measured(self):
        return self.axis.motor.current_control.Iq_measured

    def get_iq_setpoint(self):
        return self.axis.motor.current_control.Iq_setpoint

    def get_torque_setpoint(self):
        return self.axis.controller.torque_setpoint

    def get_torque_input(self):
        return self.axis.controller.input_torque
    
    def get_motor_bus_current(self):
        return self.axis.motor.current_control.Ibus

    def get_dc_calib_b(self):
        return self.axis.motor.DC_calib_phB

    def get_dc_calib_c(self):
        return self.axis.motor.DC_calib_phC



    def get_vel(self):
        return self.axis.encoder.vel_estimate * self.dir#turns/s

    def get_vel_cts(self):
        return self.axis.encoder.vel_estimate_counts * self.dir#counts/s

    def get_pos_turns(self):
        return self.axis.encoder.pos_estimate * self.dir#turns

    def accel_test(self):
        self.trq_set(1)

        ref_time = time.time()

        times = []
        vels = []
        accels = []

        prev_vel = 0
        prev_time = time.time() - ref_time
        prev_accel = 0

        saturated = False
        continous_saturation_time = 2 #seconds
        continous_saturation_time_start = time.time() - ref_time

        while not saturated:
            cur_time = time.time() - ref_time
            cur_vel = a0.get_vel()

            cur_accel = (cur_vel-prev_vel)/(cur_time-prev_time)
            jerk = cur_accel - prev_accel

            vels.append(cur_vel)
            accels.append(cur_accel)
            times.append(cur_time)

            

            if (cur_vel - prev_vel) < 0.5:
                timing = (cur_time - continous_saturation_time_start) > continous_saturation_time
                if timing:
                    saturated = True
            
            else:
                continous_saturation_time_start = time.time() - ref_time

            prev_vel = cur_vel
            prev_time = cur_time
            prev_accel = cur_accel



        plt.plot(times, vels, label='velocity')
        plt.savefig("vel.png")
        plt.clf()


        plt.plot(times, accels, label='accleration')
        plt.savefig("accel.png")

        
a0 = Axis(odrv0.axis0)
a1 = Axis(odrv0.axis1, dir=-1)

a0.setup()
a1.setup()

# straight_accel_test(a0, a1, 0.75)

# plot_torques(a0)

# a0.trq_set(0.75)
# a1.trq_set(0.75)

LQR(a0, a1)

brake_both_motors(a0, a1)




