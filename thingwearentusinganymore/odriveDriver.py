import odrive
from odrive.enums import *
import time
class Axis:
    def __init__(self, axis, dir=1):
        self.axis = axis
        self.dir = dir


    def setup(self):
        if self.axis.encoder.is_ready == False:
            self.axis.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
            while self.axis.current_state != AXIS_STATE_IDLE:
                time.sleep(0.1)
                
        if self.axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
            self.axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            
        # self.axis.controller.input_vel = 0
        self.axis.controller.input_torque = 0
        self.set_trq(0)


    def brake(self):
        while abs(self.get_vel()) > 0.05:
            self.trq_set(-0.5 if self.get_vel() > 0 else 0.5)

        self.trq_set(0)
           

    def set_vel(self, v=0):
        if self.axis.controller.config.control_mode != CONTROL_MODE_VELOCITY_CONTROL:
            self.axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
            print("changed to vel control")

        self.axis.controller.input_vel = v * self.dir

    def set_trq(self, t=0):
        self.axis.controller.config.input_mode = 1
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
    
    def get_pos_cts(self):
        return self.axis.encoder.pos_estimate_counts * self.dir#turns

    def accel_test(self):
        self.trq_set(0.5)

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
        plt.savefig(plot_dir+ "vel.png")
        plt.clf()


        plt.plot(times, accels, label='accleration')
        plt.savefig(plot_dir+ "accel.png")