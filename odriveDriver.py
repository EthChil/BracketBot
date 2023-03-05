
import odrive
from odrive.enums import *
import time


odrv0 = odrive.find_any()

class Axis:
    def __init__(self, axis):
        self.axis = axis

    def setup(self):
        if self.axis.encoder.is_ready == False:
            self.axis.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
            while self.axis.current_state != AXIS_STATE_IDLE:
                time.sleep(0.1)

        print(f"{self.axis} encoder is indexed")
        self.axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.axis.controller.input_vel = 0
        self.axis.controller.input_torque = 0

        self.trq_set()


    def vel_set(self, v=0):
        if self.axis.controller.config.control_mode != CONTROL_MODE_VELOCITY_CONTROL:
            self.axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
            print("changed to vel control")

        self.axis.controller.input_vel = v

    def trq_set(self, t=0):
        if self.axis.controller.config.control_mode != CONTROL_MODE_TORQUE_CONTROL:
            self.axis.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
            print("changed to torque control")
        
        self.axis.controller.input_torque = t

    def stop(self):
        self.axis.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
        self.axis.controller.input_vel = 0
        self.axis.controller.input_torque = 0

    def get_vel(self):
        return self.axis.encoder.vel_estimate


#a0 = Axis(odrv0.axis0)

#a1 = Axis(odrv0.axis1)


