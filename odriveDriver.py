
import odrive
from odrive.enums import *
import time

odrv0 = odrive.find_any()

class Axis:
    def __init__(self, axis):
        self.axis = axis

    def setup(self):
        self.axis.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
        while self.axis.current_state != AXIS_STATE_IDLE:
            time.sleep(0.1)

        print(f"{self.axis} encoder is indexed")
        self.axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL



    def vel_set(self, v=0):
        if self.axis.controller.config.control_mode not CONTROL_MODE_VELOCITY_CONTROL:
            self.axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
            print("changed to vel control")

        self.axis.controller.input_vel = v

    def trq_set(self, t=0):
        if self.axis.controller.config.control_mode not CONTROL_MODE_TORQUE_CONTROL:
            self.axis.controller.config.control_mode = CONTROl_MODE_TORQUE_CONTROL
            print("changed to torque control")
        
        self.axis.controller.input_torque = t


        

a0 = Axis(odrv0.axis0)
a0.setup()

a1 = Axis(odrv0.axis1)
a1.setup()


