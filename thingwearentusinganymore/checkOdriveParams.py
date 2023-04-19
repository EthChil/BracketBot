import odrive
from odrive.enums import *
import time
import matplotlib.pyplot as plt
import numpy as np
import IMU
plt.switch_backend('Agg')

odrv0 = odrive.find_any()

# Enums for control modes
CTRL_MODE_VOLTAGE_CONTROL = 0
CTRL_MODE_CURRENT_CONTROL = 1
CTRL_MODE_VELOCITY_CONTROL = 2
CTRL_MODE_POSITION_CONTROL = 3
CTRL_MODE_TRAJECTORY_CONTROL = 4

# Enums for state
AXIS_STATE_UNDEFINED = 0
AXIS_STATE_IDLE = 1
AXIS_STATE_STARTUP_SEQUENCE = 2
AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3
AXIS_STATE_MOTOR_CALIBRATION = 4
AXIS_STATE_ENCODER_INDEX_SEARCH = 6
AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7
AXIS_STATE_CLOSED_LOOP_CONTROL = 8
AXIS_STATE_LOCKIN_SPIN = 9
AXIS_STATE_ENCODER_DIR_FIND = 10
AXIS_STATE_HOMING = 11
AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION = 12

# AXIS 0

print("------- AXIS 0 --------")

# modes
print("Requested State (axis0):", odrv0.axis0.requested_state)
print("Current State (axis0):", odrv0.axis0.current_state)
print("Control Mode (axis0):", odrv0.axis0.controller.config.control_mode)

# limits
print("Velocity Limit (axis0):", odrv0.axis0.controller.config.vel_limit)
print("Current Limit (axis0):", odrv0.axis0.motor.config.current_lim)

# ramp rates
print("Velocity Ramp Rate (axis0):", odrv0.axis0.controller.config.vel_ramp_rate)
print("Torque Ramp Rate (axis0):", odrv0.axis0.controller.config.torque_ramp_rate)

# torque constants
print("Torque Constant (axis0):", odrv0.axis0.motor.config.torque_constant)

# motor
print("Current Measured Phase B (axis0 motor):", odrv0.axis0.motor.current_meas_phB)
print("Current Measured Phase C (axis0 motor):", odrv0.axis0.motor.current_meas_phC)
print("DC Calibration Phase B (axis0 motor):", odrv0.axis0.motor.DC_calib_phB)
print("DC Calibration Phase C (axis0 motor):", odrv0.axis0.motor.DC_calib_phC)
print("Id Setpoint (axis0 motor):", odrv0.axis0.motor.current_control.Id_setpoint)
print("Id Measured (axis0 motor):", odrv0.axis0.motor.current_control.Id_measured)
print("Iq Setpoint (axis0 motor):", odrv0.axis0.motor.current_control.Iq_setpoint)
print("Iq Measured (axis0 motor):", odrv0.axis0.motor.current_control.Iq_measured)

# calibration
print("Motor Calibration Status (axis0):", odrv0.axis0.motor.is_calibrated)
print("Encoder Calibration Status (axis0):", odrv0.axis0.encoder.is_ready)

# errors
print("Axis1 Error:", odrv0.axis1.error)
print("Axis1 Motor Error:", odrv0.axis1.motor.error)
print("Axis1 Controller Error:", odrv0.axis1.controller.error)
print("Axis1 Encoder Error:", odrv0.axis1.encoder.error)
print("Axis1 Sensorless Estimator Error:", odrv0.axis1.sensorless_estimator.error)


# AXIS 1
print("------- AXIS 1--------")


# modes
print("Requested State (axis1):", odrv0.axis1.requested_state)
print("Current State (axis1):", odrv0.axis1.current_state)
print("Control Mode (axis1):", odrv0.axis1.controller.config.control_mode)


# limits
print("Velocity Limit (axis1):", odrv0.axis1.controller.config.vel_limit)
print("Current Limit (axis1):", odrv0.axis1.motor.config.current_lim)

# ramp rates
print("Velocity Ramp Rate (axis1):", odrv0.axis1.controller.config.vel_ramp_rate)
print("Torque Ramp Rate (axis1):", odrv0.axis1.controller.config.torque_ramp_rate)

# torque constant
print("Torque Constant (axis1):", odrv0.axis1.motor.config.torque_constant)


# motor
print("Current Measured Phase B (axis1 motor):", odrv0.axis1.motor.current_meas_phB)
print("Current Measured Phase C (axis1 motor):", odrv0.axis1.motor.current_meas_phC)
print("DC Calibration Phase B (axis1 motor):", odrv0.axis1.motor.DC_calib_phB)
print("DC Calibration Phase C (axis1 motor):", odrv0.axis1.motor.DC_calib_phC)
print("Id Setpoint (axis1 motor):", odrv0.axis1.motor.current_control.Id_setpoint)
print("Id Measured (axis1 motor):", odrv0.axis1.motor.current_control.Id_measured)
print("Iq Setpoint (axis1 motor):", odrv0.axis1.motor.current_control.Iq_setpoint)
print("Iq Measured (axis1 motor):", odrv0.axis1.motor.current_control.Iq_measured)

# calibration
print("Motor Calibration Status (axis1):", odrv0.axis1.motor.is_calibrated)
print("Encoder Calibration Status (axis1):", odrv0.axis1.encoder.is_ready)


# errors
print("Axis0 Error:", odrv0.axis0.error)
print("Axis0 Motor Error:", odrv0.axis0.motor.error)
print("Axis0 Controller Error:", odrv0.axis0.controller.error)
print("Axis0 Encoder Error:", odrv0.axis0.encoder.error)
print("Axis0 Sensorless Estimator Error:", odrv0.axis0.sensorless_estimator.error)


