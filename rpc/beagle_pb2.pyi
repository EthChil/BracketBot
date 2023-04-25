from google.protobuf.internal import containers as _containers
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class Jetson_to_beagle(_message.Message):
    __slots__ = ["keyboard_cmd", "xf"]
    KEYBOARD_CMD_FIELD_NUMBER: _ClassVar[int]
    XF_FIELD_NUMBER: _ClassVar[int]
    keyboard_cmd: int
    xf: _containers.RepeatedScalarFieldContainer[float]
    def __init__(self, keyboard_cmd: _Optional[int] = ..., xf: _Optional[_Iterable[float]] = ...) -> None: ...

class Logging(_message.Message):
    __slots__ = ["dt_imu85_process", "dt_logging_process", "dt_moteus_control", "moteus1_current_position", "moteus1_current_velocity", "moteus1_torque_command", "moteus1_torque_reading", "moteus2_current_position", "moteus2_current_velocity", "moteus2_torque_command", "moteus2_torque_reading", "pitch_angle85", "pitch_rate85", "pitch_rate85_ewa", "theta_ego", "time", "x_ego", "x_pitch_angle", "x_pitch_rate", "x_position", "x_velocity", "x_yaw_angle", "x_yaw_rate", "y_ego", "yaw_angle85", "yaw_rate85", "yaw_rate85_ewa", "z_pitch_angle", "z_pitch_rate", "z_position", "z_velocity", "z_yaw_angle", "z_yaw_rate"]
    DT_IMU85_PROCESS_FIELD_NUMBER: _ClassVar[int]
    DT_LOGGING_PROCESS_FIELD_NUMBER: _ClassVar[int]
    DT_MOTEUS_CONTROL_FIELD_NUMBER: _ClassVar[int]
    MOTEUS1_CURRENT_POSITION_FIELD_NUMBER: _ClassVar[int]
    MOTEUS1_CURRENT_VELOCITY_FIELD_NUMBER: _ClassVar[int]
    MOTEUS1_TORQUE_COMMAND_FIELD_NUMBER: _ClassVar[int]
    MOTEUS1_TORQUE_READING_FIELD_NUMBER: _ClassVar[int]
    MOTEUS2_CURRENT_POSITION_FIELD_NUMBER: _ClassVar[int]
    MOTEUS2_CURRENT_VELOCITY_FIELD_NUMBER: _ClassVar[int]
    MOTEUS2_TORQUE_COMMAND_FIELD_NUMBER: _ClassVar[int]
    MOTEUS2_TORQUE_READING_FIELD_NUMBER: _ClassVar[int]
    PITCH_ANGLE85_FIELD_NUMBER: _ClassVar[int]
    PITCH_RATE85_EWA_FIELD_NUMBER: _ClassVar[int]
    PITCH_RATE85_FIELD_NUMBER: _ClassVar[int]
    THETA_EGO_FIELD_NUMBER: _ClassVar[int]
    TIME_FIELD_NUMBER: _ClassVar[int]
    X_EGO_FIELD_NUMBER: _ClassVar[int]
    X_PITCH_ANGLE_FIELD_NUMBER: _ClassVar[int]
    X_PITCH_RATE_FIELD_NUMBER: _ClassVar[int]
    X_POSITION_FIELD_NUMBER: _ClassVar[int]
    X_VELOCITY_FIELD_NUMBER: _ClassVar[int]
    X_YAW_ANGLE_FIELD_NUMBER: _ClassVar[int]
    X_YAW_RATE_FIELD_NUMBER: _ClassVar[int]
    YAW_ANGLE85_FIELD_NUMBER: _ClassVar[int]
    YAW_RATE85_EWA_FIELD_NUMBER: _ClassVar[int]
    YAW_RATE85_FIELD_NUMBER: _ClassVar[int]
    Y_EGO_FIELD_NUMBER: _ClassVar[int]
    Z_PITCH_ANGLE_FIELD_NUMBER: _ClassVar[int]
    Z_PITCH_RATE_FIELD_NUMBER: _ClassVar[int]
    Z_POSITION_FIELD_NUMBER: _ClassVar[int]
    Z_VELOCITY_FIELD_NUMBER: _ClassVar[int]
    Z_YAW_ANGLE_FIELD_NUMBER: _ClassVar[int]
    Z_YAW_RATE_FIELD_NUMBER: _ClassVar[int]
    dt_imu85_process: float
    dt_logging_process: float
    dt_moteus_control: float
    moteus1_current_position: float
    moteus1_current_velocity: float
    moteus1_torque_command: float
    moteus1_torque_reading: float
    moteus2_current_position: float
    moteus2_current_velocity: float
    moteus2_torque_command: float
    moteus2_torque_reading: float
    pitch_angle85: float
    pitch_rate85: float
    pitch_rate85_ewa: float
    theta_ego: float
    time: float
    x_ego: float
    x_pitch_angle: float
    x_pitch_rate: float
    x_position: float
    x_velocity: float
    x_yaw_angle: float
    x_yaw_rate: float
    y_ego: float
    yaw_angle85: float
    yaw_rate85: float
    yaw_rate85_ewa: float
    z_pitch_angle: float
    z_pitch_rate: float
    z_position: float
    z_velocity: float
    z_yaw_angle: float
    z_yaw_rate: float
    def __init__(self, time: _Optional[float] = ..., x_ego: _Optional[float] = ..., y_ego: _Optional[float] = ..., theta_ego: _Optional[float] = ..., moteus1_current_position: _Optional[float] = ..., moteus2_current_position: _Optional[float] = ..., moteus1_current_velocity: _Optional[float] = ..., moteus2_current_velocity: _Optional[float] = ..., moteus1_torque_reading: _Optional[float] = ..., moteus2_torque_reading: _Optional[float] = ..., moteus1_torque_command: _Optional[float] = ..., moteus2_torque_command: _Optional[float] = ..., dt_moteus_control: _Optional[float] = ..., pitch_angle85: _Optional[float] = ..., yaw_angle85: _Optional[float] = ..., pitch_rate85: _Optional[float] = ..., yaw_rate85: _Optional[float] = ..., pitch_rate85_ewa: _Optional[float] = ..., yaw_rate85_ewa: _Optional[float] = ..., dt_imu85_process: _Optional[float] = ..., dt_logging_process: _Optional[float] = ..., z_position: _Optional[float] = ..., z_velocity: _Optional[float] = ..., z_pitch_angle: _Optional[float] = ..., z_pitch_rate: _Optional[float] = ..., z_yaw_angle: _Optional[float] = ..., z_yaw_rate: _Optional[float] = ..., x_position: _Optional[float] = ..., x_velocity: _Optional[float] = ..., x_pitch_angle: _Optional[float] = ..., x_pitch_rate: _Optional[float] = ..., x_yaw_angle: _Optional[float] = ..., x_yaw_rate: _Optional[float] = ...) -> None: ...

class Logging_Packet(_message.Message):
    __slots__ = ["logs"]
    LOGS_FIELD_NUMBER: _ClassVar[int]
    logs: _containers.RepeatedCompositeFieldContainer[Logging]
    def __init__(self, logs: _Optional[_Iterable[_Union[Logging, _Mapping]]] = ...) -> None: ...
