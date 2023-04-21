#!/usr/bin/python3
import time
import serial
import numpy as np

from scipy.signal import savgol_coeffs


def parse_message(message):
    data = message.split(',')
    values = [float(x) for x in data]
    return values

def initial_read(ser):
    buffer = ''
    
    while True:
        # Read incoming data from the serial port
        raw_data = ser.read(ser.in_waiting).decode('utf-8')
        if not raw_data:
            continue

        # Add raw_data to the buffer
        buffer += raw_data

        # Find the start and end indices of the message
        start_idx = buffer.find('s')
        end_idx = buffer.find('e', start_idx)

        # If both start and end bits are found, process the message and return
        if start_idx != -1 and end_idx != -1:
            message = buffer[start_idx + 1:end_idx]
            values = parse_message(message)
            print("Initial read values:", values)
            return values[1]

def savgol_filter_online(data, window_length, polyorder):
    if len(data) < window_length:
        return np.zeros_like(data)
    coeffs = savgol_coeffs(window_length, polyorder)
    return np.convolve(data, coeffs[::-1], mode='valid')

zero_angle_adjust = 0.0 #shits mounted perfectly

def run_uart_imu_process(termination_event, imu_setup, imu_shared_array):
    
    serial_port = serial.Serial(
        port="/dev/ttyTHS1",
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
    )
    # Wait a second to let the port initialize
    time.sleep(2)
    
    serial_port.flushInput()

    start_yaw = initial_read(serial_port)
    # imu_setup.set()
            
    serial_port.flushInput()
    prev_time = time.time()
    
    buffer = ''   
    
    window_length = 21  # Must be odd
    polyorder = 3  # Polynomial order
    
    # Initialize variables to store pitch and yaw rate values
    pitch_rate_values = []
    yaw_rate_values = []

    while not termination_event.is_set():
        cur_time = time.time()
        dt = cur_time - prev_time
        prev_time = cur_time
        # time.sleep(0.002)
        if dt>0.005:
            print("got slow: ", dt)
            serial_port.flushInput()
        
        raw_data = serial_port.read(serial_port.in_waiting).decode('utf-8')
        if not raw_data:
            continue

        buffer += raw_data

        start_idx = buffer.find('s')
        end_idx = buffer.find('e', start_idx)

        if start_idx != -1 and end_idx != -1:
            message = buffer[start_idx + 1:end_idx]
            values = parse_message(message)
            
            imu_data = []
            
            # Collect initial samples in buffer
            pitch_rate_values.append(values[2])
            yaw_rate_values.append(values[3])

            # Start filtering once the buffer has enough samples
            if len(pitch_rate_values) > window_length:
                filtered_pitch_rate = savgol_filter_online(pitch_rate_values[-window_length:], window_length, polyorder)
                filtered_yaw_rate = savgol_filter_online(yaw_rate_values[-window_length:], window_length, polyorder)
            else:
                filtered_pitch_rate = [0]
                filtered_yaw_rate = [0]
                
            if len(pitch_rate_values) > 50:
                imu_setup.set()
                
        
            imu_data.append(values[0] - zero_angle_adjust)
            imu_data.append(values[1]-start_yaw)
            imu_data.append(filtered_pitch_rate[0])
            imu_data.append(filtered_yaw_rate[0])
            imu_data.append(dt)
            
            with imu_shared_array.get_lock():
                imu_shared_array[:] = imu_data
                
            buffer = buffer[end_idx + 1:]
            
        else:
            print("INVALID DATA LENGH")   
    