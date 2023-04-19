import asyncio
import IMU
import time

def run_imu85_process(termination_event, imu_setup, imu_and_odometry_dict):
    
    IMU1 = IMU.IMU_BNO085()
    IMU1.setupIMU()
    imu_setup.set()
    
    alpha = 0.5
    pitch_rate_ewa = 0
    yaw_rate_ewa = 0
    
    _, yaw_angle1 = IMU1.getAngles()
    yaw_start = yaw_angle1

    while not termination_event.is_set():
        pitch_angle1, yaw_angle1 = IMU1.getAngles()
        pitch_rate1, yaw_rate1 = IMU1.getRates()
        
        yaw_angle1 -= yaw_start
        
        pitch_rate_ewa = alpha * pitch_rate1 + (1 - alpha) * pitch_rate_ewa
        yaw_rate_ewa = alpha *  yaw_rate1 + (1 - alpha) * yaw_rate_ewa
        
        imu_and_odometry_dict['pitch_angle85'] = pitch_angle1
        imu_and_odometry_dict['yaw_angle85'] = yaw_angle1
        imu_and_odometry_dict['pitch_rate85'] = pitch_rate1
        imu_and_odometry_dict['yaw_rate85'] = yaw_rate1
        
        imu_and_odometry_dict['pitch_rate85_ewa'] = pitch_rate_ewa
        imu_and_odometry_dict['yaw_rate85_ewa'] = yaw_rate_ewa
        
