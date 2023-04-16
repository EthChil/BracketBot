import IMU
import time

def start_imu(IMU85_dict, IMU55_dict, imu_setup_done, termination_event):

    print("setting up IMU's in imu_process.py")
    IMU1 = IMU.IMU_BNO085()
    IMU1.setupIMU()

    IMU2 = IMU.IMU_BNO055(0, 40)
    IMU2.restoreCalibrationConstants([0, 0, 85, 0, 1, 0, 166, 1, 77, 1, 176, 1, 1, 0, 0, 0, 0, 0, 232, 3, 178, 1]) #only for bno55
    IMU2.setupIMU()
    print("IMU setups done")
    imu_setup_done.set() 
    
    
    pitch_angle1_init, yaw_angle1_init = IMU1.getAngles()
    
    while not termination_event.is_set():

        pitch_angle1, yaw_angle1 = IMU1.getAngles()
        pitch_rate1, yaw_rate1 = IMU1.getRates()
        
        IMU85_dict['pitch_angle'] = -pitch_angle1
        IMU85_dict['pitch_rate'] = pitch_rate1
        IMU85_dict['yaw_angle'] = -(yaw_angle1-yaw_angle1_init)
        IMU85_dict['yaw_rate'] = -yaw_rate1
        
        # IMU55_dict['yaw_angle'] = yaw_angle2
        # IMU55_dict['pitch_angle'] = pitch_angle2
        # IMU55_dict['pitch_rate'] = pitch_rate2
        # IMU55_dict['yaw_rate'] = yaw_rate2
        
        IMU55_dict['yaw_angle'] = 0
        IMU55_dict['pitch_angle'] = 0
        IMU55_dict['pitch_rate'] = 0
        IMU55_dict['yaw_rate'] = 0
        
        
        
    
    

