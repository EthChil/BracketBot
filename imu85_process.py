import asyncio
import IMU
import time

# async def get_angles_and_rates(imu):
#     loop = asyncio.get_running_loop()
#     pitch_angle1, yaw_angle1 = await loop.run_in_executor(None, imu.getAngles)
#     pitch_rate1, yaw_rate1 = await loop.run_in_executor(None, imu.getRates)
#     return pitch_angle1, yaw_angle1, pitch_rate1, yaw_rate1

# async def start_imu85(termination_event, imu85_dict):
#     IMU1 = IMU.IMU_BNO085()
#     IMU1.setupIMU()

#     while not termination_event.is_set():
#         pitch_angle1, yaw_angle1, pitch_rate1, yaw_rate1 = await get_angles_and_rates(IMU1)

#         imu85_dict['pitch_angle'] = pitch_angle1
#         imu85_dict['yaw_angle'] = yaw_angle1
#         imu85_dict['pitch_rate'] = pitch_rate1
#         imu85_dict['yaw_rate'] = yaw_rate1
        
#         print(time.time())

        # await asyncio.sleep(0.01)  # Yield control to the event loop, allowing other tasks to run.

def run_imu85_process(termination_event, imu_setup, imu85_dict):
    
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
        
        imu85_dict['pitch_angle'] = pitch_angle1
        imu85_dict['yaw_angle'] = yaw_angle1
        
        imu85_dict['pitch_rate'] = pitch_rate1
        imu85_dict['yaw_rate'] = yaw_rate1
        
        imu85_dict['pitch_rate_ewa'] = pitch_rate_ewa
        imu85_dict['yaw_rate_ewa'] = yaw_rate_ewa
        
