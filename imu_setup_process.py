import IMU

def setup_imu():

    print("setting up IMU's in imu_setup_process.py")
    IMU1 = IMU.IMU_BNO085()
    IMU1.setupIMU()

    IMU2 = IMU.IMU_BNO055(0, 40)
    IMU2.restoreCalibrationConstants([0, 0, 85, 0, 1, 0, 166, 1, 77, 1, 176, 1, 1, 0, 0, 0, 0, 0, 232, 3, 178, 1]) #only for bno55
    IMU2.setupIMU()
    print("IMU setups done")

