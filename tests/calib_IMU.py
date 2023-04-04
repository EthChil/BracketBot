import sys
sys.path.append('../')

import IMU

IMU = IMU.IMU_BNO085()
IMU.setupIMU()

IMU.saveCalibrationConstants()