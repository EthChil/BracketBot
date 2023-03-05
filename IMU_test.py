import IMU
import time

IMU = IMU.IMU(0, 40)
IMU.setupIMU()

while True:
    time.sleep(0.05)
    print(IMU.getAngleCompRads())