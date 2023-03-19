import sys
sys.path.append('../')
import IMU
import time
import matplotlib.pyplot as plt
import math


IMU = IMU.IMU(0, 40)
IMU.setupIMU()
'''
IMU.restoreCalibrationConstants([])
Previous data:
[0, 0, 0, 0, 0, 0, 163, 1, 70, 1, 193, 1, 255, 255, 255, 255, 0, 0, 232, 3, 78, 3]
[254, 255, 83, 0, 15, 0, 137, 1, 67, 1, 36, 1, 0, 0, 0, 0, 1, 0, 232, 3, 196, 2]
[0, 0, 0, 0, 0, 0, 141, 1, 79, 1, 251, 1, 0, 0, 0, 0, 0, 0, 232, 3, 234, 2]
[0, 0, 0, 0, 0, 0, 145, 1, 163, 255, 174, 4, 0, 0, 254, 255, 0, 0, 232, 3, 64, 1]
[13, 0, 89, 0, 251, 255, 120, 1, 31, 0, 156, 4, 0, 0, 255, 255, 0, 0, 232, 3, 170, 1]

[0, 0, 85, 0, 1, 0, 166, 1, 77, 1, 176, 1, 1, 0, 0, 0, 0, 0, 232, 3, 178, 1]
Pretty print: [0, 85, 1, 422, 333, 432, 1, 0, 0, 1000, 434]
'''
calibrationValues = []
while not calibrationValues:
    time.sleep(1)
    print(IMU.getCalibStatus())
    calibrationValues = IMU.saveCalibrationConstants()

# Print raw data
print(calibrationValues)

# Print 2s complement decimal data
calibrationValuesSigDec = []
for j in range(0, len(calibrationValues),2):
        val = (calibrationValues[j+1]<<8) + calibrationValues[j]
        calibrationValuesSigDec.append(IMU.twosComp(val, 16))
print(calibrationValuesSigDec)
