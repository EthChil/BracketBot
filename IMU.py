import time
from smbus2 import SMBus
import math
import numpy as np

maxI2CBusses = 2

class IMU:
    setup = 0

    # initialize the IMU by opening the I2C connection and 
    def __init__(self, bus, chipAdd, maxAddress=0x106, maxData=255):
        self.maxAddr = maxAddress
        self.maxData = maxData
        self.chipAdd = chipAdd

        #for wrapping yaw:
        self.prev_angle = 0
        self.wraps = 0

        #yaw start offset
        self.yaw_start_angle = 0.0

        if (bus > maxI2CBusses or bus < 0):
            print("Error attempting to access I2C bus out of range")

        # open SMBus object
        try:
            self.bus = SMBus(bus)
        except:
            print("Failed to open I2C SMBus on port " + str(bus))

    

    # generic I2C access function for reading
    def readByte(self, offset):
        if (offset > self.maxAddr):
            print("MAX ADDRESS EXCEEDED")
            return -1

        try:
            b = self.bus.read_byte_data(self.chipAdd, offset)
        except:
            print("Failed to read byte from address " + str(offset))
            return -1

        return b

    # generic I2C access function for writing
    def writeByte(self, offset, data):
        if (offset > self.maxAddr):
            print("MAX ADDRESS EXCEEDED")
            return False

        if (data > self.maxData):
            print("MAX DATA LENGTH EXCEEDED")
            return False

        try:
            self.bus.write_byte_data(self.chipAdd, offset, data)
            return True
        except:
            print("Failed to write byte " + str(data) + " to address " + str(offset))
            return False

    # simple twos complement function
    def twosComp(self, val, length):
        bits = length

        if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
            val = val - (1 << bits)        # compute negative value
        return val                         # return positive value as is

    # setup IMU and set flag
    def setupIMU(self):
        #mode information
        #NDOF mode - sensor fusion mode which allows us to access the gravity vector
        time.sleep(1)
        #write RST_SYS in SYS_TRIGGER
        self.writeByte(0x3F, 0x20)
        time.sleep(1)
        print("0x3f 20")
        print(self.readByte(0x3F))

        time.sleep(0.75)

        #write power mode in PWR_MODE
        self.writeByte(0x3E, 0x00)
        time.sleep(0.5)
        print("0x3e 00")
        print(self.readByte(0x3E))

        #write RST_SYS in SYS_TRIGGER
        self.writeByte(0x3F, 0x00)
        time.sleep(0.5)
        print("0x3f 00")
        print(self.readByte(0x3F))

        time.sleep(0.03)

        #Set operation mode in OPR_MODE (this should be NDOF)
        #sensor fusion modes
        # 0c for NDOF
        #8 for IMU
        # 7 for AMG mercedes
        # C for NDOF
        self.writeByte(0x3D, 0x08)
        time.sleep(0.5)
        print("0x3d 08")
        print(self.readByte(0x3D))

        #Set UNIT_SEL (m/s^2 RPS Centigrade)
        self.writeByte(0x80, 0x07)
        time.sleep(0.5)
        print("0x80 07")
        print(self.readByte(0x80))

        #Set ACC_CONFIG to 2g
        self.writeByte(0x08, 0b1100)
        time.sleep(0.5)
        print("0x08 0C")
        print(self.readByte(0x80))
        
        time.sleep(0.5)

        self.setup = 1
        self.yaw_start_angle = self.getYawAngle()


    def getCalibStatus(self):
        stat = self.readByte(0x35)
        return {"Accelerometer" : (stat >> 2) & 0b11 > 0, "Gyro" : (stat >> 4) & 0b11 > 0, "Magnetometer" : stat & 0b11 > 0, "System" : (stat >> 6) & 0b11 > 0}
    
    def saveCalibrationConstants(self):
        if(not self.getCalibStatus()["System"]):
            print("Calibrate fully before saving profile")
            return

        self.writeByte(0x3D, 0x00)
        time.sleep(0.5)
        print("Entering Config Mode")

        calibrationValues = []

        for i in range(0x55, 0x6A, 0x01):
            calibrationValues.append(self.readByte(i))

        self.writeByte(0x3D, 0x08)
        time.sleep(0.5)
        print("Exiting Config Mode")
        return calibrationValues
    
    def restoreCalibrationConstants(self, calibrationValues):
        self.writeByte(0x3D, 0x00)
        time.sleep(0.5)
        print("Entering Config Mode")

        calibrationStep = 0
        for i in range(0x55, 0x6A, 0x01):
            self.writeByte(i, calibrationValues[calibrationStep])

        self.writeByte(0x3D, 0x08)
        time.sleep(0.5)
        print("Exiting Config Mode")


    # get gravity vector only available in fusion modes
    def getGravityVector(self):
        if(not self.setup):
            print("ERROR IMU has not been initialized")
            return [-1, -1, -1]

        rawZ = 255
        rawX = 255
        rawY = 255

        while(abs(rawX - 255) < 5 or abs(rawX - 65280) < 5):
            XMSB = self.readByte(0x33)
            XLSB = self.readByte(0x32)
            rawX = (XMSB<<8) + XLSB

        while(abs(rawY - 255) < 5 or abs(rawY - 65280) < 5):
            YMSB = self.readByte(0x31)
            YLSB = self.readByte(0x30)
            rawY = (YMSB<<8) + YLSB

        # yo heuristics
        while(abs(rawZ - 255) < 5 or abs(rawZ - 65280) < 5):
            ZMSB = self.readByte(0x2F)
            ZLSB = self.readByte(0x2E)
            rawZ = (ZMSB<<8) + ZLSB

        return [self.twosComp(rawX, 16)/100, 
                self.twosComp(rawY, 16)/100, 
                self.twosComp(rawZ, 16)/100]




    # get relative yaw angle from start, in documentation this is the eularian using a fusion combination of accelerometer and gyro
    def getYawAngle(self):
        if(not self.setup):
            print("ERROR IMU has not been initialized")
            return -1

        rawZ = 255
        rawX = 255
        rawY = 255

        XMSB = self.readByte(0x1B)
        XLSB = self.readByte(0x1A)
        rawX = self.twosComp((XMSB<<8) + XLSB, 16)/900
        YMSB = self.readByte(0x1D)
        YLSB = self.readByte(0x1C)
        rawY = self.twosComp((YMSB<<8) + YLSB, 16)/900
        ZMSB = self.readByte(0x1F)
        ZLSB = self.readByte(0x1E)
        rawZ = self.twosComp((ZMSB<<8) + ZLSB, 16)/900

        # units are in rad from 0 -> 2pi
        yaw = rawX

        #measured in radians this will go from -pi to +pi
        norm = math.atan2(math.sin(yaw), math.cos(yaw)) - self.yaw_start_angle

        if norm - self.prev_angle > math.pi:
            self.wraps -= 1
        elif self.prev_angle - norm > math.pi:
            self.wraps += 1

        self.prev_angle = norm
        continuous_angle = norm + 2*math.pi*self.wraps

        return continuous_angle



    # calculate angle based on the angle of the gravity vector and a trim value for what it is while balanced
    def getPitchAngle(self):
        # adjust = -0.007142735669418479#rads
        # adjust = -0.11494284164306612
        adjust = -0.10796626140477657

        referenceAxis = np.array([1, 0, 0]) # for top mount

        vec = np.array(self.getGravityVector())

        return (math.acos(np.dot(referenceAxis, vec)/(np.linalg.norm(referenceAxis)*np.linalg.norm(vec)))) + adjust

    # gyro pitch rate
    def getPitchRate(self):
        # pull the wogma from the sensor
        # rotate the 
        if(not self.setup):
            print("ERROR IMU has not been initialized UN SMART INDIVIDUAL")
            return [-1, -1, -1]

        XMSB = self.readByte(0x15)
        XLSB = self.readByte(0x14)
        YMSB = self.readByte(0x17)
        YLSB = self.readByte(0x16)
        ZMSB = self.readByte(0x19)
        ZLSB = self.readByte(0x18)

        #900LSB is equal to 1rps
        vec = [self.twosComp((XMSB<<8) + XLSB, 16)/900,
                self.twosComp((YMSB<<8) + YLSB, 16)/900,
                self.twosComp((ZMSB<<8) + ZLSB, 16)/900]

        return vec[0]

    # gyro yaw rate
    def getYawRate(self):
        # pull the wogma from the sensor
        # rotate the 
        if(not self.setup):
            print("ERROR IMU has not been initialized UN SMART INDIVIDUAL")
            return [-1, -1, -1]

        XMSB = self.readByte(0x15)
        XLSB = self.readByte(0x14)
        YMSB = self.readByte(0x17)
        YLSB = self.readByte(0x16)
        ZMSB = self.readByte(0x19)
        ZLSB = self.readByte(0x18)

        #900LSB is equal to 1rad/s
        vec = [self.twosComp((XMSB<<8) + XLSB, 16)/900,
                self.twosComp((YMSB<<8) + YLSB, 16)/900,
                self.twosComp((ZMSB<<8) + ZLSB, 16)/900]

        return vec[2]
    
