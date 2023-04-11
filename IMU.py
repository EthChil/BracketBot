import time                                                 
import board
from smbus2 import SMBus
import math
import numpy as np
import busio


# import sys
# sys.path.insert(1, 'Adafruit_CircuitPython_BNO08x/')
from Adafruit_CircuitPython_BNO08x.adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GRAVITY,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)
from Adafruit_CircuitPython_BNO08x.adafruit_bno08x.i2c import BNO08X_I2C


maxI2CBusses = 2

class IMU_BNO085:
    setup = 0

    # initialize the IMU by opening the I2C connection and zeroing yaw
    def __init__(self):
        #for wrapping yaw:
        self.prev_angle = 0
        self.wraps = 0
                                         
        #yaw start offset
        self.yaw_start_angle = 0.0

        # open SMBus object
        try:
            self.bus = busio.I2C(board.SCL_1, board.SDA_1, frequency=400000)
            self.bno = BNO08X_I2C(self.bus)
        except:
            print("Failed to open I2C SMBus on port " + str(self.bus))

    # simple twos complement function
    def twosComp(self, val, length):
        bits = length

        if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
            val = val - (1 << bits)        # compute negative value
        return val                         # return positive value as is

    import math
 
    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

    # setup IMU and set flag
    def setupIMU(self):
        self.bno.initialize()

        self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        self.bno.enable_feature(BNO_REPORT_GRAVITY)
        self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
        self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)
        self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

        time.sleep(1)
        
        self.setup = 1
        self.yaw_start_angle = self.getYawAngle()

    def getCalibStatus(self):
        return self.bno.calibration_status
    
    def saveCalibrationConstants(self):
        print(self.getCalibStatus())
        if(self.getCalibStatus()):
            try:
                self.bno.save_calibration_data()
                print("Calibration saved")
            except:
                print("Failed to save calibration data")
        else:
            print("sensor not calibrated yet")
            self.bno.begin_calibration()
    
    #no adafruit function exists to do this
    def restoreCalibrationConstants(self, calibrationValues):
        return None

    # get gravity vector not natively supported in adafruit libraryS
    def getGravityVector(self):
        (q_i, q_j, q_k, q_real) = self.bno.quaternion
        grav_vec = self.euler_from_quaternion(q_i, q_j, q_k, q_real)

        return grav_vec

    # yaw angle from quaternionn 
    def getYawAngle(self):
        if(not self.setup):
            print("ERROR IMU has not been initialized")
            return None

        (q_i, q_j, q_k, q_real) = self.bno.quaternion
        (roll, pitch, yaw) = self.euler_from_quaternion(q_i, q_j, q_k, q_real)

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
        if(not self.setup):
            print("ERROR IMU has not been initialized")
            return None
        adjust = math.radians(0.15)

        return self.getGravityVector()[1] - adjust

    # gyro pitch rate
    def getPitchRate(self):
        # pull the wogma from the sensor
        # rotate the 
        if(not self.setup):
            print("ERROR IMU has not been initialized UN SMART INDIVIDUAL")
            return None

        (gyro_x, gyro_y, gyro_z) = self.bno.gyro

        return gyro_z # 0 for top mount, 2 for base mount

    # gyro yaw rate
    def getYawRate(self):
        # pull the wogma from the sensor
        # rotate the 
        if(not self.setup):
            print("ERROR IMU has not been initialized UN SMART INDIVIDUAL")
            return None

        (gyro_x, gyro_y, gyro_z) = self.bno.gyro

        return gyro_y # 0 for top mount, 2 for base mount
    


class IMU_BNO055:
    setup = 0

    # initialize the IMU by opening the I2C connection and 
    def __init__(self, bus, chipAdd, maxAddress=0x106, maxData=255, useBusIO=False):
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

        if(self.getCalibStatus()["Accelerometer"] == 3 and self.getCalibStatus()["Gyro"] == 3):
            print("already calibrated that's wild")
            self.setup = 1
            self.yaw_start_angle = self.getYawAngle()
            return

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
        self.writeByte(0x3D, 0x08) #08
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

    def getCalibStatus(self):
        stat = self.readByte(0x35)
        return {"Accelerometer" : (stat >> 2) & 0b11, "Gyro" : (stat >> 4) & 0b11, "Magnetometer" : stat & 0b11, "System" : (stat >> 6) & 0b11}
    
    def saveCalibrationConstants(self):
        print(self.getCalibStatus())
        if sum(self.getCalibStatus().values()) < 12:
            print("Calibrate fully before saving profile")
            return
        prevMode = self.readByte(0x3D)
        self.writeByte(0x3D, 0x00)
        time.sleep(0.5)
        print("Entering Config Mode")

        calibrationValues = []

        for i in range(0x55, 0x6B, 0x01):
            calibrationValues.append(self.readByte(i))

        self.writeByte(0x3D, prevMode)
        time.sleep(0.5)
        print("Exiting Config Mode")
        return calibrationValues
    
    def restoreCalibrationConstants(self, calibrationValues):
        prevMode = self.readByte(0x3D)
        self.writeByte(0x3D, 0x00)
        time.sleep(0.5)
        print("Entering Config Mode")

        calibrationStep = 0
        for i in range(0x55, 0x6B, 0x01):
            # print(f"Address: {format(i, '#x')}, Value: {calibrationValues[calibrationStep]}")
            self.writeByte(i, calibrationValues[calibrationStep])
            calibrationStep += 1

        self.writeByte(0x3D, prevMode)
        time.sleep(0.5)
        print("Exiting Config Mode")


    # get gravity vector only available in fusion modes
    def getGravityVector(self):
        if(not self.setup):
            print("ERROR IMU has not been initialized")
            return None

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
            return None

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
        yaw = rawX # base mount is rawX

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
        if(not self.setup):
            print("ERROR IMU has not been initialized")
            return None
        adjust = -math.pi/2 - 0.0279253 + math.radians(0.73)

        referenceAxis = np.array([0, 0, 1]) # 1, 0, 0 for top mounting on front/back of extrusion, 0, 0, 1 for conventional mount

        vec = np.array(self.getGravityVector())

        return (math.acos(np.dot(referenceAxis, vec)/(np.linalg.norm(referenceAxis)*np.linalg.norm(vec)))) + adjust

    # gyro pitch rate
    def getPitchRate(self):
        # pull the wogma from the sensor
        # rotate the 
        if(not self.setup):
            print("ERROR IMU has not been initialized UN SMART INDIVIDUAL")
            return None

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

        return vec[2] # 0 for top mount, 2 for base mount

    # gyro yaw rate
    def getYawRate(self):
        # pull the wogma from the sensor
        # rotate the 
        if(not self.setup):
            print("ERROR IMU has not been initialized UN SMART INDIVIDUAL")
            return None

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

        return vec[1] #2 for top mount, 1 for base mount
    
