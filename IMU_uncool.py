import time
from smbus2 import SMBus
import math

maxI2CBusses = 2

class IMU:
    setup = 0

    def __init__(self, bus, chipAdd, maxAddress=106, maxData=255):
        self.maxAddr = maxAddress
        self.maxData = maxData
        self.chipAdd = chipAdd

        if (bus > maxI2CBusses or bus < 0):
            print("Error attempting to access I2C bus out of range")

        # open SMBus object
        try:
            self.bus = SMBus(bus)
        except:
            print("Failed to open I2C SMBus on port " + str(bus))

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

    def twosComp(self, val, length):
        bits = length

        if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
            val = val - (1 << bits)        # compute negative value
        return val                         # return positive value as is

    # def twosComp(self, num, length):
    #     # get the number of bits required to represent the integer
    #     bits = num.bit_length()

    #     # if the most significant bit is 1, it's a negative number
    #     if (num >> (bits - 1)) & 1:
    #         # convert to negative signed integer using two's complement
    #         num = num - (1 << bits)

    #     return num

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
        # 7 for AMG mercedes
        self.writeByte(0x3D, 0x08)
        time.sleep(0.5)
        print("0x3d 0c")
        print(self.readByte(0x3D))

        #Seet UNIT_SEL (m/s^2 RPS Centigrade)
        self.writeByte(0x80, 0x03)
        time.sleep(0.5)
        print("0x3d 0c")
        print(self.readByte(0x80))

        time.sleep(0.5)

        self.setup = 1


    #Gravity vector is a 3D vector with 100 LSB equal to 1m/s^2 (9.8m/s^2 would be 980 or 3D4)
    # def getGravityVector(self):
    #     if(not self.setup):
    #         print("ERROR IMU has not been initialized")
    #         return [-1, -1, -1]

    #     rawX = 0
    #     rawZ = 0
    #     rawY = 0

    #     while(not rawX.bit_length()):
    #         XMSB = self.readByte(0x33)
    #         XLSB = self.readByte(0x32)
    #         rawX = (XMSB<<8) + XLSB

    #     while(not rawY.bit_length()):
    #         YMSB = self.readByte(0x31)
    #         YLSB = self.readByte(0x30)
    #         rawY = (YMSB<<8) + YLSB

    #     while(not rawZ.bit_length()):
    #         ZMSB = self.readByte(0x2F)
    #         ZLSB = self.readByte(0x2E)
    #         rawZ = (ZMSB<<8) + ZLSB

    #     return [self.twosComp(rawX, 16)/100, 
    #             self.twosComp(rawY, 16)/100, 
    #             self.twosComp(rawZ, 16)/100]
    
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

    def getGravityRaw(self):
        if(not self.setup):
            print("ERROR IMU has not been initialized")
            return [-1, -1, -1]
        rawZ =  255

        XMSB = self.readByte(0x33)
        XLSB = self.readByte(0x32)
        rawX = (XMSB<<8) + XLSB

        YMSB = self.readByte(0x31)
        YLSB = self.readByte(0x30)
        rawY = (YMSB<<8) + YLSB

        while(abs(rawZ - 255) < 5 or abs(rawZ - 65280) < 5):
            ZMSB = self.readByte(0x2F)
            ZLSB = self.readByte(0x2E)
            rawZ = (ZMSB<<8) + ZLSB

        return [self.twosComp(rawX, 16)/100, 
                self.twosComp(rawY, 16)/100, 
                self.twosComp(rawZ, 16)/100,
                rawX, 
                rawY, 
                rawZ]
        

    #Accelerometer output
    def getAccelerometer(self):
        if(not self.setup):
            print("ERROR IMU has not been initialized")
            return [-1, -1, -1]

        XMSB = self.readByte(0x09)
        XLSB = self.readByte(0x08)
        YMSB = self.readByte(0x0B)
        YLSB = self.readByte(0x0A)
        ZMSB = self.readByte(0x0D)
        ZLSB = self.readByte(0x0C)

        return [self.twosComp((XMSB<<8) + XLSB, 16)/100,
                self.twosComp((YMSB<<8) + YLSB, 16)/100,
                self.twosComp((ZMSB<<8) + ZLSB, 16)/100]

    # requires a sensor fusion mode 
    def getWogma(self):
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

        return -vec[2]

    # def getWogmaPole(self):
    def getAngle(self):
        adjust = -0.017#rads

        vec = self.getGravityVector()
        print(vec)

        nega = 0
        if(vec[2] < 0):
            nega = 1
        else:
            nega = -1
        return (math.acos(vec[1]/math.sqrt(math.pow(vec[1], 2) + math.pow(vec[2], 2))))*nega + adjust
    
    def getAngleCompRads(self):
        #compensation for gyro mount
        adjust = 0 #degrees

        vec = self.getGravityVector()

        nega = 0
        if(vec[2] < 0):
            nega = 1
        else:
            nega = -1
        return (math.degrees(math.acos(vec[1]/math.sqrt(math.pow(vec[1], 2) + math.pow(vec[2], 2))))*nega + adjust)/180 * math.pi

    
