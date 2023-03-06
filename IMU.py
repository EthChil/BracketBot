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

    def twosComp(self, val, bits):
        if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
            val = val - (1 << bits)        # compute negative value
        return val                         # return positive value as is

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
        time.sleep(1)
        print("0x3e 00")
        print(self.readByte(0x3E))

        #write RST_SYS in SYS_TRIGGER
        self.writeByte(0x3F, 0x00)
        time.sleep(1)
        print("0x3f 00")
        print(self.readByte(0x3F))

        time.sleep(0.03)

        #Set operation mode in OPR_MODE (this should be NDOF)
        self.writeByte(0x3D, 0x0C)
        time.sleep(1)
        print("0x3d 0c")
        print(self.readByte(0x3D))

        self.setup = 1


    #Gravity vector is a 3D vector with 100 LSB equal to 1m/s^2 (9.8m/s^2 would be 980 or 3D4)
    #
    def getGravityVector(self):
        if(not self.setup):
            print("ERROR IMU has not been initialized")
            return [-1, -1, -1]

        XMSB = self.readByte(0x33)
        XLSB = self.readByte(0x32)
        YMSB = self.readByte(0x31)
        YLSB = self.readByte(0x30)
        ZMSB = self.readByte(0x2F)
        ZLSB = self.readByte(0x2E)

        return [self.twosComp((XMSB<<8) + XLSB, 16)/100,
                self.twosComp((YMSB<<8) + YLSB, 16)/100,
                self.twosComp((ZMSB<<8) + ZLSB, 16)/100]

    def getAngle(self):
        vec = self.getGravityVector()

        nega = 0
        if(vec[2] < 0):
            nega = 1
        else:
            nega = -1
        return math.degrees(math.acos(vec[1]/math.sqrt(math.pow(vec[1], 2) + math.pow(vec[2], 2))))*nega


    def getAngleCompRads(self):
        #compensation for gyro mount
        adjust = -1.5 #degrees

        vec = self.getGravityVector()
        print(vec)

        nega = 0
        if(vec[2] < 0):
            nega = 1
        else:
            nega = -1
        return (math.degrees(math.acos(vec[1]/math.sqrt(math.pow(vec[1], 2) + math.pow(vec[2], 2))))*nega + adjust)/180 * math.pi

    
