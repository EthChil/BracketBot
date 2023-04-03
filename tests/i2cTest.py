import time
from smbus2 import SMBus
import math

maxI2CBusses = 2

class IMU:
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

a = IMU(0, 0x4A)
for i in range(0, 0x99):
    print(a.readByte(i))