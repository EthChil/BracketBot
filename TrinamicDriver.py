#import CommunicationHandler as comm
import BigCreteSpi as theCrete
import time
import os

#MOSI = 32
#MISO = 36
#SCK = 38
#CS6100 = 40
#CS4671 = 35

def twosComp(val, bits):
    if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)        # compute negative value
    return val                         # return positive value as is


class trinamicDriver():
    def __init__(self, mosi, miso, sck, cs4671, cs6100, setupFile):
        self.spi4671 = theCrete.BIG_CRETE_SPI(mosi, miso, sck, cs4671)

        self.spi6100 = theCrete.BIG_CRETE_SPI(mosi, miso, sck, cs6100)

        self.initTMC()
        self.initTMCfromFile(setupFile)

    # Initial Setup
    def initTMCfromFile(self, filename):
        setupFile = open(filename, 'r')

        for line in setupFile:
            if("wait" in line):
                time.sleep(1)
                continue

            data = line.split(',')
            intArr = [int(numeric_string) for numeric_string in data]
            print(intArr)
            self.spi4671.writeByte(intArr[0], intArr[1:])


    def initTMC(self):
        #setting GSTAT on 6100 to 0xFFFFFFFF to clear flags
        self.spi6100.writeByte(0x01, [255,255,255,255])

        #setting GCONF on 6100 0x00000000
        self.spi6100.writeByte(0x00, [00,00,00,00])

        #set drv strength on 6100 0
        data = self.spi6100.readByte(0x0A)
        data[0] = 00
        data[1] = 00
        self.spi6100.writeByte(0x0A, data)

        return True


    def setupEncoder(self):
        self.spi4671.writeByte(0x52, [0,0,0,2])   #TMC4671_PHI_E_SELECTION
        self.spi4671.writeByte(0x24, [0,0,9,201]) #TMC4671_UQ_UD_EXT
        self.spi4671.writeByte(0x63, [0,0,0,8])   #TMC4671_MODE_RAMP_MODE_MOTION
        self.spi4671.writeByte(0x20, [0,0,0,60])  #TMC4671_OPENLOOP_ACCELERATION
        self.spi4671.writeByte(0x21, [0,0,0,20])  #TMC4671_OPENLOOP_VELOCITY_TARGET
        time.sleep(5)
        self.spi4671.writeByte(0x21, [0,0,0,0])   #TMC4671_OPENLOOP_VELOCITY_TARGET
        self.spi4671.writeByte(0x24, [0,0,0,0])   #TMC4671_UQ_UD_EXT
        self.spi4671.writeByte(0x52, [0,0,0,3])   #TMC4671_PHI_E_SELECTION


    def rotateMotorOpenloop(self):
        self.spi4671.writeByte(0x24, [0,0,9,201])
        self.spi4671.writeByte(0x20, [0,0,0,60])
        self.spi4671.writeByte(0x63, [0,0,0,8])
        result = list(velocityTarget.to_bytes(4, 'big', signed=True))

    def rotateMotorTorque(self, torqueTarget):
        # Switch to torque mode in motion mode
        self.spi4671.writeByte(0x63, [0, 0, 0, 1])

        result = list(torqueTarget.to_bytes(2, 'big', signed=True))

        #Rotate motor1
        self.spi4671.writeByte(0x64, result + [0, 0])
    
    def rotateMotorVelocity(self, velocityTarget):
        self.spi4671.writeByte(0x63, [0,0,0,2])

        result = list(velocityTarget.to_bytes(4, 'big', signed=True))

        self.spi4671.writeByte(0x66, result)
    
    def rotateMotorPosition(self, positionTarget):
        self.spi4671.writeByte(0x63, [0,0,0,3])

        result = list(positionTarget.to_bytes(4, 'big', signed=True))

        self.spi4671.writeBytes(0x68, result)
        
    def stopMotor(self):
        #Set torque target to 0
        self.spi4671.writeByte(0x63, [0,0,0,1])
        self.spi4671.writeByte(0x21, [0,0,0,0])
        self.spi4671.writeByte(0x24, [0,0,0,0])
        self.spi4671.writeByte(0x68, [0,0,0,0])
        self.spi4671.writeByte(0x66, [0,0,0,0])
        self.spi4671.writeByte(0x64, [0,0,0,0])
        
    def hardStop(self):
        self.spi4671.writeByte(0x21, [0,0,0,0])
        self.spi4671.writeByte(0x24, [0,0,0,0])
        self.spi4671.writeByte(0x68, [0,0,0,0])
        self.spi4671.writeByte(0x66, [0,0,0,0])
        self.spi4671.writeByte(0x64, [0,0,0,0])
        self.spi4671.writeByte(0x63, [0,0,0,0])


    def getVelocity(self):
        self.spi4671.readByte(0x6A)





