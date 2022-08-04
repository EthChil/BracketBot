import CommunicationHandler as comm
import time
import os

def twosComp(val, bits):
    if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)        # compute negative value
    return val                         # return positive value as is


class trinamicDriver():
    def __init__(self, bus, device4671, device6100, setupFile, log):
        os.system("modprobe spidev")

        self.spi4671 = comm.SPIObject(bus, device4671, log)
        self.spi6100 = comm.SPIObject(bus, device6100, log)

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
        data = self.spi6100.readByte(0x0A)
        data[0] = 00
        data[1] = 00
        self.spi6100.writeByte(0x0A, data)

        #set bbm HS and LS to 400ns
        self.spi6100.writeByte(0x19, [00,00,40,40])

        return True

    def setupMotor(self):
        # set pole pairs and motor type
        self.spi4671.writeByte(0x1B, [0, 3, 0, 15])

        #PWM polarity for 6100
        self.spi4671.writeByte(0x17, [0, 0, 0, 0])

        #max pwm frequency
        self.spi4671.writeByte(0x18, [0,0,15,159])

        #Break before make to 400ns on LS and HS
        self.spi4671.writeByte(0x19, [0,0,40,40])

        #PWM chopper set to centered PWM for FOC
        self.spi4671.writeByte(0x1A, [0,0,0,7])

        # Feedback selection
        # set feedback to use encoder
        self.spi4671.writeByte(0x52, [0, 0, 0, 3])
        # set velocity mode to use encoder
        self.spi4671.writeByte(0x50, [0, 0, 0, 9])

    def setupADC(self):
        #Configure ADC I0
        self.spi4671.writeByte(0x0A, [24,0,1,0])

        #Configure ADC
        self.spi4671.writeByte(0x04, [0,16,0,16])
        self.spi4671.writeByte(0x05, [32,0,0,0])
        self.spi4671.writeByte(0x06, [32,0,0,0])
        self.spi4671.writeByte(0x07, [1,78,1,78])
        self.spi4671.writeByte(0x09, [1,0,129,80])
        self.spi4671.writeByte(0x08, [1,0,129,165])

    def setupEncoder(self):
        # ABN encoder settings
        #set ABN decoder mode (decode polarities and z tick)
        self.spi4671.writeByte(0x25, [0,0,16,0])

        #set the PPR of the encoder (8192)
        self.spi4671.writeByte(0x26, [0,0,32,0])

        #ABN decoder count
        self.spi4671.writeByte(0x27, [0,0,31,98])#1F62

        #Encoder PHI M offset 
        self.spi4671.writeByte(0x29, [0,0,0,0])

        # Init encoder (mode 0)
        # set to uq_ud_ext ramp mode motion mode
        self.spi4671.writeByte(0x63, [0, 0, 0, 8])
        # encoder PHI M offset
        self.spi4671.writeByte(0x29, [0, 0, 0, 0])
        # PHI E selection (phi e ext)
        self.spi4671.writeByte(0x52, [0, 0, 0, 1])
        # set phi e ext to 0
        self.spi4671.writeByte(0x1C, [0, 0, 0, 0])
        # set UQ UD ext to 0
        self.spi4671.writeByte(0x24, [0, 0, 0, 0])

        time.sleep(1)
        # set encoder count to 0
        self.spi4671.writeByte(0x27, [0, 0, 0, 0])

    def setupPIDConstants(self):
        #Torque PID Flux Limits
        self.spi4671.writeByte(0x5E, [0,0,3,232])

        #Set PI constant for Torque control
        self.spi4671.writeByte(0x56, [1,0,1,0])

        #Set PI constant for Flux control
        self.spi4671.writeByte(0x54, [1,0,1,0])

    #torque is in ??? (-_-)
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
        self.spi4671.writeByte(0x68, [0,0,0,0])
        self.spi4671.writeByte(0x66, [0,0,0,0])
        self.spi4671.writeByte(0x64, [0,0,0,0])
        
    def hardStop(self):
        self.spi4671.writeByte(0x68, [0,0,0,0])
        self.spi4671.writeByte(0x66, [0,0,0,0])
        self.spi4671.writeByte(0x64, [0,0,0,0])
        self.spi4671.writeByte(0x63, [0,0,0,0])








