#import CommunicationHandler as comm
import BigCreteSpi as theCrete
import time
import os
import refridge as r

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
    def __init__(self, mosi, miso, sck, cs4671, cs6100, side): #changed "setupFile" to "side"
        self.spi4671 = theCrete.BIG_CRETE_SPI(mosi, miso, sck, cs4671)
        self.spi6100 = theCrete.BIG_CRETE_SPI(mosi, miso, sck, cs6100)
        self.side = side
        self.initBOB()
        # self.initTMCfromFile(setupFile)
        self.initBOB()

    # Initial Setup
    # def initTMCfromFile(self, filename):
    #     setupFile = open(filename, 'r')

    #     for line in setupFile:
    #         if("wait" in line):
    #             time.sleep(1)
    #             continue

    #         data = line.split(',')
    #         intArr = [int(numeric_string) for numeric_string in data]
    #         print(intArr)
    #         self.spi4671.writeByte(intArr[0], intArr[1:])

    def initBOB(self):
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

    def init4671(self):
        self.spi4671.writeByte(r.regs_4671["MOTOR_TYPE_N_POLE_PAIRS"],          [0x00,0x03,0x00,0x0F])
        self.spi4671.writeByte(r.regs_4671["PWM_POLARITIES"],                   [0x00,0x00,0x00,0x00])
        self.spi4671.writeByte(r.regs_4671["PWM_MAXCNT"],                       [0x00,0x00,0x0F,0x9F])
        self.spi4671.writeByte(r.regs_4671["PWM_BBM_H_BBM_L"],                  [0x00,0x00,0x28,0x28])
        self.spi4671.writeByte(r.regs_4671["PWM_SV_CHOP"],                      [0x00,0x00,0x00,0x07])

        #ADC configuration
        self.spi4671.writeByte(r.regs_4671["ADC_I_SELECT"],                     [0x18,0x00,0x01,0x00])
        self.spi4671.writeByte(r.regs_4671["dsADC_MCFG_B_MCFG_A"],              [0x00,0x10,0x00,0x10])
        self.spi4671.writeByte(r.regs_4671["dsADC_MCLK_A"],                     [0x20,0x00,0x00,0x00])
        self.spi4671.writeByte(r.regs_4671["dsADC_MCLK_B"],                     [0x20,0x00,0x00,0x00])
        self.spi4671.writeByte(r.regs_4671["dsADC_MDEC_B_MDEC_A"],              [0x01,0x4E,0x01,0x4E])
        if self.side == "L":
            self.spi4671.writeByte(r.regs_4671["ADC_I0_SCALE_OFFSET"],          [0x01,0x00,0x81,0x3F])
            self.spi4671.writeByte(r.regs_4671["ADC_I1_SCALE_OFFSET"],          [0x01,0x00,0x81,0xA8])
        elif self.side == "R":
            self.spi4671.writeByte(r.regs_4671["ADC_I0_SCALE_OFFSET"],          [0x01,0x00,0x81,0x4B])
            self.spi4671.writeByte(r.regs_4671["ADC_I1_SCALE_OFFSET"],          [0x01,0x00,0x82,0x11])
        
        #ABN Encoder settings
        if self.side == "L":
            self.spi4671.writeByte(r.regs_4671["ABN_DECODER_MODE"],             [0x00,0x00,0x10,0x00])
            self.spi4671.writeByte(r.regs_4671["ABN_DECODER_COUNT"],            [0x00,0x00,0x18,0x94]) #Unsure about this mattering
        elif self.side == "R":
            self.spi4671.writeByte(r.regs_4671["ABN_DECODER_MODE"],             [0x00,0x00,0x00,0x00])
            self.spi4671.writeByte(r.regs_4671["ABN_DECODER_COUNT"],            [0x00,0x00,0x02,0x36]) #Unsure about this mattering
        self.spi4671.writeByte(r.regs_4671["ABN_DECODER_PPR"],                  [0x00,0x00,0x20,0x00])
        self.spi4671.writeByte(r.regs_4671["ABN_DECODER_PHI_E_PHI_M_OFFSET"],   [0x00,0x00,0x00,0x00])

        # Limits
        self.spi4671.writeByte(r.regs_4671["PID_TORQUE_FLUX_LIMITS"],           [0x00,0x00,0x03,0xE8])
        self.spi4671.writeByte(r.regs_4671["PID_VELOCITY_LIMIT"],               [0x00,0x00,0x00,0xC8])

        # PI settings *** TUNES ARE CURRENTLY SYMMETRICAL ***
        if self.side == "L":
            self.spi4671.writeByte(r.regs_4671["PID_TORQUE_P_TORQUE_I"],         [0x03,0x20,0x13,0x88]) #P=800   I=5000
            self.spi4671.writeByte(r.regs_4671["PID_FLUX_P_FLUX_I"],             [0x01,0x90,0x09,0xC4]) #P=400   I=2500
            self.spi4671.writeByte(r.regs_4671["PID_VELOCITY_P_VELOCITY_I"],     [0x4E,0x20,0x03,0x20]) #P=20000 I=800
            self.spi4671.writeByte(r.regs_4671["PID_POSITION_P_POSITION_I"],     [0x00,0x16,0x00,0x00]) #P=22    I=0
        elif self.side == "R":
            self.spi4671.writeByte(r.regs_4671["PID_TORQUE_P_TORQUE_I"],         [0x03,0x20,0x13,0x88]) #P=800   I=5000
            self.spi4671.writeByte(r.regs_4671["PID_FLUX_P_FLUX_I"],             [0x01,0x90,0x09,0xC4]) #P=400   I=2500
            self.spi4671.writeByte(r.regs_4671["PID_VELOCITY_P_VELOCITY_I"],     [0x4E,0x20,0x03,0x20]) #P=20000 I=800
            self.spi4671.writeByte(r.regs_4671["PID_POSITION_P_POSITION_I"],     [0x00,0x16,0x00,0x00]) #P=22    I=0

        #Feedback selection
        self.spi4671.writeByte(r.regs_4671["PHI_E_SELECTION"],                  [0x00,0x00,0x00,0x03])
        self.spi4671.writeByte(r.regs_4671["VELOCITY_SELECTION"],               [0x00,0x00,0x00,0x09])

    def setupEncoder(self):
        print("Initializing Encoder...")
        self.spi4671.writeByte(r.regs_4671["MODE_RAMP_MODE_MOTION"],            [0x00,0x00,0x00,0x08])
        self.spi4671.writeByte(r.regs_4671["ABN_DECODER_PHI_E_PHI_M_OFFSET"],   [0x00,0x00,0x00,0x00])
        self.spi4671.writeByte(r.regs_4671["PHI_E_SELECTION"],                  [0x00,0x00,0x00,0x01])
        self.spi4671.writeByte(r.regs_4671["PHI_E_EXT"],                        [0x00,0x00,0x00,0x00])
        self.spi4671.writeByte(r.regs_4671["UQ_UD_EXT"],                        [0x00,0x00,0x0F,0xA0])
        time.sleep(1)
        self.spi4671.writeByte(r.regs_4671["ABN_DECODER_COUNT"],                [0x00,0x00,0x00,0x00])
        print("Encoder Initialized")


    def rotateMotorOpenloop(self, velocityTarget):
        self.spi4671.writeByte(r.regs_4671["PHI_E_SELECTION"], [0,0,0,2]) #phi_e_openloop
        self.spi4671.writeByte(0x1F, [0,0,0,0]) #TMC4671_OPENLOOP_PHI_DIRECTION
        self.spi4671.writeByte(0x24, [0,0,15,160])
        self.spi4671.writeByte(0x20, [0,0,0,60])
        self.spi4671.writeByte(r.regs_4671["MODE_RAMP_MODE_MOTION"], [0,0,0,8]) #UQ_UD_EXT
        result = list(velocityTarget.to_bytes(4, 'big', signed=True))
        self.spi4671.writeByte(0x21, result)


    def rotateMotorTorque(self, torqueTarget):
        # Switch to torque mode in motion mode
        
        self.spi4671.writeByte(r.regs_4671["PHI_E_SELECTION"], [0,0,0,3]) #phi_e_abn
        self.spi4671.writeByte(r.regs_4671["MODE_RAMP_MODE_MOTION"], [0, 0, 0, 1]) #torque_mode

        result = list(torqueTarget.to_bytes(2, 'big', signed=True))

        self.spi4671.writeByte(0x64, result + [0, 0])
    
    def rotateMotorVelocity(self, velocityTarget):
        self.spi4671.writeByte(r.regs_4671["PHI_E_SELECTION"], [0,0,0,3]) #phi_e_abn
        self.spi4671.writeByte(r.regs_4671["MODE_RAMP_MODE_MOTION"], [0,0,0,2]) #velocity_mode

        result = list(velocityTarget.to_bytes(4, 'big', signed=True))

        self.spi4671.writeByte(0x66, result)
    
    def rotateMotorPosition(self, positionTarget):
        self.spi4671.writeByte(r.regs_4671["PHI_E_SELECTION"], [0,0,0,3]) #phi_e_abn
        self.spi4671.writeByte(r.regs_4671["MODE_RAMP_MODE_MOTION"], [0,0,0,3]) #position_mode

        result = list(positionTarget.to_bytes(4, 'big', signed=True))

        self.spi4671.writeBytes(0x68, result)
        
    def stopMotor(self):
        #Set torque target to 0
        self.spi4671.writeByte(r.regs_4671["MODE_RAMP_MODE_MOTION"], [0,0,0,1]) #torque_mode for soft stopping
        self.spi4671.writeByte(0x21, [0,0,0,0])
        self.spi4671.writeByte(0x24, [0,0,0,0])
        self.spi4671.writeByte(0x68, [0,0,0,0])
        self.spi4671.writeByte(0x66, [0,0,0,0])
        self.spi4671.writeByte(0x64, [0,0,0,0])
        self.spi4671.writeByte(0x52, [0,0,0,0])
        
    def hardStop(self):
        self.spi4671.writeByte(0x21, [0,0,0,0])
        self.spi4671.writeByte(0x24, [0,0,0,0])
        self.spi4671.writeByte(0x68, [0,0,0,0])
        self.spi4671.writeByte(0x66, [0,0,0,0])
        self.spi4671.writeByte(0x64, [0,0,0,0])
        self.spi4671.writeByte(0x63, [0,0,0,0])


    def getVelocity(self):
        intVelocity = int.from_bytes(bytearray(self.spi4671.readByte(0x6A)), 'big', signed=True)
        return intVelocity



