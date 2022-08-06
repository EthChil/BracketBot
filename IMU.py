import CommunicationHandler as comm
import time

log = comm.logger()

IMU = comm.I2CObject(0,40,log)

print(IMU.readByte(0))

setup = 0

def twosComp(val, bits):
    if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)        # compute negative value
    return val                         # return positive value as is

def setupIMU():
    global setup

    #mode information
    #NDOF mode - sensor fusion mode which allows us to access the gravity vector
    time.sleep(1)
    #write RST_SYS in SYS_TRIGGER
    IMU.writeByte(0x3F, 0x20)
    time.sleep(1)
    print("0x3f 20")
    print(IMU.readByte(0x3F))

    time.sleep(0.75)

    #write power mode in PWR_MODE
    IMU.writeByte(0x3E, 0x00)
    time.sleep(1)
    print("0x3e 00")
    print(IMU.readByte(0x3E))

    #write RST_SYS in SYS_TRIGGER
    IMU.writeByte(0x3F, 0x00)
    time.sleep(1)
    print("0x3f 00")
    print(IMU.readByte(0x3F))

    time.sleep(0.03)

    #Set operation mode in OPR_MODE (this should be NDOF)
    IMU.writeByte(0x3D, 0x0C)
    time.sleep(1)
    print("0x3d 0c")
    print(IMU.readByte(0x3D))

    setup = 1


#Gravity vector is a 3D vector with 100 LSB equal to 1m/s^2 (9.8m/s^2 would be 980 or 3D4)
#
def getGravity():
    global setup
    if(not setup):
        print("ERROR IMU has not been initialized")
        return [-1, -1, -1]

    XMSB = IMU.readByte(0x33)
    XLSB = IMU.readByte(0x32)
    YMSB = IMU.readByte(0x31)
    YLSB = IMU.readByte(0x30)
    ZMSB = IMU.readByte(0x2F)
    ZLSB = IMU.readByte(0x2E)

    
    return [twosComp((XMSB<<8) + XLSB, 16)/100,
            twosComp((YMSB<<8) + YLSB, 16)/100,
            twosComp((ZMSB<<8) + ZLSB, 16)/100]


setupIMU()
print(getGravity())
    
