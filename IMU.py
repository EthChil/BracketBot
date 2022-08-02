import CommunicationHandler as comm
import time

log = comm.logger()

IMU = comm.I2CObject(0,40,log)

print(IMU.readByte(40))

setup = 0

def twosComp(val, bits):
    if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)        # compute negative value
    return val                         # return positive value as is

def setupIMU():
    global setup

    #mode information
    #NDOF mode - sensor fusion mode which allows us to access the gravity vector

    #write RST_SYS in SYS_TRIGGER
    IMU.writeByte(int("3F", 16), int("20", 16))

    time.sleep(0.03)

    #write power mode in PWR_MODE
    IMU.writeByte(int("3E", 16), 0)


    #write RST_SYS in SYS_TRIGGER
    IMU.writeByte(int("3F", 16), int("00", 16))

    time.sleep(0.03)

    #Set operation mode in OPR_MODE (this should be NDOF)
    IMU.writeByte(int("3D", 16), int("0C", 16))

    setup = 1


#Gravity vector is a 3D vector with 100 LSB equal to 1m/s^2 (9.8m/s^2 would be 980 or 3D4)
#
def getGravity():
    global setup
    if(not setup):
        print("ERROR IMU has not been initialized")
        return [-1, -1, -1]

    XMSB = IMU.readByte(int("33", 16))
    XLSB = IMU.readByte(int("32", 16))
    YMSB = IMU.readByte(int("31", 16))
    YLSB = IMU.readByte(int("30", 16))
    ZMSB = IMU.readByte(int("2F", 16))
    ZLSB = IMU.readByte(int("2E", 16))

    print(XMSB)
    print(XLSB)
    print(YMSB)
    print(YLSB)
    print(ZMSB)
    print(ZLSB)


setupIMU()
getGravity()
