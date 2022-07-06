from smbus2 imoprt SMBus
import datetime
import spidev

maxI2CBusses = 2

class logger():
    def __init__(self):
        self.log = open("logs/" + datetime.date.today().strftime('%Y-%m-%d') + ".txt", "w")

    def pr(self, message):
        print(message)
        self.log.writeline(datetime.datetime.today().strftime("%H:%M:%S.%f") + message + "/n")
        self.log.flush()

    def close(self):
        self.log.flush()
        self.log.close()

class I2CObject():
    def __init__(self, bus, logger, maxAddress=106, maxData=255):
        self.log = logger
        self.maxAddr = maxAddress
        self.maxData = maxData

        if(bus > maxI2CBusses or bus < 0):
            print("Error attempting to access I2C bus out of range")

        #open SMBus object
        try:
            self.bus = SMBus(bus)
            bus.pec = 1 #enable packet error checking
        except:
            self.log.pr("Failed to open I2C SMBus on port " + str(bus))

    def readByte(self, address):
        if(address > self.maxAddr):
            self.log.pr("MAX ADDRESS EXCEEDED")
            return -1

        try:
            b = self.bus.read_byte_data(address, 0)
        except:
            self.log.pr("Failed to read byte from address " + address)
            return -1

        return b

    def writeByte(self, address, data):
        if(address > self.maxAddr):
            self.log.pr("MAX ADDRESS EXCEEDED")
            return False

        if(data > self.maxData):
            self.log.pr("MAX DATA LENGTH EXCEEDED")
            return False

        try:
            self.bus.write_byte_data(address, 0, data)
            return True
        except:
            self.log.pr("Failed to write byte " + data + " to address " + address)
            return False

class SPIObject():
    def __init__(self, bus, logger, ):