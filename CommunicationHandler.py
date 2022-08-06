import math

from smbus2 import SMBus
import datetime

maxI2CBusses = 2

#Notes on jetson hardware
#| PIN | Function |
#|-----|----------|
#|  1  | 3.3V     |
#|  2  | 5V       |
#|  3  | i2c2     |
#|  4  | 5V       |
#|  5  | i2c2     |
#|  6  | GND      |
#|  7  | unused   |
#|  8  | uartb    |
#|  9  | GND      |
#|  10 | uartb    |
#|  11 | unused   |
#|  12 | unused   |
#|  13 | spi2_sck |
#|  14 | GND      |
#|  15 | NA       |
#|  16 | spi2_cs1 |
#|  17 | 3.3V     |
#|  18 | spi2_cs0 |
#|  19 | spi1_dout|
#|  20 | GND      |
#|  21 | spi1_din |
#|  22 | spi2_din |
#|  23 | spi1_sck |
#|  24 | spi1_cs0 |
#|  25 | GND      |
#|  26 | spi1_cs1 |
#|  27 | i2c1     |
#|  28 |         |
#|  29 |         |
#|  30 |         |
#|  31 |         |
#|  32 |         |
#|  33 |         |
#|  34 |         |
#|  35 |         |
#|  36 |         |
#|  37 |         |
#|  38 |         |
#|  39 |         |
#|  40 |         |

#i2c1
#27
#28

#i2c2
#3
#5

#spi1
#19 - dout
#21 - din
#23 - sck
#24 - cs0
#26 - cs1

#spi2
#16 - cs1
#18 - cs0
#22 - din
#37 - dout
#13 - sck



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
    def __init__(self, bus, chipAdd, logger, maxAddress=106, maxData=255):
        self.log = logger
        self.maxAddr = maxAddress
        self.maxData = maxData
        self.chipAdd = chipAdd

        if(bus > maxI2CBusses or bus < 0):
            print("Error attempting to access I2C bus out of range")

        #open SMBus object
        try:
            self.bus = SMBus(bus)
            #self.bus.pec = 1 #enable packet error checking
        except:
            self.log.pr("Failed to open I2C SMBus on port " + str(bus))


    def readByte(self, offset):
        if(offset > self.maxAddr):
            self.log.pr("MAX ADDRESS EXCEEDED")
            return -1

        try:
            b = self.bus.read_byte_data(self.chipAdd, offset)
        except:
            self.log.pr("Failed to read byte from address " + str(offset))
            return -1
        
        return b

    def writeByte(self, offset, data):
        if(offset > self.maxAddr):
            self.log.pr("MAX ADDRESS EXCEEDED")
            return False

        if(data > self.maxData):
            self.log.pr("MAX DATA LENGTH EXCEEDED")
            return False

        try:
            self.bus.write_byte_data(self.chipAdd, offset, data)
            return True
        except:
            self.log.pr("Failed to write byte " + str(data) + " to address " + str(offset))
            return False

class SPIObject():
    def __init__(self, bus, device, logger, maxSpeed=500000, mode=0b11, wordSize=32):
        self.bus = spidev.SpiDev()
        self.log = logger

        try:
            self.bus.open(bus, device)

            self.bus.max_speed_hz = maxSpeed
            self.bus.mode = mode
            self.word = wordSize
        except:
            self.log.pr("Failed to open spi dev" + str(bus) + "." + str(device))

    def readByte(self, address):
        if(address > 127):
            self.log.pr("ERROR: Invalid read address for SPI " + str(address))
            return False

        try:
            #print([address,00,00,00,00])
            readData = self.bus.xfer2([address,00,00,00,00])
        except:
            self.log.pr("ERROR: Failed to read from SPI " + str(address))
            return False

        print(readData)
        return readData


    def writeByte(self, address, data):
        if (address > 127):
            self.log.pr("ERROR: Invalid write address for SPI " + str(address))
            return False

        # if (data > (math.pow(2, self.word)-1)):
        #     self.log.pr("ERROR: Invalid write data for SPI " + str(data))
        #     return False
        #print([0b10000000 + address] + data)
        try:
            self.bus.xfer2([0b10000000 + address] + data)
        except:
            self.log.pr("ERROR: Failed to read from SPI " + str(address))
            return False
        return True

    def close(self):
        self.bus.close()



