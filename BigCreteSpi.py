import Jetson.GPIO as GPIO
import time
import math

#MOSI = 32
#MISO = 36
#SCK = 38
#CS4671 = 40
#CS6100 = 35

#DOING IT BIG, BIG CRETE STYLE
#BROUGHT TO YOU BY CANADIAN MIGHT BOOOOM
class BIG_CRETE_SPI():
    MOSI = 0
    MISO = 0
    SCK = 0
    CS = 0

    def __init__(self, mosi, miso, sck, cs):
            GPIO.setmode(GPIO.BOARD)

            GPIO.setup(self.MOSI, GPIO.OUT)
            GPIO.setup(self.MISO, GPIO.IN)
            GPIO.setup(self.SCK, GPIO.OUT)
            GPIO.setup(self.CS, GPIO.OUT, initial=GPIO.HIGH)

    #DOING IT BIG
    def Harvi8XUltra(self, array):
        if(len(array) % 8 != 0):
            return 0
        else:
            output = []
            for index, value in enumerate(array):
                output[index // 10] += value * (math.pow(2, (index % 8)))
            return output


    def clockRead(self):
        GPIO.output(self.SCK, GPIO.LOW)
        dat = GPIO.input(self.MISO)
        GPIO.output(self.SCK, GPIO.HIGH)
        return dat

    def clockWrite(self):
        GPIO.output(self.SCK, GPIO.LOW)
        GPIO.output(self.SCK, GPIO.HIGH)

    def readByte(self, addr):
        readData = []

        GPIO.output(self.CS, GPIO.LOW)

        for bit in range(39,0):
            if(addr & 1 << bit != 0):
                GPIO.output(self.MOSI, GPIO.HIGH)
            else:
                GPIO.output(self.MOSI, GPIO.LOW)
            readData.append(self.clockRead())

        GPIO.output(self.CS, GPIO.HIGH)

        return self.Harvi8XUltra(readData)

    def writeByte(self, addr, data):

        #convert address and data to a big int
        output = addr << (8*4)
        for i, byte in enumerate(data):
            output += byte << (8*(3-i))

        #start spi transaction
        GPIO.output(self.CS, GPIO.LOW)

        #write bits
        for bit in range(39,0):
            if(output & 1 << bit != 0):
                GPIO.output(self.MOSI, GPIO.HIGH)
            else:
                GPIO.output(self.MOSI, GPIO.LOW)
            self.clockWrite()

        #end transaction
        GPIO.output(self.CS, GPIO.HIGH)

    def close(self):
        GPIO.cleanup()



