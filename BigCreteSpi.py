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

            self.MOSI = mosi
            self.MISO = miso
            self.SCK = sck
            self.CS = cs

            GPIO.setup(self.MOSI, GPIO.OUT)
            GPIO.setup(self.MISO, GPIO.IN)
            GPIO.setup(self.SCK, GPIO.OUT, initial=GPIO.HIGH)
            GPIO.setup(self.CS, GPIO.OUT, initial=GPIO.HIGH)

    #DOING IT BIG
    def Harvi8XUltra(self, array):
        print(len(array))
        if(len(array) > 40):
            return 0
        else:
            output = [0,0,0,0,0]
            for index, value in enumerate(array):
                output[index // 8] += int(value * (2 ** (7 - (index % 8))))
            return output

    def clockRead(self):
        GPIO.output(self.SCK, GPIO.LOW)
        GPIO.output(self.SCK, GPIO.HIGH)
        dat = GPIO.input(self.MISO)
        return dat


    def clockWrite(self):
        GPIO.output(self.SCK, GPIO.LOW)
        GPIO.output(self.SCK, GPIO.HIGH)

    def readByte(self, addr):
        readData = []

        autism = time.time()
        GPIO.output(self.CS, GPIO.LOW)

        for bit in range(39,-1,-1):
            if((addr<<32) & 1 << bit != 0):
                GPIO.output(self.MOSI, GPIO.HIGH)
            else:
                GPIO.output(self.MOSI, GPIO.LOW)
            readData.append(self.clockRead())
        GPIO.output(self.CS, GPIO.HIGH)
        variable = time.time()
        
        print(variable - autism)

        return self.Harvi8XUltra(readData)[1:]

    def writeByte(self, addr, data):

        #convert address and data to a big int
        output = (addr << (8*4)) + (1 << 39)
        for i, byte in enumerate(data):
            output += byte << (8*(3-i))

        #start spi transaction
        GPIO.output(self.CS, GPIO.LOW)

        #write bits
        for bit in range(39,-1,-1):
            if(output & 1 << bit != 0):
                GPIO.output(self.MOSI, GPIO.HIGH)
            else:
                GPIO.output(self.MOSI, GPIO.LOW)
            self.clockWrite()

        #end transaction
        GPIO.output(self.CS, GPIO.HIGH)

    def close(self):
        GPIO.cleanup()



