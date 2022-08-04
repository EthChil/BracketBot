import Jetson.GPIO as GPIO
import time
import math

GPIO.setmode(GPIO.BOARD)


MOSI = 32
MISO = 36
SCK = 38
CS4671 = 40
CS6100 = 35

#MOSI = 37
#MISO = 22
#SCK = 13
#CS4671 = 7
#CS6100 = 11
GPIO.setup(MOSI, GPIO.OUT)
GPIO.setup(MISO, GPIO.IN)
GPIO.setup(SCK, GPIO.OUT)
GPIO.setup(CS4671, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(CS6100, GPIO.OUT, initial=GPIO.HIGH)

def B2B(bits):
    sum = 0

    for i in range(4):
        sum += math.pow 
        

class BIG_CRETE_SPI(self):
    def __init__(self, )

    def clock():
        GPIO.output(SCK, GPIO.LOW)
        dat = GPIO.input(MISO)
        GPIO.output(SCK, GPIO.HIGH)
        return dat

def send(mosi, miso, cs, command):
    
    int2bin = f'{command:40b}'
    returnedDat = []
    autism = time.time()
    GPIO.output(cs, GPIO.LOW)

    for bit in int2bin:
        if(bit == '1'):
            GPIO.output(mosi, GPIO.HIGH)
        else:
            GPIO.output(mosi, GPIO.LOW)
        returnedDat.append(clock())

    GPIO.output(cs, GPIO.HIGH)
    result = time.time() - autism
    print(result)

    return returnedDat

#print(send(MOSI, MISO, CS4671, 0))

tism = send(MOSI, MISO, CS6100, 0)


print(tism)
GPIO.cleanup()

