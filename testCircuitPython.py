#import CommunicationHandler as comm
import busio
import digitalio
import board
from adafruit_bus_device.spi_device import SPIDevice

#log = comm.logger()

#bus = input("enter spibus number (0 - LEFT) or (1 - RIGHT): ")
device = input("enter device number (1 - 4671) or (0 - 6100): ")

comm_port = busio.SPI(board.SCK_1, board.MOSI_1, board.MISO_1)
Left4671cs = digitalio.DigitalInOut(board.D7)
Left6100cs = digitalio.DigitalInOut(board.D11)

Left4671device = SPIDevice(comm_port, Left4671cs, polarity=1, phase=1)
Left6100device = SPIDevice(comm_port, Left6100cs, polarity=1, phase=1)

if(device):
    activeDevice = Left4671device
    print("4671 on Left controller")
else:
    activeDevice = Left6100device
    print("6100 on Left controller")

while(True):        
    addr = int(input("Address to read/write in hex EX. 0x0A: "), 16)
    opp = input("Read (R) or Write (W): ")
    outbuf = addr.to_bytes(5, byteorder='little') 
    inbuf = bytearray(5)
    testout = bytearray([129,0,0,0,0])
    print(testout)
    if(opp == 'R' or opp == 'r'):
        with activeDevice as bus_device:
            bus_device.write_readinto(testout,inbuf)
            print(inbuf)
