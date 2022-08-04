import CommunicationHandler as comm

log = comm.logger()

bus = input("enter spibus number (0 - LEFT) or (1 - RIGHT):")
device = input("enter device number (1 - 4671) or (0 - 6100):")
spi = comm.SPIObject(int(bus),int(device),log,mode=0b11)


while(True):        
    add = input("address to read/write in hex EX. 0x0A:")
    opp = input("Read (R) or Write (W):")

    if(opp == 'R' or opp == 'r'):
        retDat = spi.readByte(int(add, 16))
        print("bit range, hex,  ASCII")
        ctr = 4
        for item in retDat:
            print((str((ctr*8) + 7) + "->" + str(ctr*8)).ljust(9) + "  " + hex(item) + "  " + chr(item))
            ctr -= 1
    else:
        data = input("data to write (31->0) EX. 12345678:")
        dataArr = []
        for i in range(4):
            #print(i)
            #print(data[i*2:(i*2)+2])
            dataArr.append(int(data[i*2:(i*2)+2], 16))
        print(dataArr)
        print(spi.writeByte(int(add, 16), dataArr))





#spi = comm.SPIObject(0,0,log,mode=0b00)
#print(spi.readByte(0b0000000))

#spi = comm.SPIObject(0,0,log,mode=0b00)
#print(spi.readByte(0b0000000))

#spi = comm.SPIObject(0,0,log,mode=0b00)
#print(spi.readByte(0b0000000))

#spi = comm.SPIObject(0,0,log,mode=0b11)
##print(spi.readByte(0b0000000))

#spi = comm.SPIObject(0,0,log,mode=0b11)
#print(spi.readByte(0b0000000))

#spi = comm.SPIObject(0,0,log,mode=0b11)
#print(spi.readByte(0b0000000))

#spi = comm.SPIObject(0,0,log,mode=0b11)
#print(spi.readByte(0b0000000))

