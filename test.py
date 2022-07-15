import CommunicationHandler as comm

log = comm.logger()

spi = comm.SPIObject(0,0,log,mode=0b00)
print(spi.readByte(0b0000000))

spi = comm.SPIObject(0,1,log,mode=0b00)
print(spi.readByte(0b0000000))

spi = comm.SPIObject(1,0,log,mode=0b00)
print(spi.readByte(0b0000000))

spi = comm.SPIObject(1,1,log,mode=0b00)
print(spi.readByte(0b0000000))

spi = comm.SPIObject(0,0,log,mode=0b11)
print(spi.readByte(0b0000000))

spi = comm.SPIObject(0,1,log,mode=0b11)
print(spi.readByte(0b0000000))

spi = comm.SPIObject(1,0,log,mode=0b11)
print(spi.readByte(0b0000000))

spi = comm.SPIObject(1,1,log,mode=0b11)
print(spi.readByte(0b0000000))

