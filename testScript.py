import CommunicationHandler as comm

log = comm.logger()

spi = comm.SPIObject(0,0,log)

print(spi.readByte(0b0000000))

