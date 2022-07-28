import CommunicationHandler as comm

log = comm.logger()

#device = input("enter device number (1 - 4671) or (0 - 6100):")

spi4671 = comm.SPIObject(1,1,log)
spi6100 = comm.SPIObject(1,0,log)

print("setting GSTAT on 6100 to 0xFFFFFFFF to clear flags")
spi6100.writeByte(0x01, [255,255,255,255])

print("setting GCONF on 6100 to 0x00000000")
spi6100.writeByte(0x00, [00,00,00,00])

print("setting DRV_STRENGTH on 6100 to 0")
data = spi6100.readByte(0x0A)
print("setting DRV_STRENGTH on 6100 to 0")
data = spi6100.readByte(0x0A)
print(data)
data[0] = 00
data[1] = 00
spi6100.writeByte(0x0A, data)

print("setting BBM HS and LS to 400ns")
spi6100.writeByte(0x19, [00,00,40,40])

print("setup complete")
