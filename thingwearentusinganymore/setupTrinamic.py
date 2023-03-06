#import CommunicationHandler as comm
import BigCreteSpi as theCrete
#log = comm.logger()

# mosi miso sck cs
left_spi4671 = theCrete.BIG_CRETE_SPI(13, 12, 11, 16)
left_spi6100 = theCrete.BIG_CRETE_SPI(13, 12, 11, 7)
right_spi4671 = theCrete.BIG_CRETE_SPI(36, 38, 37, 40)
right_spi6100 = theCrete.BIG_CRETE_SPI(36, 38, 37, 35)

print("setting up Left side")
print("setting GSTAT on 6100 to 0xFFFFFFFF to clear flags")
left_spi6100.writeByte(0x01, [255,255,255,255])

print("setting GCONF on 6100 to 0x00000000")
left_spi6100.writeByte(0x00, [00,00,00,00])

print("setting DRV_STRENGTH on 6100 to 0")
data = left_spi6100.readByte(0x0A)
print("setting DRV_STRENGTH on 6100 to 0")
data = left_spi6100.readByte(0x0A)
print(data)
data[0] = 00
data[1] = 00
left_spi6100.writeByte(0x0A, data)

print("setting BBM HS and LS to 400ns")
left_spi6100.writeByte(0x19, [00,00,40,40])

print("setup complete")


print("setting up Right side")
print("setting GSTAT on 6100 to 0xFFFFFFFF to clear flags")
right_spi6100.writeByte(0x01, [255,255,255,255])

print("setting GCONF on 6100 to 0x00000000")
right_spi6100.writeByte(0x00, [00,00,00,00])

print("setting DRV_STRENGTH on 6100 to 0")
data = right_spi6100.readByte(0x0A)
print("setting DRV_STRENGTH on 6100 to 0")
data = right_spi6100.readByte(0x0A)
print(data)
data[0] = 00
data[1] = 00
right_spi6100.writeByte(0x0A, data)

print("setting BBM HS and LS to 400ns")
right_spi6100.writeByte(0x19, [00,00,40,40])

print("setup complete")
