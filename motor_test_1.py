import CommunicationHandler as comm
import time

log = comm.logger()

spi4671 = comm.SPIObject(1,1,log)
spi6100 = comm.SPIObject(1,0,log)

# Initial Setup
spi6100.writeByte(0x01, [255,255,255,255])
spi6100.writeByte(0x00, [00,00,00,00])
data = spi6100.readByte(0x0A)
data = spi6100.readByte(0x0A)
data[0] = 00
data[1] = 00
spi6100.writeByte(0x0A, data)
spi6100.writeByte(0x19, [00,00,40,40])

# Run motor

spi4671.writeByte(0x1B, [0,3,0,15])
spi4671.writeByte(0x17, [0,0,0,0]) 
spi4671.writeByte(0x18, [0,0,15,159])
spi4671.writeByte(0x19, [0,0,40,40])
spi4671.writeByte(0x1A, [0,0,0,7])

# ADC configuration
spi4671.writeByte(0x0A, [24,0,1,0])
spi4671.writeByte(0x04, [0,16,0,16])
spi4671.writeByte(0x05, [32,0,0,0])
spi4671.writeByte(0x06, [32,0,0,0])
spi4671.writeByte(0x07, [1,78,1,78])
spi4671.writeByte(0x09, [1,0,129,80])
spi4671.writeByte(0x08, [1,0,129,165])

# ABN encoder settings
spi4671.writeByte(0x25, [0,0,16,0])
spi4671.writeByte(0x26, [0,0,32,0])
spi4671.writeByte(0x27, [0,0,13,210])
spi4671.writeByte(0x29, [0,0,0,0])

# Limits
spi4671.writeByte(0x5E, [0,0,3,232])

# PI settings
spi4671.writeByte(0x56, [1,0,1,0])
spi4671.writeByte(0x54, [1,0,1,0])

#  ===== ABN encoder test drive =====

# Init encoder (mode 0)
spi4671.writeByte(0x63, [0,0,0,8])
spi4671.writeByte(0x29, [0,0,0,0])
spi4671.writeByte(0x52, [0,0,0,1])
spi4671.writeByte(0x1C, [0,0,0,0])
spi4671.writeByte(0x24, [0,0,0,0])
time.sleep(1)
spi4671.writeByte(0x27, [0,0,0,0])
# Feedback selection
spi4671.writeByte(0x52, [0,0,0,3])
spi4671.writeByte(0x50, [0,0,0,9])

# Switch to torque mode
spi4671.writeByte(0x63, [0,0,0,1])

# Rotate right
spi4671.writeByte(0x64, [3,232,0,0])
time.sleep(3)

# Rotate left
spi4671.writeByte(0x64, [252,24,0,0])
time.sleep(3)

# Stop
spi4671.writeByte(0x64, [0,0,0,0])
