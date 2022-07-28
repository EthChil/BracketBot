import CommunicationHandler as comm

log = comm.logger()

IMU = comm.I2CObject(0,log)

print(IMU.readByte(40))
