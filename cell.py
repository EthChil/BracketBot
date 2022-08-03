import TrinamicDriver as t
import CommunicationHandler as comm
import time

c = comm.logger()
driver = t.trinamicDriver(1,1,0,'TrinamicConfig/parsedConfig.txt', c)

driver.rotateMotorOpenloop(750)
time.sleep(3)
driver.stopMotor()
time.sleep(3)
driver.rotateMotorVelocity(10)
time.sleep(3)
driver.stopMotor()
