import TrinamicDriver as t
import IMU
import threading
import queue
import sys
import time

#MOSI MISO SCK 4671 6100
driverLEFT = t.trinamicDriver(13, 12, 11, 16, 7,'TrinamicConfig/parsedConfigL.txt')
driverRIGHT = t.trinamicDriver(36, 38, 37, 40, 35,'TrinamicConfig/parsedConfigR.txt')
driverRIGHT.stopMotor()
driverLEFT.stopMotor()


IMU = IMU.IMU(0, 40)
IMU.setupIMU()

trim = 0

kP = 0
kI = 0
kD = 0
accumError = 0
prevError = 0

q = queue.Queue()

#assumes that positive angle is forward leaning
def handlePID(target, actual, deltaTime):
    global kP, kI, kD, accumError, prevError

    #if the robot is leaning forward 20 degrees and wants to go to 0
    #this will generate a positive number
    error = actual-target

    #note: take for instance the robot is at 20 degrees and wants to go to zero this will be negative
    #(leaning forward generates a negative slope and derivative)
    derivative = (error-prevError)/deltaTime
    accumError += error*deltaTime

    return (kP * error) + (kD * derivative) + (kI * accumError)


def inputTask():
    while True:
        desiredAngle = input("enter angle:")
        q.put(desiredAngle)

def main():
    threading.Thread(target=inputTask, daemon=True).start()

    targAng = 0
    lastTime = time.time()

    while(True):
        if(not q.empty()):
            targAng = q.get()

        currAng = IMU.getAngle()

        output = handlePID(targAng, currAng, time.time()-lastTime)
        lastTime = time.time()
        driverLEFT.rotateMotorOpenloop(output)
        driverRIGHT.rotateMotorOpenloop(output)







print("Motor Testing Wizard")
print("Motor Selection (L)eft/(R)ight/(B)oth")
print("Mode Selection (O)penloop/(T)orque/(V)elocity/(P)osition")
print("Value")
print("Example: BV10 rotates both motors at 10rpm, LT500 rotates left motor at 500Nm")
print("Stop side with L or R")
print("E-Stop with Enter")

while True:
    instr = input("Enter command: ")
    inlen = len(instr)
    if inlen == 0:
        driverLEFT.stopMotor()
        driverRIGHT.stopMotor()
    elif inlen == 1:
        if instr == 'L':
            driverLEFT.stopMotor()
        elif instr == 'R':
            driverRIGHT.stopMotor()
    elif inlen > 2 and instr[2:].isdigit():
        if instr[0] == 'L' or instr[0] == 'B':
            if instr[1] == 'O':
                driverLEFT.rotateMotorOpenloop()
                print("driverLEFT.rotateMotorOpenloop()")
            elif instr[1] == 'T':
                driverLEFT.rotateMotorTorque(int(instr[2:]))
                print("driverLEFT.rotateMotorTorque(",int(instr[2:]),")")
            elif instr[1] == 'V':
                driverLEFT.rotateMotorVelocity(int(instr[2:]))
                print("driverLEFT.rotateMotorVelocity(",int(instr[2:]),")")
            elif instr[1] == 'P':
                driverLEFT.rotateMotorPosition(int(instr[2:]))
                print("driverLEFT.rotateMotorPosition(",int(instr[2:]),")")
            else:
                print("Illegal Mode")
        elif instr[0] != 'R':
            print("Illegal Side")
        if instr[0] == 'R' or instr[0] == 'B':
            if instr[1] == 'O':
                driverRIGHT.rotateMotorOpenloop()
                print("driverRIGHT.rotateMotorOpenloop()")
            elif instr[1] == 'T':
                driverRIGHT.rotateMotorTorque(int(instr[2:]))
                print("driverRIGHT.rotateMotorTorque(",int(instr[2:]),")")
            elif instr[1] == 'V':
                driverRIGHT.rotateMotorVelocity(int(instr[2:]))
                print("driverRIGHT.rotateMotorVelocity(",int(instr[2:]),")")
            elif instr[1] == 'P':
                driverRIGHT.rotateMotorPosition(int(instr[2:]))
                print("driverRIGHT.rotateMotorPosition(",int(instr[2:]),")")
            else:
                print("Illegal Mode")
        elif instr[0] != 'L':
            print("Illegal Side")
    else:
        print("Illegal Command")

driverRIGHT.stopMotor()
driverLEFT.stopMotor()
print("Thank you for using the BIG CRETE MOTOR UTILITY, Im sure you'll be back!")
