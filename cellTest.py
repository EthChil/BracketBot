import TrinamicDriver as t
import time

#MOSI MISO SCK 4671 6100
driverLEFT = t.trinamicDriver(13, 12, 11, 16, 7,'TrinamicConfig/parsedConfigL.txt')
driverRIGHT = t.trinamicDriver(36, 38, 37, 40, 35,'TrinamicConfig/parsedConfigR.txt')
driverRIGHT.stopMotor()
driverLEFT.stopMotor()
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
