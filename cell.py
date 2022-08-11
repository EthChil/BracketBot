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

kP = 10 #deg - deg * kP = rpm
kI = 20
kD = 0
cP = 0.0005 #rpm - rpm * cP = angle
cI = 0
cD = 0
balanceAccumError = 0
balancePrevError = 0
setpointAccumError = 0
setpointPrevError = 0
defaultSetpoint = -0.58
a = 0
b = 0
c = 0
d = 0
fi = open("run.txt", "w")

q = queue.Queue()

def setpointPID(target, actual, deltaTime):
    global cP, cI, cD, fi, setpointAccumError, setpointPrevError
    # where input is velocity, output is angle set point
    # positive angle is forward leaning -> forward velocity
    # if velocity is positive, reduce angle
    # example: if target is 0, actual velocity is 2, error = 2 I want to reduce angle by 0.05deg so kP should be around 0.025

    error = target-actual

    setpointAccumError += error*deltaTime
    derivative = (error-setpointPrevError)/deltaTime
    print("cP", cP * error)
    #print("cI", cI * setpointAccumError)
    #print("cD", cD * derivative)

    return (cP * error) + (cD * derivative) + (cI * balanceAccumError) + defaultSetpoint

#assumes that positive angle is forward leaning
def balancePID(target, actual, deltaTime):
    global kP, kI, kD, fi, balanceAccumError, balancePrevError

    #if the robot is leaning forward 20 degrees and wants to go to 0
    #this will generate a positive number
    error = actual-target

    #note: take for instance the robot is at 20 degrees and wants to go to zero this will be negative
    #(leaning forward generates a negative slope and derivative)
    derivative = (error-balancePrevError)/deltaTime
    if(abs(balanceAccumError*kI) < 20 or abs(balanceAccumError + error) < abs(balanceAccumError)):
        balanceAccumError += error*deltaTime

    print("kP", kP * error)
    print("kI", kI * balanceAccumError)
    #print("kD", kD * derivative)

    fi.write(str(kP*error) + ", " + str(kI*balanceAccumError) + ", " + str(kD*derivative) + ", " + str(target) + ", " + str(actual) + "\n")

    return (kP * error) + (kD * derivative) + (kI * balanceAccumError)


def inputTask():
    while True:
        desiredVelocity = input("enter velocity: ")
        if(len(desiredVelocity) == 0):
            #driverLEFT.stopMotor()
            #driverRIGHT.stopMotor()
            driverLEFT.rotateMotorOpenloop(0)
            driverRIGHT.rotateMotorOpenloop(0)
            print("Stopping motor, appending -1000")
            q.put(int(-1000))
            exit(0)
        else:
            var = defaultSetpoint
            try:
                var = int(desiredVelocity)
            except:
                print("illegal character")
            q.put(var)

def main():
    threading.Thread(target=inputTask, daemon=True).start()

    targAng = defaultSetpoint
    targVel = 0
    lastTime = time.time()

    while(True):
        if(not q.empty()):
            targVel = q.get()
        
        if(targVel == -1000):
            print("Stopping motor again, exitting thread")
            #driverLEFT.stopMotor()
            #driverRIGHT.stopMotor()
            driverLEFT.rotateMotorOpenloop(0)
            driverRIGHT.rotateMotorOpenloop(0)
            time.sleep(1)
            exit(0)

        currAng = IMU.getAngle()
        currVel = (driverLEFT.getVelocity() + driverRIGHT.getVelocity())/2
        targAngtemp = setpointPID(targVel, currVel, time.time()-lastTime)
        output = balancePID(targAng, currAng, time.time()-lastTime)
        lastTime = time.time()
        print("target angle: ", targAngtemp)
        #print("RIGHT IS GOING: ", int(output))
        print("ANGLE: ", currAng) 
        print("Motor speed: ", int(output))
        driverLEFT.rotateMotorOpenloop(int(output))
        driverRIGHT.rotateMotorOpenloop(int(output))


main()
driverLEFT.stopMotor()
driverRIGHT.stopMotor()
time.sleep(1)
print("End of script")
