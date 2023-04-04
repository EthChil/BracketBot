
import odrive
from odrive.enums import *
import time
import matplotlib.pyplot as plt
import numpy as np
import IMU
import math

from odriveDriver import Axis
plt.switch_backend('Agg')

plot_dir = './plots/'

IMU = IMU.IMU_BNO085()
# IMU.restoreCalibrationConstants([0, 0, 85, 0, 1, 0, 166, 1, 77, 1, 176, 1, 1, 0, 0, 0, 0, 0, 232, 3, 178, 1]) only for bno55
IMU.setupIMU()

# IMU = IMU.IMU_BNO055(0, 40)
# IMU.restoreCalibrationConstants([0, 0, 85, 0, 1, 0, 166, 1, 77, 1, 176, 1, 1, 0, 0, 0, 0, 0, 232, 3, 178, 1]) #only for bno55
# IMU.setupIMU()

odrv0 = odrive.find_any()

ref_time = time.time()
t2m = 0.528 #turns to meters


#with ivans fix
K = np.array([[-5.77, -7.74, -42.86, -16.89, 0.00, 0.00],[-0.00, -0.00, -0.00, -0.00, 1.83, 1.77]] )


#PATH PLANNING STUFF
position_numbers = []
lines = 0

with open('paths/path1.txt', 'r') as file:
    x = 0
    y = 0
    for line in file:
        lines += 1
        stripped_line = line.strip()
        
        if stripped_line:
            try:
                x, y = stripped_line.split(',')
                position_numbers.append([float(x), float(y)])
            except:
                position_numbers.append([float(x), float(y)])
        
        
print(position_numbers)

pathplan_lps = 3 #path planning lines per second to read from the text file
path_time = lines/pathplan_lps

print('path time', path_time)


def LQR(axis0, axis1):
    # read initial values to offset
    x_init = axis0.get_pos_turns() * t2m
    
    pos_init = (axis0.get_pos_turns() * t2m + axis1.get_pos_turns() * t2m)/2

    pitch_angle=IMU.getPitchAngle() 
    yaw_angle=IMU.getYawAngle()
    pitch_rate=IMU.getPitchRate()
    yaw_rate=IMU.getYawRate()

    Xf = np.array([0, 0, 0, 0, 0, 0])

    start_time = time.time()
    cur_time = time.time() - start_time
    prev_time = time.time() - start_time

    # SAVE THINGS TO PLOT
    times = []
    dts = []
    xs = []
    vs = []
    dxdts = []

    pitchAngles = []
    yawAngles = []
    pitchRates = []
    yawRates = []

    Cl_commands = []
    Cr_commands = []
    
    posCommandeds = []
    posActuals = []
    
    angleCommandeds = []
    angleActuals = []

    while cur_time < path_time:
        time.sleep(0.0001)
        
        cur_time = time.time() - start_time # relative time starts at 0
        dt = cur_time - prev_time
        
        #stop the program if the torque or vel gets super high
        if abs(axis0.get_torque_input()) > 10:
            print("torque too high: ",axis0.get_torque_input(),"Nm")
            break
        if abs(axis0.get_vel()) > 3:
            print("velocity too high: ",axis0.get_vel(),"turns/s")
            break

        # if cur_time > 3:
        #     posCommand = 0.0 + (cur_time-3)*0.1
        #     Xf = np.array([posCommand, 0, 0, 0, 0, 0])
        posCommand = 0
        angleCommand = 0
        
        #PATH PLANNING
        index = min(round(cur_time * pathplan_lps), lines-1)
        posCommand = position_numbers[index][0]
        angleCommand = position_numbers[index][1]
        
        print(posCommand, angleCommand)
        
        Xf = np.array([posCommand, 0, 0, 0, angleCommand, 0])

        x = (axis0.get_pos_turns() * t2m  + axis1.get_pos_turns() * t2m)/2 - pos_init
        v = (axis0.get_vel() * t2m + axis1.get_vel() * t2m)/2

        
        yaw_angle= -IMU.getYawAngle() # positive for base mount
        pitch_angle= -IMU.getPitchAngle() # positive for base mount
        pitch_rate= IMU.getPitchRate() # negative for base mount
        yaw_rate= -IMU.getYawRate() # negative for base mount
        

        X = np.array([x, v, pitch_angle, pitch_rate, yaw_angle, yaw_rate])

        # U = -K @ (X - Xf)

        D = np.array([[0.5, 0.5],[0.5, -0.5]])
        # print("K mat: ", K)
        # print("X mat: ", X)
        # print("Xf mat: ", Xf)
        # print("D mat:", D)
        Cl, Cr = D @ (-K @ (X - Xf))
        # print("Cl: ", Cl)
        # print("Cr: ", Cr)
        
        
        axis0.set_trq(Cl)
        axis1.set_trq(Cr)
        

        times.append(cur_time)
        dts.append(dt)
        xs.append(x)
        vs.append(v)

        pitchAngles.append(pitch_angle)
        yawAngles.append(yaw_angle)
        pitchRates.append(pitch_rate)
        yawRates.append(yaw_rate)

        Cl_commands.append(Cl)
        Cr_commands.append(Cr)
        
        posCommandeds.append(posCommand)
        posActuals.append((axis0.get_pos_turns()*t2m + axis1.get_pos_turns()*t2m)/2 - pos_init)
        
        angleCommandeds.append(angleCommand)
        angleActuals.append(math.degrees(yaw_angle))
        

        # print(time.time() - start_time)
        
        prev_time = cur_time

    brake_both_motors(a0, a1)

    fig, axs = plt.subplots(nrows=9, ncols=1, figsize=(8, 16))

    # Plot 1
    axs[0].plot(times, dts, label='dts')
    axs[0].legend()
    axs[0].set_title("dts")

    # Plot 2
    axs[1].plot(times, xs, label='xs')
    axs[1].legend()
    axs[1].set_title("xs")

    # Plot 3
    axs[2].plot(times, vs, label='vs')
    axs[2].legend()
    axs[2].set_title("vs")

    # Plot 5
    axs[3].plot(times, pitchAngles, label='Pitch Angles')
    axs[3].legend()
    axs[3].set_title("Pitch Angles")

    # Plot 6
    axs[4].plot(times, pitchRates, label='pitch rates')
    axs[4].legend()
    axs[4].set_title("pitch rates")

    # Plot 7
    axs[5].plot(times, yawAngles, label='Yaw Angles')
    axs[5].legend()
    axs[5].set_title("Yaw Angles")

    # Plot 7
    axs[6].plot(times, yawRates, label='Yaw Rates')
    axs[6].legend()
    axs[6].set_title("Yaw Rates")

    axs[7].plot(times, Cl_commands, label='torque right')
    axs[7].plot(times, Cr_commands, label='torque left')
    axs[7].legend()
    axs[7].set_title("Balance Torques")
    
    axs[8].plot(times, posCommandeds, label='pos commanded')
    axs[8].plot(times, posActuals, label='pos actual')
    axs[8].legend()
    axs[8].set_title("Positions Planning")

    plt.tight_layout()
    plt.savefig(plot_dir+ "balance_plots.png")
    plt.clf()



def brake_both_motors(axis0, axis1):

    while abs(axis0.get_vel()) > 0.05 and abs(axis1.get_vel()) > 0.05:
        axis0.set_trq(-0.5 if axis0.get_vel() > 0 else 0.5)
        axis1.set_trq(-0.5 if axis1.get_vel() > 0 else 0.5)
        
    axis0.set_trq(0)
    axis1.set_trq(0)

        
a0 = Axis(odrv0.axis0, dir=1)
a1 = Axis(odrv0.axis1, dir=-1)

a0.setup()
a1.setup()

LQR(a0, a1)

brake_both_motors(a0, a1)
