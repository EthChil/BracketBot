import odrive
from odrive.enums import *
import time
import matplotlib.pyplot as plt
import numpy as np
import IMU
from sshkeyboard import listen_keyboard
from threading import Thread
from queue import Queue

queue = Queue()


from odriveDriver import Axis
plt.switch_backend('Agg')

plot_dir = './plots/'

IMU = IMU.IMU(0, 40)
IMU.setupIMU()

odrv0 = odrive.find_any()

ref_time = time.time()
t2m = 0.528 #turns to meters

K = np.array([[-5.77, -6.92, -35.52, -10.83, -0.00, 0.00],[-0.00, -0.00, -0.00, 0.00, 1.83, 1.77]]) #stronger R's set to 3
Xf = np.array([0, 0, 0, 0, 0, 0],dtype=np.float64)
newXf = Xf

def press(key):
    if key == 'w':
        queue.put(np.array([0.1, 0, 0, 0, 0, 0], dtype=np.float64))
        print("W Pressed, 0.1m forward")
    elif key == 's':
        queue.put(np.array([-0.1, 0, 0, 0, 0, 0], dtype=np.float64))
        print("S Pressed, 0.1m backward")
    elif key == 'a':
        queue.put(np.array([0, 0, 0, 0, 0, -0.5], dtype=np.float64))
        print("A Pressed, 0.5rad left")
    elif key == 'd':
        queue.put(np.array([0, 0, 0, 0, 0, 0.5], dtype=np.float64))
        print("D Pressed, 0.5rad right")
    print(f"'{key}' pressed")

def update_Xf():
    global Xf
    while True:
        # Get the next update from the queue and add it to Xf
        update = queue.get()
        Xf += update

def release(key):
    print(f"'{key}' released")

def keyboard_listener():
    listen_keyboard(on_press=press)

keyboard_thread = Thread(target=keyboard_listener)
keyboard_thread.start()

# Start the Xf update thread
update_thread = Thread(target=update_Xf)
update_thread.start()

def LQR(axis0, axis1):
    # read initial values to offset
    global Xf
    x_init = axis0.get_pos_turns() * t2m

    pitch_angle=IMU.getPitchAngle() 
    yaw_angle=IMU.getYawAngle()
    pitch_rate=IMU.getPitchRate()
    yaw_rate=IMU.getYawRate()

    

    # Xf = np.array([0, 0, 0, 0, 0, 0])

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

    while cur_time < 10:
        time.sleep(0.001)

        if 5 > cur_time > 3:
            Xf = np.array([0.0 + (cur_time-3)*0.1, 0, 0, 0, 0, 0])
            newXf = Xf

        if 6 > cur_time > 5:
            Xf = np.array([0, 0.0 + (cur_time-5)*0.1, 0, 0, 0, 0]) + newXf
            newXf = Xf



        #stop the program if the vel gets super high
        if abs(axis0.get_vel()) > 2:
            break

        cur_time = time.time() - start_time # relative time starts at 0
        dt = cur_time - prev_time

        x = axis0.get_pos_turns() * t2m - x_init
        v = axis0.get_vel() * t2m

        pitch_angle=IMU.getPitchAngle() 
        yaw_angle=IMU.getYawAngle()
        pitch_rate=-IMU.getPitchRate()
        yaw_rate=-IMU.getYawRate()

        X = np.array([x, v, pitch_angle, pitch_rate, yaw_angle, yaw_rate])

        U = -K @ (X - Xf)

        D = np.array([[0.5, 0.5],[0.5, -0.5]])
        Cl, Cr = D @ (-K @ (X - Xf))


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

        prev_time = cur_time

    brake_both_motors(a0, a1)

    fig, axs = plt.subplots(nrows=8, ncols=1, figsize=(8, 16))

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

# Wait for the threads to finish
keyboard_thread.join()
update_thread.join()


brake_both_motors(a0, a1)




