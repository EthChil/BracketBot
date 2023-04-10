import odrive
from odrive.enums import *
import time
import matplotlib.pyplot as plt
import numpy as np
import IMU
import math

# from odriveDriver import Axis
plt.switch_backend('Agg')
plot_dir = './plots/'

# IMU SETUP
# print("Initializing IMU 085 from ego_motion.py")
IMU = IMU.IMU_BNO085()
# IMU.setupIMU()
# print("IMU 085 initialized")

import odrive
from odrive.enums import *
from odriveDriver import Axis

def EGO(position_array):
    
    
    print("EGO Motion Running...")
    
    odrv0 = odrive.find_any()
    a0 = Axis(odrv0.axis0, dir=1)
    a1 = Axis(odrv0.axis1, dir=-1)
    # a0.setup()
    # a1.setup()
    
    
    def update_position(x, y, theta, d_left, d_right, W):
        distance_left = d_left
        distance_right = d_right

        distance_avg = (distance_left + distance_right) / 2
        delta_theta = (distance_right - distance_left) / W

        x += distance_avg * math.cos(theta + delta_theta / 2)
        y += distance_avg * math.sin(theta + delta_theta / 2)
        theta += delta_theta

        return x, y, theta


    t2m = 0.528 #turns to meters
    x, y, theta = 0, 0, 0 # Initial position and heading

    # Constants
    W = 0.567   # Distance between wheels in meters



    pos_init_a0 = a0.get_pos_turns() * t2m
    pos_init_a1 = a1.get_pos_turns() * t2m
    

    pos_a0_cur = a0.get_pos_turns() * t2m - pos_init_a0
    pos_a1_cur = a1.get_pos_turns() * t2m - pos_init_a1
    pos_a0_prev = pos_a0_cur
    pos_a1_prev = pos_a1_cur
    


    start_time = time.time()
    cur_time = time.time() - start_time
    prev_time = time.time() - start_time
    

    xs = []
    ys = []
    thetas = []

    while True:
        time.sleep(0.001)
            
        cur_time = time.time() - start_time # relative time starts at 0
        dt = cur_time - prev_time
        
        yaw_angle = IMU.getYawAngle()
        pos_a0_cur = a0.get_pos_turns() * t2m - pos_init_a0
        pos_a1_cur = a1.get_pos_turns() * t2m - pos_init_a1
        
        pos_a0_delta = pos_a0_cur-pos_a0_prev
        pos_a1_delta = pos_a1_cur-pos_a1_prev


        x, y, theta = update_position(x, y, theta, pos_a0_delta, pos_a1_delta, W)
        # print(x, y, theta)
        xs.append(x)
        ys.append(y)
        thetas.append(theta)
        
        position_array.append([x,y,theta])

        pos_a0_prev = pos_a0_cur
        pos_a1_prev = pos_a1_cur
        prev_time = cur_time


# fig, axs = plt.subplots(nrows=1, ncols=1, figsize=(8, 16))

# # Plot 1
# axs.plot(ys, xs, label='ego')
# axs.set_title("dts")
# plt.tight_layout()
# plt.savefig(plot_dir+ "ego.png")
# plt.clf()

# brake_both_motors(a0, a1)