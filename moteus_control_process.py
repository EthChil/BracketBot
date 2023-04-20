import time
import matplotlib.pyplot as plt
import numpy as np
import math
import asyncio
import moteus

plt.switch_backend('Agg')
plot_dir = './plots/'

def update_position(x, y, theta, d_left, d_right, W):
        distance_left = d_left
        distance_right = d_right

        distance_avg = (distance_left + distance_right) / 2
        delta_theta = (distance_right - distance_left) / W

        x += distance_avg * math.cos(theta + delta_theta / 2)
        y += distance_avg * math.sin(theta + delta_theta / 2)
        theta += delta_theta

        return x, y, theta
    

async def control_main(termination_event, imu_and_odometry_dict, input_dict):
    fdcanusb = moteus.Fdcanusb(debug_log='./MASTER_LOGS/fdcanusb_debug.log')
    moteus1 = moteus.Controller(id=1, transport=fdcanusb)
    moteus2 = moteus.Controller(id=2, transport=fdcanusb)

    m1state = await moteus1.set_stop(query=True)
    m2state = await moteus2.set_stop(query=True)

    t2m = 0.528 #turns to meters
    W = 0.567   # Distance between wheels in meters
    max_torque_delta = 0.2
    
    
    #KEYBOARD CONTROL
    K_balance             = np.array([[-3.87, -6.55, -43.01, -20.61, -0.00, -0.00],[-0.00, -0.00, -0.00, -0.00, 0.32, 0.76]])
    K_forward_backward    = np.array([[-0.45, -6.82, -55.79, -26.84, 0.00, 0.00],[0.00, 0.00, 0.00, 0.00, 1.41, 1.54]])
    K_left_right          = np.array([[-1.73, -5.30, -41.64, -19.93, 0.00, 0.00],[0.00, 0.00, 0.00, 0.00, 0.03, 2.46]] )
    
    
    # SET THESE
    moteus1_direction = -1 
    moteus2_direction = -1

    moteus1_initial_position = m1state.values[moteus.Register.POSITION] * t2m * moteus1_direction
    moteus2_initial_position = m2state.values[moteus.Register.POSITION] * t2m * moteus2_direction
    combined_initial_position = (moteus1_initial_position + moteus2_initial_position)/2
    
    moteus1_previous_position = 0
    moteus2_previous_position = 0    
    moteus1_previous_torque_command = 0
    moteus2_previous_torque_command = 0
    x_ego, y_ego, theta_ego = 0, 0, 0
    
    pitch_angle85    = imu_and_odometry_dict.get("pitch_angle85", 0)
    yaw_angle85      = imu_and_odometry_dict.get("yaw_angle85", 0) 
    pitch_rate85 = imu_and_odometry_dict.get("pitch_rate85", 0)
    yaw_rate85   = imu_and_odometry_dict.get("yaw_rate85", 0)
    
    # FOR KEYBOARD CONTROL
    x_stable = 0
    yaw_stable = 0
    prev_dir = 0

    start_time = time.time()
    cur_time = 0
    prev_time = 0

    m1state = await moteus1.set_stop(query=True)
    m2state = await moteus2.set_stop(query=True)

    while cur_time < 600 and not termination_event.is_set():        
        cur_time = time.time() - start_time
        dt = cur_time - prev_time
        prev_time = cur_time
        
        
        # VALUE UPDATING
        moteus1_current_position = m1state.values[moteus.Register.POSITION] * t2m * moteus1_direction - moteus1_initial_position
        moteus2_current_position = m2state.values[moteus.Register.POSITION] * t2m * moteus2_direction - moteus2_initial_position
        
        moteus1_current_velocity = m1state.values[moteus.Register.VELOCITY] * t2m * moteus1_direction
        moteus2_current_velocity = m2state.values[moteus.Register.VELOCITY] * t2m * moteus2_direction
        
        # CLAYTON NEEDS THIS
        if abs(moteus1_current_velocity)>(2) or abs(moteus2_current_velocity)>(2):
            print("TOO FAST")
            termination_event.set()
        
        moteus1_delta_position = moteus1_current_position - moteus1_previous_position
        moteus2_delta_position = moteus2_current_position - moteus2_previous_position
        
        combined_current_position = (moteus1_current_position + moteus2_current_position)/2
        combined_current_velocity = (moteus1_current_velocity + moteus2_current_velocity)/2
        
        pitch_angle85    = imu_and_odometry_dict.get("pitch_angle85", 0)
        yaw_angle85      = imu_and_odometry_dict.get("yaw_angle85", 0) 
        pitch_rate85 = imu_and_odometry_dict.get("pitch_rate85", 0)
        yaw_rate85   = imu_and_odometry_dict.get("yaw_rate85", 0)
        
        x_ego, y_ego, theta_ego = update_position(x_ego, y_ego, theta_ego, moteus1_delta_position, moteus2_delta_position, W)
        
        # KEYBOARD CONTRL
        if input_dict.get("key", "NOPE") == "NONE":
            #0.3 is to allow it to slow down
            Xf_selected = np.array([x_stable+0.3*prev_dir, 0, 0.0, 0, yaw_stable, 0])
            K_selected = K_balance
            
        elif input_dict.get("key", "NOPE") == "W":
            prev_dir = 1
            x_stable = combined_current_position
            yaw_stable = -yaw_angle85
            Xf_selected = np.array([combined_current_position, 0.25, 0.01, 0, yaw_stable, 0])
            K_selected = K_forward_backward
            
        elif input_dict.get("key", "NOPE") == "S":
            prev_dir = -1
            x_stable = combined_current_position
            yaw_stable = -yaw_angle85
            Xf_selected = np.array([combined_current_position, -0.25, -0.01, 0, yaw_stable, 0])
            K_selected = K_forward_backward
            
        elif input_dict.get("key", "NOPE") == "A":
            prev_dir = 0
            x_stable = combined_current_position
            yaw_stable = -yaw_angle85
            Xf_selected = np.array([x_stable, 0, 0.01, 0, -yaw_angle85, -1.5])
            K_selected = K_left_right
            
        elif input_dict.get("key", "NOPE") == "D":
            prev_dir = 0
            x_stable = combined_current_position
            yaw_stable = -yaw_angle85
            Xf_selected = np.array([x_stable, 0, 0.01, 0, -yaw_angle85, 1.5])
            K_selected = K_left_right
        
        
        
        # CONTROL CODE
        X = np.array([
            combined_current_position, 
            combined_current_velocity, 
            -pitch_angle85, 
            pitch_rate85, 
            -yaw_angle85, 
            -yaw_rate85 
            ])
        D = np.array([[0.5, 0.5],[0.5, -0.5]])
        Cl, Cr = D @ (-K_selected @ (X - Xf_selected))
        
        #TORQUE RAMPING
        Cl = np.clip(Cl, moteus1_previous_torque_command - max_torque_delta, moteus1_previous_torque_command + max_torque_delta)
        Cr = np.clip(Cr, moteus2_previous_torque_command - max_torque_delta, moteus2_previous_torque_command + max_torque_delta)
        
        # m1state = await moteus1.set_position(position=math.nan, velocity=0.0, accel_limit=0.0, feedforward_torque=Cl, kp_scale=0, kd_scale=0, maximum_torque=5, query=True)
        # m2state = await moteus2.set_position(position=math.nan, velocity=0.0, accel_limit=0.0, feedforward_torque=Cr, kp_scale=0, kd_scale=0, maximum_torque=5, query=True)
        
        await moteus1.set_position(position=math.nan, velocity=0.0, accel_limit=0.0, feedforward_torque=0, kp_scale=0, kd_scale=0, maximum_torque=5, query=False)
        await moteus2.set_position(position=math.nan, velocity=0.0, accel_limit=0.0, feedforward_torque=0, kp_scale=0, kd_scale=0, maximum_torque=5, query=False)
        
        
        # LOGGING
        imu_and_odometry_dict['x_ego']                    = x_ego
        imu_and_odometry_dict['y_ego']                    = y_ego
        imu_and_odometry_dict['theta_ego']                = theta_ego
        imu_and_odometry_dict['moteus1_current_position'] = moteus1_current_position
        imu_and_odometry_dict['moteus2_current_position'] = moteus2_current_position
        imu_and_odometry_dict['moteus1_current_velocity'] = moteus1_current_velocity
        imu_and_odometry_dict['moteus2_current_velocity'] = moteus2_current_velocity
        imu_and_odometry_dict['moteus1_torque_reading']   = m1state.values[moteus.Register.TORQUE]
        imu_and_odometry_dict['moteus2_torque_reading']   = m2state.values[moteus.Register.TORQUE]
        imu_and_odometry_dict['moteus1_torque_command']   = Cl
        imu_and_odometry_dict['moteus2_torque_command']   = Cr
        imu_and_odometry_dict['dt_moteus_control']        = dt
    
        moteus1_previous_position = moteus1_current_position
        moteus2_previous_position = moteus2_current_position
        moteus1_previous_torque_command = Cl
        moteus2_previous_torque_command = Cr
        
        await asyncio.sleep(0)

    m1state = await moteus1.set_stop(query=True)
    m2state = await moteus2.set_stop(query=True)

def run_moteus(termination_event, imu_setup, imu85_dict, input_dict, orbslam_setup):
    imu_setup.wait() 
    orbslam_setup.wait()
    
    asyncio.run(control_main(termination_event, imu85_dict, input_dict))
    termination_event.set()