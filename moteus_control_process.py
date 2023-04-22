import time
import matplotlib.pyplot as plt
import numpy as np
import math
import asyncio
import moteus
import os
from numba import jit

from filterpy.kalman import KalmanFilter#, predict, update
# import numba

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
  
@jit(nopython=True)
def predict(x, P, F, Q, u, G):
    x = F @ x + G @ u
    P = (F @ P @ F.T) + Q
    return x, P

@jit(nopython=True)
def update(x, P, z, R, H):
    y = z - H @ x
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)
    x = x + K @ y
    KH = K @ H
    I_KH = np.eye(KH.shape[0]) - KH
    P = I_KH @ P @ I_KH.T + K @ R @ K.T
    return x, P


async def control_main(termination_event, input_value, imu_shared_array, odometry_shared_array, LQR_state_array):
    fdcanusb = moteus.Fdcanusb(debug_log='./MASTER_LOGS/fdcanusb_debug.log')
    
    qr = moteus.QueryResolution()
    qr.position = moteus.multiplex.F32
    qr.velocity = moteus.multiplex.F32
    qr.torque = moteus.multiplex.F32
    qr.voltage = moteus.multiplex.F32
    
    moteus1 = moteus.Controller(id=1, transport=fdcanusb, query_resolution=qr)
    moteus2 = moteus.Controller(id=2, transport=fdcanusb, query_resolution=qr)

    m1state = await moteus1.set_stop(query=True)
    m2state = await moteus2.set_stop(query=True)

    print(m1state.values[moteus.Register.VOLTAGE])
    if m1state.values[moteus.Register.VOLTAGE] < 31:
        print("YOOO BATTERY LOW WE AINT RUNNING HOE")
        termination_event.set()
        
    t2m = 0.528 #turns to meters
    W = 0.567   # Distance between wheels in meters
    max_torque_delta = 0.5
    
    
    #KEYBOARD CONTROL
    K_balance             = np.array([[-1.00, -2.43, -25.97, -12.60, -0.00, -0.00],[-0.00, -0.00, -0.00, -0.00, 1.00, 1.27]] )
    K_forward_backward    = np.array([[-0.71, -7.85, -61.71, -29.74, 0.00, -0.00],[0.00, 0.00, 0.00, 0.00, 2.24, 1.98]] )
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
    
    # for claytons camera floor planner
    egos = []
    dts_to_save = []
    
    with imu_shared_array.get_lock():
        imu_data = imu_shared_array[:]
    
    pitch_angle85    = imu_data[0]
    yaw_angle85      = imu_data[1]
    pitch_rate85 = imu_data[2]
    yaw_rate85   = imu_data[3]
    
    # FOR KEYBOARD CONTROL
    x_stable = 0
    yaw_stable = 0
    prev_dir = 0

    cur_time = 0
    prev_time = 0

    m1state = await moteus1.set_stop(query=True)
    m2state = await moteus2.set_stop(query=True)
    
    
    # Kalman Filter
    N=6
    f = KalmanFilter(dim_x=N, dim_z=N, dim_u=2)
    A  = np.loadtxt('./lqr_params/A.txt', delimiter=',')[:N,:N]
    B = np.loadtxt('./lqr_params/B.txt', delimiter=',')[:N]
    C = np.eye(N, dtype=float)
    Q_cov = np.loadtxt('./lqr_params/Q_cov.txt')
    P = np.loadtxt('./lqr_params/P.txt') if os.path.exists('./lqr_params/P.txt') else np.eye(6) * 1e-6
    
    H = C
    R = np.diag([1, 1e4, 1, 1e7, 1, 1e8]) * 1e-7
    Q = Q_cov[:N,:N]
    
    x_kf_init = [
        0,
        0,
        pitch_angle85,
        pitch_rate85,
        yaw_angle85,
        yaw_rate85
    ]
    x_hat = np.array(x_kf_init, dtype=float)
    
    U = np.array([0.,0.])
    
    # THIS STEP IS ONLY FOR NUMBA TO COMPILE THE FUNCTIONS
    F = A*0.01 + np.eye(N)
    G = B*0.01
    predict(x_hat, P, F, Q, U, G)
    update(x_hat, P, x_hat, R, H)
    
    states_to_save = []
    torques_to_save = []
    start_time = time.time()
    
    
    while cur_time < 600 and not termination_event.is_set():        
        cur_time = time.time() - start_time
        dt = cur_time - prev_time
        prev_time = cur_time
        
        dts_to_save.append(dt)

        
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
        
        with imu_shared_array.get_lock():
            imu_data = imu_shared_array[:]
    
        pitch_angle85    = imu_data[0]
        yaw_angle85      = imu_data[1]
        pitch_rate85 = imu_data[2]
        yaw_rate85   = imu_data[3]
                    
        x_ego, y_ego, theta_ego = update_position(x_ego, y_ego, theta_ego, moteus1_delta_position, moteus2_delta_position, W)
        egos.append([x_ego, y_ego, theta_ego, time.time()])
        
        # KEYBOARD CONTRL
        if input_value.value == -1:
            #0.3 is to allow it to slow down
            #Xf_selected = np.array([x_stable+0.3*prev_dir, 0, 0.0, 0, yaw_stable, 0])
            Xf_selected = np.array([combined_current_position, 0, 0.0, 0, -yaw_angle85, -yaw_rate85]) #FOR PUSHING AROUND
            K_selected = K_balance
            
        elif input_value.value == 1:
            prev_dir = 1
            x_stable = combined_current_position
            yaw_stable = -yaw_angle85
            Xf_selected = np.array([combined_current_position, 0.25, 0.02, 0, yaw_stable, 0])
            K_selected = K_forward_backward
            
        elif input_value.value == 2:
            prev_dir = -1
            x_stable = combined_current_position
            yaw_stable = -yaw_angle85
            Xf_selected = np.array([combined_current_position, -0.5, -0.03, 0, yaw_stable, 0])
            K_selected = K_forward_backward
            
        elif input_value.value == 3:
            prev_dir = 0
            x_stable = combined_current_position
            yaw_stable = -yaw_angle85
            Xf_selected = np.array([combined_current_position, 0, 0, 0, -yaw_angle85, -1.5])
            K_selected = K_left_right
            
        elif input_value.value == 4:
            prev_dir = 0
            x_stable = combined_current_position
            yaw_stable = -yaw_angle85
            Xf_selected = np.array([combined_current_position, 0, 0, 0, -yaw_angle85, 1.5])
            K_selected = K_left_right
        
    
        # CONTROL CODE
        z = np.array([
            combined_current_position, 
            combined_current_velocity, 
            -pitch_angle85, 
            pitch_rate85, 
            -yaw_angle85, 
            -yaw_rate85 
            ])
        
        #Kalman stuff
        F = A*dt + np.eye(N)
        G = B*dt
        x_hat, P = predict(x_hat, P, F, Q, U, G)
        x_hat, P = update(x_hat, P, z, R, H)
        
        D = np.array([[0.5, 0.5],[0.5, -0.5]])
        U = -K_selected @ (z - Xf_selected)
        Cl, Cr = D @ U

        states_to_save.append(x_hat)
        torques_to_save.append(U)
        dts_to_save.append(dt)
        
        #TORQUE RAMPING
        Cl = np.clip(Cl, moteus1_previous_torque_command - max_torque_delta, moteus1_previous_torque_command + max_torque_delta)
        Cr = np.clip(Cr, moteus2_previous_torque_command - max_torque_delta, moteus2_previous_torque_command + max_torque_delta)
        
        
        m1state = await moteus1.set_position(position=math.nan, velocity=0.0, accel_limit=0.0, feedforward_torque=Cl, kp_scale=0, kd_scale=0, maximum_torque=10, query=True)
        m2state = await moteus2.set_position(position=math.nan, velocity=0.0, accel_limit=0.0, feedforward_torque=Cr, kp_scale=0, kd_scale=0, maximum_torque=10, query=True)
        
        # LOGGING
        
        odometry_data = []
        
        odometry_data.append(x_ego)
        odometry_data.append(y_ego)
        odometry_data.append(theta_ego)
        odometry_data.append(moteus1_current_position)
        odometry_data.append(moteus2_current_position)
        odometry_data.append(moteus1_current_velocity)
        odometry_data.append(moteus2_current_velocity)
        odometry_data.append(m1state.values[moteus.Register.TORQUE])
        odometry_data.append(m2state.values[moteus.Register.TORQUE])
        odometry_data.append(Cl)
        odometry_data.append(Cr)
        odometry_data.append(dt)
        
        with odometry_shared_array.get_lock():
            odometry_shared_array[:] = odometry_data
            
        with LQR_state_array.get_lock():
            if x_hat[3]>2: #REMOVE THIS LATER ITS JUST TO REMOVEI NITIAL SPIKE
                LQR_state_array[:] = np.append(np.array([0,0,0,0,0,0]), np.array([0,0,0,0,0,0]))
            else:
                LQR_state_array[:] = np.append(z*np.array([1,1,-1,1,-1,-1]), x_hat*np.array([1,1,-1,1,-1,-1]))
                
                
                
            
        moteus1_previous_position = moteus1_current_position
        moteus2_previous_position = moteus2_current_position
        moteus1_previous_torque_command = Cl
        moteus2_previous_torque_command = Cr
        
        await asyncio.sleep(0)
        
        
    np.savetxt("./MASTER_LOGS/egos_for_camera.txt", egos)

    m1state = await moteus1.set_stop(query=True)
    m2state = await moteus2.set_stop(query=True)
    
    states_to_save = np.stack(states_to_save)
    np.savetxt("./MASTER_LOGS/states.txt", states_to_save)
    
    torques_to_save = np.stack(torques_to_save)
    np.savetxt("./MASTER_LOGS/torques.txt", torques_to_save)
    
    dts_to_save = np.stack(dts_to_save)
    np.savetxt("./MASTER_LOGS/dts.txt", dts_to_save)
    
    

def run_moteus(termination_event, imu_setup, imu_shared_array, odometry_shared_array,LQR_state_array, input_value, orbslam_setup):
    imu_setup.wait() 
    orbslam_setup.wait()
    
    asyncio.run(control_main(termination_event, input_value, imu_shared_array, odometry_shared_array,LQR_state_array))
