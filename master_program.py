from multiprocessing import Process, Event, Value, Pipe, Array

from uart_imu_process import run_uart_imu_process
#from camera_process import run_camera
from moteus_control_process import run_moteus
from logging_process import run_logging_process
from keyboard_input_process import keyboard_input
from orbslam_process import run_orbslam


if __name__ == "__main__":

    
    input_value = Value('i', -1) #-1 is balance , 1=W, 2=S, 3=A, 4=D reason is you have to use integers
    
    imu_shared_array = Array('d', [0.0] * 5)
    odometry_shared_array = Array('d', [0.0] * 12)
    LQR_state_array = Array('d', [0.0] * 12)
    
    termination_event = Event()
    imu_setup = Event()
    orbslam_setup = Event()
    orbslam_setup.set()
    
    moteus_control = Process(target=run_moteus, args=(termination_event, imu_setup, imu_shared_array, odometry_shared_array, LQR_state_array, input_value, orbslam_setup))
    uart_imu_process = Process(target=run_uart_imu_process, args=(termination_event, imu_setup, imu_shared_array))
    logging_process = Process(target=run_logging_process, args=(termination_event, imu_setup, imu_shared_array, odometry_shared_array, LQR_state_array))
    keyboard_input_process = Process(target=keyboard_input, args=(input_value, termination_event))
    #camera_process = Process(target=run_camera, args=(termination_event, ))
    # orbslam_runner_process = Process(target=run_orbslam, args=('ORBvoc.txt', 'arducam.yaml', termination_event, orbslam_setup))
        
    uart_imu_process.start()
    moteus_control.start()
    logging_process.start()
    keyboard_input_process.start()
    #camera_process.start()
    # orbslam_runner_process.start()
    
    uart_imu_process.join()
    moteus_control.join()
    logging_process.join()
    keyboard_input_process.join()
    #camera_process.join()
    # orbslam_runner_process.join()

        
        
        
        
