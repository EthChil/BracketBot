from multiprocessing import Process, Event, Value, Manager

# from imu85_process import run_imu85_process
from uart_imu_process import run_uart_imu_process
from moteus_control_process import run_moteus
from logging_process import run_logging_process
from keyboard_input_process import keyboard_input
from orbslam_process import run_orbslam


if __name__ == "__main__":
    with Manager() as manager:
        
        input_dict = manager.dict()
        input_dict['key'] = "NONE"
        
        
        imu_and_odometry_dict = manager.dict()
        
        termination_event = Event()
        imu_setup = Event()
        orbslam_setup = Event()
        orbslam_setup.set()
        
        uart_imu_process = Process(target=run_uart_imu_process, args=(termination_event, imu_setup, imu_and_odometry_dict))
        moteus_control = Process(target=run_moteus, args=(termination_event, imu_setup, imu_and_odometry_dict, input_dict, orbslam_setup))
        logging_process = Process(target=run_logging_process, args=(termination_event, imu_setup, imu_and_odometry_dict ))
        keyboard_input_process = Process(target=keyboard_input, args=(input_dict, termination_event))
        # orbslam_runner_process = Process(target=run_orbslam, args=('ORBvoc.txt', 'arducam.yaml', termination_event, orbslam_setup))
            
        uart_imu_process.start()
        moteus_control.start()
        logging_process.start()
        keyboard_input_process.start()
        # orbslam_runner_process.start()
        
        uart_imu_process.join()
        moteus_control.join()
        logging_process.join()
        keyboard_input_process.join()
        # orbslam_runner_process.join()
        
        
        
        