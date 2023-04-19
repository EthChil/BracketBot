from multiprocessing import Process, Manager, Event, Queue

from imu85_process import run_imu85_process
from moteus_control_process import run_moteus
# from keyboard_input_process import keyboard_input


if __name__ == "__main__":
    with Manager() as manager:
        
        input_dict = manager.dict()
        imu_and_odometry_dict = manager.dict()
        
        termination_event = Event()
        imu_setup = Event()
        
        imu85_process = Process(target=run_imu85_process, args=(termination_event, imu_setup, imu_and_odometry_dict))
        moteus_control = Process(target=run_moteus, args=(termination_event, imu_setup, imu_and_odometry_dict))
        # keyboard_input_process = Process(target=keyboard_input, args=(input_dict, termination_event))
            
        imu85_process.start()
        moteus_control.start()
        # keyboard_input_process.start()
        
        imu85_process.join()
        moteus_control.join()
        # keyboard_input_process.join()
        
        
        
        