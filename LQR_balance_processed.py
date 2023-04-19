from multiprocessing import Process, Manager, Event, Queue

from imu85_process import run_imu85_process
from moteus_control_process import run_moteus

if __name__ == "__main__":
    with Manager() as manager:
        
        imu85_dict = manager.dict()
        termination_event = Event()
        imu_setup = Event()
        
        imu85_process = Process(target=run_imu85_process, args=(termination_event, imu_setup, imu85_dict))
        moteus_control = Process(target=run_moteus, args=(termination_event, imu_setup, imu85_dict))
            
        imu85_process.start()
        moteus_control.start()
        
        imu85_process.join()
        moteus_control.join()
        
        
        
        