from multiprocessing import Process, Manager, Event, Queue
import signal
import keyboard
import time

from odrive_setup_process import setup_odrive
from imu_process import start_imu
from odrive_process import run_odrive
from logging_process import logger

ALLOWED_MODES = ["IDLE", "BALANCE", "KEYBOARD"]

def prompt_drive_mode():
    while True:
        mode = input("Enter drive mode (IDLE, BALANCE, KEYBOARD): ").upper()
        if mode in ALLOWED_MODES:
            return mode
        else:
            return "IDLE"
        print("Invalid drive mode. Please enter one of the allowed modes.")
        
def sigint_handler(signal, frame):
    print("Ctrl+C pressed, setting termination event")
    termination_event.set()


def keyboard_input(drive_mode, termination_event):
    default_value = "NONE" 
    drive_mode["key"] = default_value

    while not termination_event.is_set():
        time.sleep(0.05)
        event = keyboard.read_event()
        event_name_upper = event.name.upper()
        print(event_name_upper)

        if event_name_upper in ['W', 'A', 'S', 'D']:
            if event.event_type == keyboard.KEY_DOWN:  # If a key is pressed
                drive_mode["key"] = event_name_upper
            elif event.event_type == keyboard.KEY_UP:  # If a key is released
                drive_mode["key"] = default_value
            
if __name__ == "__main__":
    signal.signal(signal.SIGINT, sigint_handler)
    selected_drive_mode = prompt_drive_mode()
    
    with Manager() as manager:
        
        imu_setup_done = Event() 
        odrive_setup_done = Event() 
        termination_event = Event()

        # position_list = manager.list([None])
        ego_estimation = manager.dict()
        drive_stats = manager.dict()
        
        imu55_dict = manager.dict()
        imu85_dict = manager.dict()
        drive_mode = manager.dict()
        drive_mode["mode"] = selected_drive_mode
        
        odrive_setup_process = Process(target=setup_odrive, args=(odrive_setup_done, ))
        odrive_setup_process.start()
        odrive_setup_process.join()
        
        imu_runner_process = Process(target=start_imu, args=(imu85_dict, imu55_dict, imu_setup_done, termination_event))
        odrive_runner_process = Process(target=run_odrive, args=(drive_mode, imu_setup_done, odrive_setup_done, imu85_dict, imu55_dict, ego_estimation, drive_stats, termination_event))
        logger_runner_process = Process(target=logger, args=(drive_mode, imu_setup_done, odrive_setup_done, imu85_dict, imu55_dict, ego_estimation, drive_stats, termination_event))
        keyboard_input_process = Process(target=keyboard_input, args=(drive_mode, termination_event))


        imu_runner_process.start()
        odrive_runner_process.start()
        logger_runner_process.start()
        keyboard_input_process.start()
        
        imu_runner_process.join()
        odrive_runner_process.join()
        logger_runner_process.join()
        keyboard_input_process.join()
        
        
          
        
        

        
    
