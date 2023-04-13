from multiprocessing import Process, Manager, Event, Queue
import signal
# import keyboard
from pynput import keyboard
import time

from odrive_setup_process import setup_odrive
from imu_process import start_imu
from odrive_process import run_odrive
from logging_process import logger
from orbslam_process import run_orbslam

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
    keys = {'w', 'a', 's', 'd'}

    def on_press(key):
        try:
            if key.char.lower() in keys:
                drive_mode["key"] = key.char.upper()
        except AttributeError:
            pass

    def on_release(key):
        drive_mode["key"] = default_value

    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()
                
            
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
        set_points = manager.dict()
        
        imu55_dict = manager.dict()
        imu85_dict = manager.dict()
        drive_mode = manager.dict()
        drive_mode["mode"] = selected_drive_mode
        
        odrive_setup_process = Process(target=setup_odrive, args=(odrive_setup_done, ))
        odrive_setup_process.start()
        odrive_setup_process.join()
        
        imu_runner_process = Process(target=start_imu, args=(imu85_dict, imu55_dict, imu_setup_done, termination_event))
        odrive_runner_process = Process(target=run_odrive, args=(drive_mode, imu_setup_done, odrive_setup_done, imu85_dict, imu55_dict, ego_estimation, drive_stats, set_points, termination_event))
        logger_runner_process = Process(target=logger, args=(drive_mode, imu_setup_done, odrive_setup_done, imu85_dict, imu55_dict, ego_estimation, drive_stats, set_points, termination_event))
        keyboard_input_process = Process(target=keyboard_input, args=(drive_mode, termination_event))
        # orbslam_runner_process = Process(target=run_orbslam, args=('ORBvoc.txt', 'arducam.yaml', termination_event))

        imu_runner_process.start()
        odrive_runner_process.start()
        logger_runner_process.start()
        keyboard_input_process.start()
        # orbslam_runner_process.start()
        
        imu_runner_process.join()
        odrive_runner_process.join()
        logger_runner_process.join()
        keyboard_input_process.join()
        # orbslam_runner_process.join()
        
        
          
        
        

        
    
