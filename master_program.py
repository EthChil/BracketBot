from multiprocessing import Process, Event, Value, Pipe

# from imu85_process import run_imu85_process
from uart_imu_process import run_uart_imu_process
from moteus_control_process import run_moteus
from logging_process import run_logging_process
from keyboard_input_process import keyboard_input
from orbslam_process import run_orbslam


if __name__ == "__main__":

    
    input_value = Value('i', -1) #-1 is balance , 1=W, 2=S, 3=A, 4=D reason is you have to use integers

    imu_writer_to_controller, imu_reader_controller = Pipe()
    imu_writer_to_logger, imu_reader_logger = Pipe()
    controller_writer_to_logger, controller_reader_logger = Pipe()

    
    termination_event = Event()
    imu_setup = Event()
    orbslam_setup = Event()
    orbslam_setup.set()
    
    uart_imu_process = Process(target=run_uart_imu_process, args=(termination_event, imu_setup, imu_writer_to_controller, imu_writer_to_logger))
    moteus_control = Process(target=run_moteus, args=(termination_event, imu_setup, imu_reader_controller, controller_writer_to_logger, input_value, orbslam_setup))
    logging_process = Process(target=run_logging_process, args=(termination_event, imu_setup, imu_reader_logger, controller_reader_logger ))
    keyboard_input_process = Process(target=keyboard_input, args=(input_value, termination_event, imu_writer_to_logger, imu_reader_logger, controller_reader_logger, controller_writer_to_logger, imu_writer_to_controller, imu_reader_controller))
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
        
        
        
        