
from pynput import keyboard


def on_key_press(key, input_value, termination_event, imu_writer_to_logger, imu_reader_logger, controller_reader_logger, controller_writer_to_logger, imu_writer_to_controller, imu_reader_controller):
    try:
        key_name = key.char.upper()
    except AttributeError:
        key_name = None
    
    key_mapping = {'W': 1, 'S': 2, 'A': 3, 'D': 4}
    
    if key_name in ['W', 'A', 'S', 'D']:
        input_value.value = key_mapping[key_name]
    elif key_name == 'Q':  # Check for the 'Q' key
        print("cancelling")
        termination_event.set()  # Set the termination_event
        imu_writer_to_logger.close()
        imu_reader_logger.close()
        controller_reader_logger.close()
        controller_writer_to_logger.close()
        imu_writer_to_controller.close()
        imu_reader_controller.close()

def on_key_release(key, input_value):
    try:
        key_name = key.char.upper()
    except AttributeError:
        key_name = None
    
    if key_name in ['W', 'A', 'S', 'D']:
        input_value.value = -1

def keyboard_input(input_value, termination_event, imu_writer_to_logger, imu_reader_logger, controller_reader_logger, controller_writer_to_logger, imu_writer_to_controller, imu_reader_controller):
    default_value = -1
    input_value.value = default_value

    with keyboard.Listener(
        on_press=lambda key: on_key_press(key, input_value, termination_event, imu_writer_to_logger, imu_reader_logger, controller_reader_logger, controller_writer_to_logger, imu_writer_to_controller, imu_reader_controller),
        on_release=lambda key: on_key_release(key, input_value)
    ) as listener:
        while not termination_event.is_set():
            pass

        listener.stop()