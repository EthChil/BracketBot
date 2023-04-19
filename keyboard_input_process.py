
from pynput import keyboard


def on_key_press(key, input_dict, termination_event):
    try:
        key_name = key.char.upper()
    except AttributeError:
        key_name = None
    
    if key_name in ['W', 'A', 'S', 'D']:
        input_dict["key"] = key_name
    elif key_name == 'Q':  # Check for the 'Q' key
        print("cancelling")
        termination_event.set()  # Set the termination_event

def on_key_release(key, input_dict):
    try:
        key_name = key.char.upper()
    except AttributeError:
        key_name = None
    
    if key_name in ['W', 'A', 'S', 'D']:
        input_dict["key"] = "NONE"

def keyboard_input(input_dict, termination_event):
    default_value = "NONE"
    input_dict["key"] = default_value

    with keyboard.Listener(
        on_press=lambda key: on_key_press(key, input_dict, termination_event),
        on_release=lambda key: on_key_release(key, input_dict)
    ) as listener:
        while not termination_event.is_set():
            pass

        listener.stop()