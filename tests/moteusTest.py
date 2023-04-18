import sys
sys.path.append('../')
import moteusDriver

moteus1 = moteusDriver.MoteusController(device_id=1, direction=1)
moteus1.stop()

moteus2 = moteusDriver.MoteusController(device_id=2, direction=1)
moteus2.stop()

# while True:
#     print(moteus1.get_position())
#     print(moteus2.get_position())
    
while True:
    moteus1.set_torque(1)
    moteus2.set_torque(1)
    
    print(moteus1.get_velocity())
    print(moteus2.get_velocity())
    

