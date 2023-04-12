import sys
sys.path.append('../')
import odrive
from odrive.enums import *
import time
import matplotlib.pyplot as plt
import numpy as np
import IMU


from odriveDriver import Axis
plt.switch_backend('Agg')

plot_dir = './plots/'


odrv0 = odrive.find_any()


a0 = Axis(odrv0.axis0, dir=1)
a1 = Axis(odrv0.axis1, dir=1)

a0.setup()
a1.setup()


# a0_init_pos = a0.get_pos_turns()
# a1_init_pos = a1.get_pos_turns()
# print(a0.get_pos_turns())
# print(a1.get_pos_turns())
# print(a0.get_vel())
# print(a1.get_vel())
a0.set_trq(0)
a1.set_trq(0) 
# print("RUNNING")

# time.sleep(5)

# a0.set_trq(0.0)
# a1.set_trq(0.0)
# print(a0.get_pos_turns() - a0_init_pos)
# print(a1.get_pos_turns() - a1_init_pos)
# print(a0.get_pos_turns())
# print(a1.get_pos_turns())
# print(a0.get_vel())
# print(a1.get_vel())

