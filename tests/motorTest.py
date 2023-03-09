import sys
sys.path.append('../')
import odrive
from odrive.enums import *
import time
import matplotlib.pyplot as plt
import numpy as np
import IMU_uncool


from odriveDriver import Axis
plt.switch_backend('Agg')

plot_dir = './plots/'


odrv0 = odrive.find_any()


a0 = Axis(odrv0.axis0)
# a1 = Axis(odrv0.axis1, dir=-1)

a0.setup()

a0.set_trq(0.5)
# a1.setup()