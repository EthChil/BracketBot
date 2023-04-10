import odrive
from odrive.enums import *
from odriveDriver import Axis

def brake_both_motors():
    odrv0 = odrive.find_any()
    a0 = Axis(odrv0.axis0, dir=1)
    a1 = Axis(odrv0.axis1, dir=-1)

    while abs(axis0.get_vel()) > 0.05 and abs(axis1.get_vel()) > 0.05:
        axis0.set_trq(-0.5 if axis0.get_vel() > 0 else 0.5)
        axis1.set_trq(-0.5 if axis1.get_vel() > 0 else 0.5)
        
    axis0.set_trq(0)
    axis1.set_trq(0)