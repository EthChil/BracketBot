import odrive
from odrive.enums import *
from odriveDriver import Axis


def setup_odrive(odrive_setup_done):
    # ODRIVE SETUP
    odrv0 = odrive.find_any()
    a0 = Axis(odrv0.axis0, dir=1)
    a1 = Axis(odrv0.axis1, dir=-1)
    a0.setup()
    a1.setup()
    
    odrive_setup_done.set()