import odrive
from odrive.enums import *

odrv0 = odrive.find_any()

odrv0.axis0.clear_errors()
odrv0.axis1.clear_errors()