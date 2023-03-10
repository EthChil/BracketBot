import odrive
from odrive.enums import *

odrv0 = odrive.find_any()

odrv0.clear_errors()
