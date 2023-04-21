import odrive
from odrive.enums import *
from odriveDriver import Axis


odrv0 = odrive.find_any()
a0 = Axis(odrv0.axis0, dir=1)
a1 = Axis(odrv0.axis1, dir=-1)
a0.setup()
a1.setup()

while True:
    print(a0.get_pos_turns(), a1.get_pos_turns())
    