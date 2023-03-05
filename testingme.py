from odriveDriver import Axis

import odrive

odrv0 = odrive.find_any()

new = Axis(odrv0.axis0)

new.setup()
