import asyncio
import math
import moteus

class MoteusController:
    def __init__(self, device_id=1, direction=1):
        self.device_id = device_id
        self.loop = asyncio.get_event_loop()
        
        # self.fdcanusb = self.loop.run_until_complete(self._initFD())
    
        self.controller = moteus.Controller(id=self.device_id)
        
        self.direction = direction
        self.state = self.stop()
        

    def stop(self):
        self.state = self.loop.run_until_complete(self._stop())
        return self.state

    def get_position(self):
        return self.state.values[moteus.Register.POSITION] * self.direction
    
    def get_velocity(self):
        return self.state.values[moteus.Register.VELOCITY] * self.direction
    
    def get_torque(self):
        return self.state.values[moteus.Register.TORQUE] * self.direction

    def set_torque(self, torque, **kwargs):
        return self.loop.run_until_complete(self._set_torque(torque=torque*self.direction, **kwargs))

    async def _stop(self):
        self.state = await self.controller.set_stop(query=True)
        return self.state

    async def _set_torque(self, torque, **kwargs):
        self.state = await self.controller.set_position(position=math.nan, velocity=0.0, accel_limit=0.0, feedforward_torque=torque, kp_scale=0, kd_scale=0, maximum_torque=10, query=True)