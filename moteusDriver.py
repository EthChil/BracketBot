import asyncio
import math
import moteus

class MoteusController:
    def __init__(self, device_id=1, direction=1):
        self.device_id = device_id
        self.controller = moteus.Controller(id=self.device_id)
        self.loop = asyncio.get_event_loop()
        self.direction = direction

    def stop(self):
        return self.loop.run_until_complete(self._stop())

    def get_position(self):
        return self.loop.run_until_complete(self._get_position()) * self.direction
    
    def get_velocity(self):
        return self.loop.run_until_complete(self._get_velocity()) * self.direction

    def set_torque(self, torque, **kwargs):
        return self.loop.run_until_complete(self._set_torque(torque=torque*self.direction, **kwargs))

    async def _stop(self):
        await self.controller.set_stop()

    async def _get_position(self):
        state = await self.controller.set_position(position=math.nan, query=True)
        return state.values[moteus.Register.POSITION]
    
    async def _get_velocity(self):
        state = await self.controller.set_position(position=math.nan, query=True)
        return state.values[moteus.Register.VELOCITY]

    async def _set_torque(self, torque, **kwargs):
        await self.controller.set_position(position=math.nan, velocity=0.0, accel_limit=2.0, feedforward_torque=torque, kp_scale=0, kd_scale=0, maximum_torque=5, **kwargs)