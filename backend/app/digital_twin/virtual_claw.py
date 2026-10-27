"""Explicit local claw model alongside the official Docker arm simulator.

Never attempts native claw initialization or silently retries a failed driver.
This models aperture only; it does not simulate contact, force or grasp physics.
"""
import math
import time


class DockerArmWithVirtualClaw:
    gripper_provider='local_virtual'

    def __init__(self,native,*,clock=time.perf_counter_ns):
        self.native=native;self.clock=clock
        self._amplitude=0.;self._target=0.;self._force=30.;self._updated=clock()

    def __getattr__(self,name):return getattr(self.native,name)

    def _advance(self):
        now=self.clock();delta=max(0,now-self._updated)/1_000_000_000;self._updated=now
        step=60*delta;difference=self._target-self._amplitude
        self._amplitude+=max(-step,min(step,difference))

    async def get_claw(self):
        self._advance()
        return dict(force=self._force,amplitude=self._amplitude,weight=0.,
            hold_on=abs(self._target-self._amplitude)<1e-9)

    async def set_claw(self,force,amplitude):
        if any(type(x) not in (int,float) or not math.isfinite(x) for x in (force,amplitude)):
            raise ValueError('virtual_claw_invalid_target')
        if not 0<=force<=30 or not 0<=amplitude<=100:raise ValueError('virtual_claw_invalid_target')
        self._advance();self._target=float(amplitude);self._force=float(force)
