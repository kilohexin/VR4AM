"""Trace test candidate state against real control/pump with injected SDK only."""

import asyncio
from collections import deque
from contextlib import asynccontextmanager

import numpy as np

from tests.control.brake_retarget_probe import BrakeBeforeRetarget
from tests.control.elapsed_step_probe import publish
from tests.control.test_braking_supervision import active_control
from tests.robots.test_lebai_backpressure_experiments import Experiment, joints


class ControlClock:
    def __init__(self, clock):
        self.clock = clock

    def now_ns(self):
        return self.clock.now_ns()

    def advance_ms(self, ms):
        self.clock.advance(ms / 1000)


class TracedRetarget(BrakeBeforeRetarget):
    def __init__(self, limits, run):
        super().__init__(limits)
        self.run = run
        self.generated = []
        self.resets = []

    def velocity(self, channel):
        return self.linear_velocity if channel == 'linear' else self.angular_velocity

    def reset_motion(self):
        self.resets.append(dict(seq=self.run.seq,
            speed=float(np.linalg.norm(self.velocity(self.run.channel))), braking=self.braking))
        super().reset_motion()

    def limit_motion(self, previous, requested, dt):
        result = super().limit_motion(previous, requested, dt)
        self.generated.append(dict(seq=self.run.seq, pose=result,
            velocity=self.velocity(self.run.channel).copy(), dt=dt, braking=self.braking))
        return result


class RetargetRun(Experiment):
    def __init__(self, channel):
        super().__init__()
        self.channel = channel
        self.control_clock = ControlClock(self.clock)
        self.seq = 2

    async def initialize(self):
        await self.adapter.connect()
        self.control, self.latest, _, _, self.bridge, self.recorder = await active_control(
            self.adapter, self.control_clock)
        self.candidate = TracedRetarget(self.bridge.limits, self)
        self.bridge.candidate = self.candidate

    async def submit(self, solutions, *, refresh=True, advance_ms=40, reverse=False):
        self.control_clock.advance_ms(advance_ms)
        if refresh:
            await self.adapter.get_state()
            self.client.ik_results = deque(joints(q) for q in solutions)
        else:
            self.client.ik_results.extend(joints(q) for q in solutions)
        self.seq += 1
        sign = -1 if reverse else 1
        publish(self.latest, self.control_clock, self.seq, True,
                sign * 0.1 if self.channel == 'linear' else 0,
                sign * 0.4 if self.channel == 'angular' else 0)
        await self.control.tick()
        assert self.control._fault is None

    async def step(self, solutions, **kwargs):
        await self.submit(solutions, **kwargs)
        event = await asyncio.wait_for(self.sent.get(), 1)
        assert event['command_id'] == self.seq
        assert self.adapter.pump_fault is None
        return event

    async def warm(self):
        for q in [0.04, 0.08, 0.12, 0.16, 0.20, 0.22, *([0.22] * 8)]:
            event = await self.step([q])
            assert event['pvat_mode'] == 'advance'
        assert self.adapter._accepted_target_command_id == self.seq
        assert self.control.last_target == self.adapter._last_sent_tcp
        assert np.linalg.norm(self.candidate.velocity(self.channel)) > 0


@asynccontextmanager
async def retarget_run(channel):
    run = RetargetRun(channel)
    try:
        await run.initialize()
        yield run
    finally:
        run.client.release_ik.set()
        run.client.kin_data['actual_joint_speed'] = [0.] * 6
        await run.adapter.disconnect()
        run.factory.assert_awaited_once()
        assert run.client.network_calls == run.client.discovery_calls == 0
        assert not run.adapter._pump.running
