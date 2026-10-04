"""Synthetic cadence characterization; no SDK, network or production changes.

RobotControl.tick is called explicitly: this does NOT simulate its run-loop
overrun policy. PvatPump is real; its handler only waits and records a target.
"""

import asyncio
import json

import pytest

from app.control.coordinate_mapper import CoordinateMapper
from app.control.filters import PoseFilter
from app.control.robot_control import CONTROL_DT_S, LatestVRFrame, RobotControl
from app.recording.noop import NoopRecorder
from app.robots.base import StopReason
from app.robots.lebai_pump import PvatPump
from app.schemas.messages import TeleopMode
from tests.control.test_robot_control import FakeBackend, FakeClock, frame
from tests.robots.test_vr_profile_experiments import limiter


class Timeline(FakeClock):
    def __init__(self):
        super().__init__()
        self.waiters = []

    async def sleep(self, seconds):
        if seconds <= 0:
            await asyncio.sleep(0)
            return
        future = asyncio.get_running_loop().create_future()
        self.waiters.append((self.value + round(seconds * 1e9), future))
        await future

    async def step(self):
        self.advance_ms(1)
        for deadline, future in self.waiters:
            if deadline <= self.value and not future.done():
                future.set_result(None)
        self.waiters = [(t, f) for t, f in self.waiters if not f.done()]
        await settle()


async def settle():
    # Bounded turns for handler completion -> zero sleep -> next pending request.
    # All nonzero waits are driven by Timeline, never by wall-clock time.
    for _ in range(4):
        await asyncio.sleep(0)


class ObservedFilter(PoseFilter):
    def __init__(self, clock):
        super().__init__()
        self.clock = clock
        self.rows = []

    def update(self, pose, dt):
        filtered = super().update(pose, dt)
        self.rows.append((self.clock.now_ns(), pose, filtered, dt))
        return filtered


class QueuedBackend(FakeBackend):
    def __init__(self, clock, handler_ms):
        super().__init__()
        self.clock = clock
        self.handler_ms = handler_ms
        self.started = []
        self.completed = []
        self.submitted_ns = []
        self.pump = PvatPump(
            self.handle, period_s=0.04,
            clock=lambda: clock.now_ns() / 1e9, sleep=clock.sleep,
        )

    async def command_tcp(self, target, command_id):
        await super().command_tcp(target, command_id)
        self.submitted_ns.append(self.clock.now_ns())
        self.pump.submit(target, command_id)

    async def handle(self, request):
        self.started.append((self.clock.now_ns(), request))
        await self.clock.sleep(self.handler_ms / 1000)
        if self.pump.is_current(request.generation):
            self.completed.append(request)

    async def stop(self, reason):
        self.pump.invalidate()
        await super().stop(reason)


async def setup(handler_ms=12):
    clock = Timeline()
    backend = QueuedBackend(clock, handler_ms)
    latest = LatestVRFrame()
    control = RobotControl(
        backend=backend, latest=latest, clock=clock, recorder=NoopRecorder(),
        mapper=CoordinateMapper(translation_scale=0.2), limiter=limiter(),
    )
    control.filter = ObservedFilter(clock)
    await backend.pump.start()
    await control.connect()
    latest.publish(frame(1, False, p=(0, 0, 0)), clock.now_ns())
    await control.tick()
    await control.arm()
    latest.publish(frame(2, True, p=(0, 0, 0)), clock.now_ns())
    await control.tick()
    assert control.mode is TeleopMode.ACTIVE
    return control, latest, backend, clock


async def run_case(input_ms, tick_ms, handler_ms=12):
    control, latest, backend, clock = await setup(handler_ms)
    seq = 2
    consumed = []
    at_one_second = None
    try:
        # Tie ordering: finish due handler waits, publish input, control tick,
        # then allow a newly awakened handler to run. Last input is at <=2 s.
        for ms in range(1, 2001):
            await clock.step()
            if ms % input_ms == 0:
                seq += 1
                x = 0.03 * min(ms / 1000, 1)
                latest.publish(frame(seq, True, p=(x, 0, 0)), clock.now_ns())
            if ms % tick_ms == 0:
                previous_count = len(backend.targets)
                await control.tick()
                if len(backend.targets) > previous_count:
                    consumed.append(latest.snapshot().frame.seq)
            await settle()
            if ms == 1000:
                at_one_second = dict(
                    mapped_mm=(control.filter.rows[-1][1].p[0] - 0.3) * 1000,
                    filtered_mm=(control.filter.rows[-1][2].p[0] - 0.3) * 1000,
                    limited_mm=(backend.targets[-1][1].p[0] - 0.3) * 1000,
                )
        assert control.mode is TeleopMode.ACTIVE
        assert not backend.stops
        window_started = len(backend.started)
        # Drain transport only, with no more ticks: outside the 2 s control
        # window, not an assertion that active control tolerates a stale input.
        for _ in range(200):
            await clock.step()
        assert not backend.pump.has_pending
        assert len(backend.started) == len(backend.completed)
        assert backend.pump.fault is None
        assert backend.completed[-1].command_id == backend.targets[-1][0]
        ids = [item.command_id for _, item in backend.started]
        starts = [at for at, _ in backend.started]
        # Serial handlers never overlap. An idle wait can shift a start past
        # the periodic deadline, so the next start need not be >=40 ms later.
        assert all(b - a >= handler_ms * 1_000_000
                   for a, b in zip(starts, starts[1:]))
        assert ids == sorted(set(ids))
        assert set(ids) <= set(consumed)
        assert len(consumed) == len(control.filter.rows)
        assert all(row[3] == CONTROL_DT_S == 0.02 for row in control.filter.rows)
        prior_x = backend.actual_tcp.p[0]
        prior_ns = 1_000_000_000
        peak_mmps = 0.0
        for (command_id, pose), at_ns in zip(backend.targets, backend.submitted_ns):
            delta = pose.p[0] - prior_x
            assert -1e-12 <= delta <= 0.005 * 0.02 + 1e-12
            peak_mmps = max(peak_mmps, delta * 1e12 / (at_ns - prior_ns))
            prior_x, prior_ns = pose.p[0], at_ns
        row = dict(
            input_ms=input_ms, tick_ms=tick_ms, handler_ms=handler_ms,
            published=seq - 2, consumed=len(consumed),
            input_overwritten=seq - 2 - len(consumed),
            integration_budget_ms=len(consumed) * 20,
            mapped_mm=(control.filter.rows[-1][1].p[0] - 0.3) * 1000,
            filtered_mm=(control.filter.rows[-1][2].p[0] - 0.3) * 1000,
            limited_mm=(backend.targets[-1][1].p[0] - 0.3) * 1000,
            peak_submission_mmps=peak_mmps,
            handler_started_in_window=window_started,
            handler_started_after_drain=len(ids),
            pending_overwritten=len(consumed) - len(ids),
            final_handled_mm=(backend.completed[-1].target.p[0] - 0.3) * 1000,
            at_one_second=at_one_second,
            min_handler_start_interval_ms=min(b - a for a, b in zip(starts, starts[1:])) / 1e6,
        )
        print('CADENCE_RESULT ' + json.dumps(row, sort_keys=True))
        return row, backend.targets
    finally:
        await backend.pump.stop()
        assert not backend.pump.running


@pytest.mark.parametrize('input_ms', [20, 27, 40])
@pytest.mark.parametrize('tick_ms', [10, 20, 40])
async def test_fixed_dt_depends_on_consumed_frames(input_ms, tick_ms):
    row, _ = await run_case(input_ms, tick_ms)
    # Independent latest-slot count with no dependence on filter/limiter code.
    eligible = [ms // input_ms for ms in range(tick_ms, 2001, tick_ms)]
    expected = len(set(eligible) - {0})
    assert row['consumed'] == expected
    assert row['mapped_mm'] == pytest.approx(6)
    assert row['limited_mm'] <= min(6, expected * 0.1) + 1e-9
    if input_ms == 40 or tick_ms == 40:
        assert row['limited_mm'] == pytest.approx(4.424)
    if input_ms == 20 and tick_ms <= 20:
        assert row['limited_mm'] == pytest.approx(6, abs=1e-6)


@pytest.mark.parametrize('input_ms', [20, 27, 40])
async def test_extra_duplicate_ticks_do_not_improve_final_progress(input_ms):
    fast, _ = await run_case(input_ms, 10)
    normal, _ = await run_case(input_ms, 20)
    assert fast['consumed'] == normal['consumed']
    assert fast['integration_budget_ms'] == normal['integration_budget_ms']
    assert fast['limited_mm'] == pytest.approx(normal['limited_mm'])


@pytest.mark.parametrize('input_ms', [20, 27, 40])
@pytest.mark.parametrize('handler_ms', [53, 80])
async def test_slow_handler_coalesces_without_changing_control_targets(input_ms, handler_ms):
    baseline, baseline_targets = await run_case(input_ms, 20, 12)
    slow, slow_targets = await run_case(input_ms, 20, handler_ms)
    assert slow_targets == baseline_targets
    assert slow['pending_overwritten'] > baseline['pending_overwritten']
    assert slow['handler_started_in_window'] < baseline['handler_started_in_window']
    assert slow['final_handled_mm'] == pytest.approx(slow['limited_mm'])


@pytest.mark.parametrize('age_ms', [20, 99, 100, 101, 250])
async def test_duplicate_ticks_and_stale_pause_do_not_integrate(age_ms):
    control, latest, backend, clock = await setup()
    try:
        clock.advance_ms(20)
        latest.publish(frame(3, True, p=(0.03, 0, 0)), clock.now_ns())
        await control.tick()
        before = list(backend.targets)
        clock.advance_ms(age_ms)
        await control.tick()
        assert backend.targets == before
        assert len(control.filter.rows) == 1
        if age_ms < 100:
            assert control.mode is TeleopMode.ACTIVE
            assert not backend.stops
            latest.publish(frame(4, True, p=(0.03, 0, 0)), clock.now_ns())
            await control.tick()
            assert len(backend.targets) == 2
            assert backend.targets[-1][1].p[0] - before[-1][1].p[0] <= 0.0001 + 1e-12
            assert control.filter.rows[-1][3] == 0.02
        else:
            assert StopReason.STALE in backend.stops
            assert control.mode is TeleopMode.STALE
            latest.publish(frame(4, True, p=(0.03, 0, 0)), clock.now_ns())
            await control.tick()
            assert backend.targets == before
            assert control.mode is not TeleopMode.ACTIVE
    finally:
        await backend.pump.stop()
