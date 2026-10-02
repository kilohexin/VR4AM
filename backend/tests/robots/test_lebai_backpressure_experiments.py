"""Synthetic adapter experiments. Every client is injected; no SDK/network access.

The one-joint feedback model integrates gain * commanded velocity over 40 ms.
It is a scheduling/constraint experiment, not a model of real robot dynamics.
"""

from __future__ import annotations

import asyncio
from collections import Counter, deque
from contextlib import asynccontextmanager
from dataclasses import replace
from unittest.mock import AsyncMock

import pytest

from app.control.coordinate_mapper import CoordinateMapper
from app.control.robot_control import LatestVRFrame, RobotControl
from app.control.safety import SafetyLimiter
from app.recording.noop import NoopRecorder
from app.robots.base import BackendCommandError, StopReason
from app.robots.lebai_adapter import RealLebaiAdapter
from app.schemas.messages import ControllerState, Pose, TeleopMode, VRFrame
from tests.robots.fake_lebai import FakeLebaiClient, IDLE_Q
from tests.robots.real_settings import control_settings


PERIOD_S = 0.04


def target(x: float = 0.307) -> Pose:
    return Pose(p=(x, 0.0, 0.4), q=(0.0, 0.0, 0.0, 1.0))


def joints(q0: float) -> list[float]:
    return [q0, *IDLE_Q[1:]]


class ManualClock:
    ns = 0

    def now_ns(self) -> int:
        return self.ns

    def advance(self, seconds: float) -> None:
        self.ns += round(seconds * 1_000_000_000)

    async def sleep(self, seconds: float) -> None:
        self.advance(seconds)
        await asyncio.sleep(0)


async def yield_only(_seconds: float) -> None:
    # Request steps advance the clock explicitly, once per 40 ms model step.
    await asyncio.sleep(0)


async def wait_until(predicate) -> None:
    async def wait() -> None:
        while not predicate():
            await asyncio.sleep(0)

    await asyncio.wait_for(wait(), timeout=1)


class Experiment:
    def __init__(self) -> None:
        self.client = FakeLebaiClient.idle()
        self.clock = ManualClock()
        self.events: list[dict[str, object]] = []
        self.sent: asyncio.Queue[dict[str, object]] = asyncio.Queue()
        settings = control_settings()
        settings = replace(
            settings,
            control=replace(settings.control, max_joint_speed_radps=0.05),
        )
        self.factory = AsyncMock(return_value=self.client)
        self.adapter = RealLebaiAdapter(
            settings,
            client_factory=self.factory,
            clock=self.clock.now_ns,
            sleep=self.clock.sleep,
            pump_clock=lambda: self.clock.now_ns() / 1_000_000_000,
            pump_sleep=yield_only,
            event_callback=self.record,
            # Exercise the production selection policy using only a fake client.
            backend_label="LEBAI",
        )
        self.command_id = 0

    async def record(self, event: dict[str, object], timestamp: int) -> None:
        entry = {**event, "server_mono_ns": timestamp}
        self.events.append(entry)
        if event.get("kind") == "pvat_sent":
            self.sent.put_nowait(entry)

    async def submit(self, solutions: list[float], pose: Pose | None = None) -> str | None:
        self.clock.advance(PERIOD_S)
        self.command_id += 1
        self.client.ik_results = deque(joints(q0) for q0 in solutions)
        try:
            await self.adapter.command_tcp(pose or target(), self.command_id)
        except BackendCommandError as error:
            # The existing asynchronous API enqueues before returning a previous
            # soft constraint. Await the pump result separately in each test.
            assert str(error) in {"ik_tracking_lag", "ik_joint_jump"}
            return str(error)
        return None

    async def step(self, q0: float, pose: Pose | None = None) -> tuple[dict[str, object], str | None]:
        feedback = await self.submit([q0], pose)
        event = await asyncio.wait_for(self.sent.get(), timeout=1)
        assert event["command_id"] == self.command_id
        assert self.adapter.pump_fault is None
        assert max(abs(v) for v in event["v"]) <= 0.05 + 1e-12
        assert max(abs(a) for a in event["a"]) <= 0.5 + 1e-12
        return event, feedback

    def integrate_feedback(self, event: dict[str, object], gain: float) -> None:
        q = list(self.client.kin_data["actual_joint_pose"])
        v = [gain * value for value in event["v"]]
        self.client.kin_data["actual_joint_pose"] = [
            position + speed * PERIOD_S for position, speed in zip(q, v, strict=True)
        ]
        self.client.kin_data["actual_joint_speed"] = v

    async def warm_up(self) -> None:
        # Build accepted history through real pump calls; feedback stays at zero.
        for index, q0 in enumerate((0.04, 0.08, 0.12, 0.16, 0.20, 0.22), start=1):
            event, feedback = await self.step(q0, target(0.3 + index * 0.001))
            assert event["pvat_mode"] == "advance"
            assert feedback is None
        assert self.adapter._last_accepted_solution_q[0] == pytest.approx(0.22)
        assert self.adapter._accepted_target_command_id == 6
        assert event["p"][0] == pytest.approx(0.004)
        assert self.client.kin_data["actual_joint_pose"][0] == 0.0


@asynccontextmanager
async def experiment():
    run = Experiment()
    await run.adapter.connect()  # Calls only the injected AsyncMock factory.
    try:
        await run.warm_up()
        yield run
    finally:
        run.client.release_ik.set()
        # The fake stop has no dynamics; make cleanup verification deterministic.
        run.client.kin_data["actual_joint_speed"] = [0.0] * 6
        await run.adapter.disconnect()
        run.factory.assert_awaited_once()
        assert run.client.network_calls == 0
        assert run.client.discovery_calls == 0
        assert not run.adapter._pump.running


@pytest.mark.asyncio
@pytest.mark.parametrize(("gain", "catch_up_steps"), [(1.0, 5), (0.25, 20)])
async def test_slow_feedback_quantifies_freeze_and_recovery(gain: float, catch_up_steps: int) -> None:
    async with experiment() as run:
        samples = []
        for index in range(catch_up_steps + 2):
            actual_before = run.client.kin_data["actual_joint_pose"][0]
            event, feedback = await run.step(0.26)
            samples.append(event)
            if index < catch_up_steps:
                assert event["pvat_mode"] == "catch_up"
                assert event["accepted_target_command_id"] == 6
                assert event["ik_solution_q"][0] == pytest.approx(0.22)
                assert event["target_tcp"] == target(0.306).model_dump()
                assert event["requested_tcp"] == target().model_dump()
                assert run.adapter.constraint == "tracking_lag"
            else:
                assert event["pvat_mode"] == "advance"
                assert event["accepted_target_command_id"] == event["command_id"]
                assert event["ik_solution_q"][0] == pytest.approx(0.26)
                assert run.adapter.constraint is None
                if index == catch_up_steps:
                    assert actual_before == pytest.approx(0.01)
                    assert event["tracking_error_rad"] == pytest.approx(0.25)
                    assert feedback == "ik_tracking_lag"  # Previous pump result.
                else:
                    assert feedback is None
            assert event["p"][0] == pytest.approx(actual_before + event["v"][0] * 0.08)
            run.integrate_feedback(event, gain)
        assert Counter(sample["pvat_mode"] for sample in samples) == {
            "catch_up": catch_up_steps, "advance": 2,
        }
        assert samples[catch_up_steps]["server_mono_ns"] - samples[0]["server_mono_ns"] == round(
            catch_up_steps * PERIOD_S * 1_000_000_000
        )
        assert run.adapter._consecutive_ik_failures == 0


@pytest.mark.asyncio
async def test_no_position_progress_stays_in_backpressure_for_four_seconds() -> None:
    async with experiment() as run:
        first = None
        for _ in range(101):
            event, _ = await run.step(0.26)
            first = first or event
            assert event["pvat_mode"] == "catch_up"
            assert event["accepted_target_command_id"] == 6
            assert event["p"][0] == pytest.approx(0.004)
            assert event["tracking_error_rad"] == pytest.approx(0.22)
            run.integrate_feedback(event, 0.0)
        assert event["server_mono_ns"] - first["server_mono_ns"] == 4_000_000_000
        assert run.adapter.constraint == "tracking_lag"
        assert run.adapter.pump_fault is None
        assert run.adapter._consecutive_ik_failures == 0


@pytest.mark.asyncio
async def test_branch_change_during_backpressure_does_not_replace_accepted_history() -> None:
    async with experiment() as run:
        for _ in range(3):
            event, _ = await run.step(0.26)
            run.integrate_feedback(event, 0.25)
        ik_calls_before = len(run.client.ik_calls)
        await run.submit([0.5, 0.45, 0.4], target(0.32))
        await wait_until(lambda: run.adapter.constraint == "motion_continuity_boundary")
        assert len(run.client.ik_calls) - ik_calls_before == 3
        assert run.sent.empty()
        assert run.adapter._last_accepted_solution_q[0] == pytest.approx(0.22)
        assert run.adapter._last_sent_tcp == target(0.306)
        assert run.adapter._accepted_target_command_id == 6
        assert run.adapter.pump_fault is None
        rejections = [
            event for event in run.events
            if event.get("kind") == "ik_candidate_rejected"
            and event.get("command_id") == run.command_id
        ]
        assert [event["reason"] for event in rejections] == ["ik_joint_jump"] * 3
        recovered, feedback = await run.step(0.24)
        assert feedback == "ik_joint_jump"
        assert recovered["pvat_mode"] == "advance"
        assert recovered["accepted_target_command_id"] == run.command_id
        assert run.adapter.constraint is None


@pytest.mark.asyncio
@pytest.mark.parametrize("operation", ["stop", "disconnect"])
async def test_stop_during_backpressure_discards_inflight_and_pending_targets(operation: str) -> None:
    async with experiment() as run:
        await run.step(0.26)
        run.client.ik_started.clear()
        run.client.block_ik = True
        await run.submit([0.26])
        await asyncio.wait_for(run.client.ik_started.wait(), timeout=1)
        # A newer pending request exists as well as the blocked in-flight solve.
        await run.submit([0.26], target(0.308))
        assert run.adapter._pump.has_pending
        writes_before = list(run.client.write_calls)
        generation = run.adapter._pump._generation
        task = asyncio.create_task(
            run.adapter.stop(StopReason.GRIP_RELEASED)
            if operation == "stop" else run.adapter.disconnect()
        )
        try:
            await wait_until(lambda: not run.adapter._pump.is_current(generation))
            run.client.release_ik.set()
            await asyncio.wait_for(task, timeout=1)
            assert not run.adapter._pump.has_pending
            assert run.sent.empty()
            assert run.adapter._last_accepted_solution_q is None
            assert run.adapter._accepted_target_command_id is None
            assert run.client.write_calls[len(writes_before):] == [("stop_move",)]
        finally:
            run.client.release_ik.set()
            if not task.done():
                task.cancel()
            await asyncio.gather(task, return_exceptions=True)


@pytest.mark.asyncio
async def test_stale_snapshot_cannot_send_a_catch_up_target() -> None:
    async with experiment() as run:
        await run.step(0.26)
        original = run.client.kinematics_inverse

        async def expired_ik(pose, seed):
            result = await original(pose, seed)
            run.clock.advance(0.341)
            return result

        run.client.kinematics_inverse = expired_ik
        writes_before = list(run.client.write_calls)
        await run.submit([0.26])
        await wait_until(lambda: run.adapter.pump_fault is not None)
        assert str(run.adapter.pump_fault) == "robot_state_stale"
        assert run.client.write_calls == writes_before
        assert run.sent.empty()


@pytest.mark.asyncio
async def test_control_pipeline_resumes_after_slow_feedback_and_links_samples_to_requests() -> None:
    class TraceRecorder(NoopRecorder):
        def __init__(self) -> None:
            self.samples = []

        async def write_event(self, event: object, server_mono_ns: int) -> None:
            if isinstance(event, dict) and event.get("kind") == "control_target_sample":
                self.samples.append({**event, "server_mono_ns": server_mono_ns})

    def frame(seq: int, grip: bool, x: float = 0.0) -> VRFrame:
        return VRFrame(
            v=1, type="vr_frame",
            session_id="synthetic-backpressure", seq=seq, client_mono_ms=seq * 40,
            tracking_valid=True, visibility="visible",
            right=ControllerState(p=(x, 0.0, 0.0), q=(0, 0, 0, 1), grip=grip, trigger=0),
        )

    async with experiment() as run:
        latest = LatestVRFrame()
        recorder = TraceRecorder()
        control = RobotControl(
            backend=run.adapter, latest=latest, clock=run.clock, recorder=recorder,
            mapper=CoordinateMapper(translation_scale=0.2),
            limiter=SafetyLimiter(max_linear_speed=0.005, max_linear_accel=0.02),
        )
        await control.connect()
        latest.publish(frame(101, False), run.clock.now_ns())
        await control.tick()
        await control.arm()
        latest.publish(frame(102, True), run.clock.now_ns())
        await control.tick()
        frozen_target = None
        pump_samples = {}
        for index in range(27):
            seq = 103 + index
            run.clock.advance(PERIOD_S)
            run.client.ik_results = deque([joints(0.26)])
            latest.publish(frame(seq, True, 0.03), run.clock.now_ns())
            await control.tick()
            event = await asyncio.wait_for(run.sent.get(), timeout=1)
            pump_samples[seq] = event
            assert event["command_id"] == seq
            if index == 0:
                frozen_target = control.last_target.model_copy(deep=True)
            elif index <= 20:
                # At index 20 the pump resumes, but command_tcp reported its
                # previous soft constraint; the next control input can advance.
                assert control.last_target == frozen_target
                assert (await control.state_message()).constraint == "tracking_lag"
            else:
                assert control.last_target.p[0] > frozen_target.p[0]
            assert event["pvat_mode"] == ("catch_up" if index < 20 else "advance")
            assert control.mode is TeleopMode.ACTIVE
            assert control._fault is None
            run.integrate_feedback(event, 0.25)

        assert (await control.state_message()).constraint is None
        assert recorder.samples
        for sample in recorder.samples:
            requested = pump_samples[sample["command_id"]]["requested_tcp"]
            assert Pose.model_validate(sample["limited_tcp"]) == Pose.model_validate(requested)
            assert sample["session_id"] == "synthetic-backpressure"
            assert sample["control_generation"] == control.control_generation
        first_sample = recorder.samples[0]
        first_pvat = pump_samples[first_sample["command_id"]]
        assert Pose.model_validate(first_sample["limited_tcp"]) != Pose.model_validate(first_pvat["target_tcp"])
