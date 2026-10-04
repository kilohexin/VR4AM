"""Test-only timing substitution at filter/limiter boundaries, never a runtime API."""

import numpy as np
from scipy.spatial.transform import Rotation

from app.control.coordinate_mapper import CoordinateMapper
from app.control.filters import PoseFilter
from app.control.robot_control import LatestVRFrame, RobotControl
from app.recording.noop import NoopRecorder
from app.schemas.messages import ControllerState, TeleopMode, VRFrame
from tests.control.test_robot_control import FakeBackend, FakeClock
from tests.robots.test_vr_profile_experiments import limiter


class StepPolicy:
    def __init__(self, cap_ms):
        self.cap_ms = cap_ms
        self.last_ns = None
        self.elapsed = None
        self.dt = None

    def reset(self, now_ns):
        self.last_ns = now_ns
        self.elapsed = self.dt = None

    def consume(self, now_ns):
        if self.last_ns is None or now_ns <= self.last_ns:
            raise ValueError('nonpositive_elapsed')
        self.elapsed = (now_ns - self.last_ns) / 1e9
        self.last_ns = now_ns  # Discard excess elapsed time; no backlog.
        self.dt = 0.02 if self.cap_ms is None else min(self.elapsed, self.cap_ms / 1000)
        return self.dt


class Probe:
    def __init__(self, clock, cap_ms):
        self.clock = clock
        self.policy = StepPolicy(cap_ms)
        self.rows = []
        self.previous_velocity = np.zeros(3)
        self.previous_angular_velocity = np.zeros(3)

    def reset(self):
        self.policy.reset(self.clock.now_ns())
        self.previous_velocity[:] = 0
        self.previous_angular_velocity[:] = 0

    def wrap_limiter(self, limits):
        original = limits.limit_motion

        def measured(previous, requested, production_dt):
            assert production_dt == 0.02
            assert self.policy.last_ns == self.clock.now_ns()
            dt, elapsed = self.policy.dt, self.policy.elapsed
            before_v = limits.linear_velocity.copy()
            before_w = limits.angular_velocity.copy()
            result = original(previous, requested, dt)
            displacement = np.asarray(result.p) - np.asarray(previous.p)
            rotation = (Rotation.from_quat(result.q) * Rotation.from_quat(previous.q).inv()).as_rotvec()
            velocity = displacement / elapsed
            angular_velocity = rotation / elapsed
            self.rows.append(dict(
                dt=dt, elapsed=elapsed,
                step_m=float(np.linalg.norm(displacement)),
                step_rad=float(np.linalg.norm(rotation)),
                speed_mps=float(np.linalg.norm(velocity)),
                speed_radps=float(np.linalg.norm(angular_velocity)),
                # Right-interval finite differences of interval-average target
                # velocities. Not a continuous or physical acceleration trace.
                accel_mps2=float(np.linalg.norm(velocity - self.previous_velocity) / elapsed),
                accel_radps2=float(np.linalg.norm(angular_velocity - self.previous_angular_velocity) / elapsed),
                internal_accel_mps2=float(np.linalg.norm(limits.linear_velocity - before_v) / dt),
                internal_accel_radps2=float(np.linalg.norm(limits.angular_velocity - before_w) / dt),
            ))
            self.previous_velocity = velocity
            self.previous_angular_velocity = angular_velocity
            return result

        limits.limit_motion = measured


class ProbeFilter(PoseFilter):
    def __init__(self, probe):
        super().__init__()
        self.probe = probe

    def reset(self, pose):
        super().reset(pose)
        self.probe.reset()

    def clear(self):
        super().clear()
        self.probe.reset()

    def update(self, pose, production_dt):
        assert production_dt == 0.02
        return super().update(pose, self.probe.policy.consume(self.probe.clock.now_ns()))


def publish(latest, clock, seq, grip, x, rz):
    latest.publish(VRFrame(
        v=1, type='vr_frame', session_id='elapsed-step-offline', seq=seq,
        client_mono_ms=clock.now_ns() / 1e6, tracking_valid=True, visibility='visible',
        right=ControllerState(p=(x, 0, 0), q=tuple(Rotation.from_rotvec((0, 0, rz)).as_quat()),
                              grip=grip, trigger=0),
    ), clock.now_ns())


async def make_probe(cap_ms):
    clock, backend, latest = FakeClock(), FakeBackend(), LatestVRFrame()
    probe = Probe(clock, cap_ms)
    limits = limiter()
    probe.wrap_limiter(limits)
    control = RobotControl(
        backend=backend, latest=latest, clock=clock, recorder=NoopRecorder(),
        mapper=CoordinateMapper(translation_scale=0.2, rotation_dead_zone_deg=0.5),
        limiter=limits,
    )
    control.filter = ProbeFilter(probe)
    await control.connect()
    publish(latest, clock, 1, False, 0, 0)
    await control.tick()
    await control.arm()
    publish(latest, clock, 2, True, 0, 0)
    await control.tick()
    assert control.mode is TeleopMode.ACTIVE
    return control, latest, backend, clock, probe
