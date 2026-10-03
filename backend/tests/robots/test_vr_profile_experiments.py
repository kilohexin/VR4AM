"""Synthetic candidate comparison. No real SDK, field configuration or network.

Run with pytest -s to print reproducible PROFILE_RESULT JSON rows. IK is an
explicit linear sensitivity fixture, NOT LM3 kinematics or a fitted field model.
"""

import asyncio
from collections import Counter
from dataclasses import replace
import json
import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from app.commissioning.low_speed_profile import LOW_SPEED_CAPS
from app.control.coordinate_mapper import CoordinateMapper
from app.control.filters import PoseFilter
from app.control.robot_control import LatestVRFrame, RobotControl
from app.control.safety import SafetyLimiter
from app.recording.noop import NoopRecorder
from app.schemas.messages import ControllerState, Pose, TeleopMode, VRFrame
from tests.robots.real_settings import control_settings
from tests.robots.test_lebai_backpressure_experiments import Experiment, joints, target


SCALES = (0.1, 0.2, 0.35)
GESTURES = ("translation", "rotation", "combined")
DT = 0.02  # Production CONTROL_DT_S, even for 25 Hz fresh input.
DURATION_S = 4
# Characterized counts for this deterministic model, not field predictions.
HIGH_SENSITIVITY_CATCHUPS = {
    0.25: {"translation": (0, 18, 18), "rotation": (0, 0, 0), "combined": (11, 34, 34)},
    0.0: {"translation": (0, 45, 45), "rotation": (3, 3, 3), "combined": (53, 60, 60)},
}


def hand(t, gesture):
    ramp = min(t, 1.0)
    x = 0.03 * ramp if gesture != "rotation" else 0.0
    angle = math.radians(6) * ramp if gesture != "translation" else 0.0
    return Pose(p=(x, 0, 0), q=tuple(Rotation.from_rotvec((0, 0, angle)).as_quat()))


def limiter():
    return SafetyLimiter(
        max_linear_speed=LOW_SPEED_CAPS["max_tcp_speed_mps"],
        max_angular_speed=LOW_SPEED_CAPS["max_tcp_rotation_radps"],
        max_linear_accel=LOW_SPEED_CAPS["max_tcp_acceleration_mps2"],
        max_angular_accel=LOW_SPEED_CAPS["max_tcp_angular_acceleration_radps2"],
        max_linear_step_m=LOW_SPEED_CAPS["max_tcp_step_m"],
        max_angular_step_rad=math.radians(LOW_SPEED_CAPS["max_tcp_rotation_step_deg"]),
        workspace_radius=math.inf,
    )


def emit(row):
    print("PROFILE_RESULT " + json.dumps(row, sort_keys=True))


@pytest.mark.parametrize("scale", SCALES)
@pytest.mark.parametrize("gesture", GESTURES)
@pytest.mark.parametrize("input_hz", (25, 50))
def test_requested_pipeline_speeds_and_endpoints(scale, gesture, input_hz):
    anchor = target(0.3)
    mapper = CoordinateMapper(translation_scale=scale, rotation_dead_zone_deg=0.5)
    mapper.capture(hand(0, gesture), anchor)
    filtering = PoseFilter()
    filtering.reset(anchor)
    limits = limiter()
    limits.set_pose_anchor(anchor)
    previous = anchor
    peak_mmps = peak_radps = 0.0
    rate_limited = 0
    for step in range(1, DURATION_S * input_hz + 1):
        mapped = mapper.target(hand(step / input_hz, gesture))
        projection = limits.project_workspace(mapped)
        assert not projection.constrained
        filtered = filtering.update(projection.pose, DT)
        limited = limits.limit_motion(previous, filtered, DT)
        distance = float(np.linalg.norm(np.asarray(limited.p) - previous.p))
        angle = (Rotation.from_quat(limited.q) * Rotation.from_quat(previous.q).inv()).magnitude()
        peak_mmps = max(peak_mmps, distance * input_hz * 1000)
        peak_radps = max(peak_radps, angle * input_hz)
        assert distance <= 0.005 * DT + 1e-12
        assert angle <= 0.05 * DT + 1e-12
        rate_limited += int(not np.allclose(limited.p, filtered.p, atol=1e-10, rtol=0)
                            or (Rotation.from_quat(limited.q) * Rotation.from_quat(filtered.q).inv()).magnitude() > 1e-10)
        previous = limited
    mapped_mm = (mapped.p[0] - anchor.p[0]) * 1000
    limited_mm = (limited.p[0] - anchor.p[0]) * 1000
    limited_deg = math.degrees(Rotation.from_quat(limited.q).magnitude())
    assert mapped_mm == pytest.approx(30 * scale if gesture != "rotation" else 0)
    assert math.degrees(Rotation.from_quat(mapped.q).magnitude()) == pytest.approx(6 if gesture != "translation" else 0)
    if input_hz == 50:
        assert limited_mm == pytest.approx(mapped_mm, abs=1e-7)
        assert limited_deg == pytest.approx(6 if gesture != "translation" else 0, abs=1e-7)
    if gesture != "rotation":
        assert peak_mmps <= 5 * input_hz / 50 + 1e-9
    emit(dict(kind="pipeline", scale=scale, gesture=gesture, input_hz=input_hz,
              mapped_mm=mapped_mm, limited_mm=limited_mm, limited_deg=limited_deg,
              peak_mmps=peak_mmps, peak_radps=peak_radps, rate_limited_frames=rate_limited))


@pytest.mark.parametrize("scale", SCALES)
@pytest.mark.parametrize("gesture", GESTURES)
@pytest.mark.parametrize("slope,angular_slope,response", ((10, 1, 0.25), (50, 3, 0.25), (50, 3, 0.0)))
async def test_profile_joint_backpressure_sensitivity(scale, gesture, slope, angular_slope, response):
    settings = control_settings()
    caps = {**LOW_SPEED_CAPS, "translation_scale": scale}
    run = Experiment(replace(settings, control=replace(settings.control, **caps)))
    # Keep the actual joint-to-TCP mapping unspecified: this fixture studies
    # selection/backpressure only, and never reports a physical TCP response.
    async def synthetic_ik(pose, seed):
        run.client.ik_calls.append((dict(pose), list(seed)))
        return joints(slope * (pose["x"] - 0.3) + angular_slope * pose["rz"])

    run.client.kinematics_inverse = synthetic_ik
    latest = LatestVRFrame()
    control = RobotControl(
        backend=run.adapter, latest=latest, clock=run.clock, recorder=NoopRecorder(),
        mapper=CoordinateMapper(translation_scale=scale, rotation_dead_zone_deg=0.5),
        limiter=limiter(),
    )

    def publish(seq, grip, pose):
        latest.publish(VRFrame(
            v=1, type="vr_frame", session_id="synthetic-profile", seq=seq,
            client_mono_ms=run.clock.now_ns() / 1_000_000,
            tracking_valid=True, visibility="visible",
            right=ControllerState(p=pose.p, q=pose.q, grip=grip, trigger=0),
        ), run.clock.now_ns())

    counts = Counter()
    first_catchup_s = None
    peak_joint_speed = 0.0
    try:
        await run.adapter.connect()  # Injected factory only; no real SDK imported.
        await control.connect()
        publish(1, False, hand(0, gesture))
        await control.tick()
        await control.arm()
        publish(2, True, hand(0, gesture))
        await control.tick()
        for step in range(1, DURATION_S * 25 + 1):
            # 50 Hz control ticks, but only 25 Hz distinct VR input frames.
            run.clock.advance(DT)
            await control.tick()
            assert run.sent.empty()
            run.clock.advance(DT)
            publish(step + 2, True, hand(step / 25, gesture))
            await control.tick()
            event = await asyncio.wait_for(run.sent.get(), timeout=1)
            assert event["command_id"] == step + 2
            counts[event["pvat_mode"]] += 1
            if event["pvat_mode"] == "catch_up" and first_catchup_s is None:
                first_catchup_s = step / 25
            peak_joint_speed = max(peak_joint_speed, max(abs(v) for v in event["v"]))
            assert peak_joint_speed <= 0.05 + 1e-12
            assert max(abs(a) for a in event["a"]) <= 0.2 + 1e-12
            assert control.mode is TeleopMode.ACTIVE
            assert run.adapter.pump_fault is None
            run.integrate_feedback(event, response)
        assert sum(counts.values()) == 100
        assert run.client.network_calls == run.client.discovery_calls == 0
        assert all(call[0] == "move_pvat" for call in run.client.write_calls)
        if slope == 10:
            assert counts["catch_up"] == 0
        else:
            assert counts["catch_up"] == HIGH_SENSITIVITY_CATCHUPS[response][gesture][SCALES.index(scale)]
        emit(dict(kind="sensitivity", scale=scale, gesture=gesture, input_hz=25,
                  slope_rad_per_m=slope, angular_slope=angular_slope, response=response,
                  advance=counts["advance"], catch_up=counts["catch_up"],
                  first_catchup_s=first_catchup_s, peak_joint_speed=peak_joint_speed,
                  final_accepted_mm=(event["target_tcp"]["p"][0] - 0.3) * 1000,
                  final_accepted_deg=math.degrees(Rotation.from_quat(event["target_tcp"]["q"]).magnitude())))
    finally:
        run.client.kin_data["actual_joint_speed"] = [0.0] * 6
        await run.adapter.disconnect()
        run.factory.assert_awaited_once()
