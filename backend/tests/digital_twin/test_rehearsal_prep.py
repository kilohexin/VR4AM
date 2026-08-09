from __future__ import annotations

import asyncio
import json
import math
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

import app.acceptance.fake_lebai as fake_acceptance
from app.commissioning.actions import RotationAction, TranslationAction
from app.schemas.messages import Pose, RobotStateMessage, TeleopMode
from app.sim.kinematics import forward_pose, geometric_jacobian
from app.sim.lm3_model import LM3Model
from app.sim.self_collision import is_self_colliding


ROOT = Path(__file__).resolve().parents[3]
FAKE_CONFIG = ROOT / "config" / "fake-lebai.yaml"
REHEARSAL_CONFIG = ROOT / "config" / "fake-offline-rehearsal.json"


def _config() -> dict[str, object]:
    return json.loads(REHEARSAL_CONFIG.read_text(encoding="utf-8"))


def _pose_error(actual: Pose, target: Pose) -> tuple[float, float]:
    position = float(np.linalg.norm(np.asarray(actual.p) - np.asarray(target.p)))
    rotation = float(
        (
            Rotation.from_quat(target.q)
            * Rotation.from_quat(actual.q).inv()
        ).magnitude()
    )
    return position, rotation


def _advance_hand(current: Pose, target: Pose) -> Pose:
    delta = np.asarray(target.p) - np.asarray(current.p)
    distance = float(np.linalg.norm(delta))
    position = np.asarray(current.p) + delta * (
        0.0 if distance == 0.0 else min(1.0, 0.002 / distance)
    )
    error = Rotation.from_quat(target.q) * Rotation.from_quat(current.q).inv()
    vector = error.as_rotvec()
    angle = float(np.linalg.norm(vector))
    step = min(angle, math.radians(1.0))
    rotation = (
        Rotation.from_rotvec(vector * (0.0 if angle == 0.0 else step / angle))
        * Rotation.from_quat(current.q)
    )
    return Pose(p=tuple(position), q=tuple(rotation.as_quat()))


def _assert_safe_state(state: RobotStateMessage, model: LM3Model) -> None:
    assert state.mode is TeleopMode.ACTIVE
    assert state.fault is None
    assert state.constraint is None
    assert not is_self_colliding(state.actual_q, model)


async def _drive_until_confirmed(
    harness: fake_acceptance.Harness,
    *,
    frame_factory,
    target: Pose,
    sequence: int,
    timeout_s: float,
) -> tuple[int, float]:
    period_s = 1 / harness.settings.lebai.control.loop_hz  # type: ignore[union-attr]
    confirmations = 0
    for cycle in range(math.ceil(timeout_s / period_s)):
        harness.latest.publish(frame_factory(sequence), harness.clock.now_ns())
        sequence += 1
        await harness.control.tick()
        await harness.clock.sleep(period_s)
        await asyncio.sleep(0)
        state = await fake_acceptance._publish_state(harness)
        _assert_safe_state(state, harness.client.model)
        position_error, rotation_error = _pose_error(state.actual_tcp, target)
        confirmations = (
            confirmations + 1
            if position_error <= 0.003 and rotation_error <= math.radians(2.0)
            else 0
        )
        if confirmations == 3:
            return sequence, (cycle + 1) * period_s
    raise AssertionError("authoritative_confirmation_timeout")


def test_frozen_fake_prep_preserves_home_and_has_robust_margin() -> None:
    settings = fake_acceptance.load_digital_twin_settings(FAKE_CONFIG)
    assert settings.lebai is not None
    config = _config()
    prep_q = np.asarray(config["prep_q"], dtype=float)
    prep_tcp = config["prep_tcp"]
    assert isinstance(prep_tcp, dict)
    model = LM3Model()

    assert settings.lebai.home_q == pytest.approx(model.home_q)
    assert settings.lebai.home_q == pytest.approx(
        (0.0, -math.pi / 4, math.pi / 2, -math.pi / 4, math.pi / 2, -math.pi / 2)
    )
    assert np.all(prep_q >= np.asarray(settings.lebai.soft_joint_min_rad))
    assert np.all(prep_q <= np.asarray(settings.lebai.soft_joint_max_rad))
    assert not is_self_colliding(prep_q, model)
    assert float(np.min(np.linalg.svd(geometric_jacobian(prep_q, model), compute_uv=False))) >= 0.08
    actual = forward_pose(prep_q, model)
    assert actual.p == pytest.approx(prep_tcp["p"], abs=1e-12)
    assert actual.q == pytest.approx(prep_tcp["q"], abs=1e-12)


@pytest.mark.asyncio
async def test_fake_prep_and_full_local_envelope_confirm_with_timeout_margin(monkeypatch) -> None:
    config = _config()
    prep_q = tuple(float(value) for value in config["prep_q"])
    prep_payload = config["prep_tcp"]
    assert isinstance(prep_payload, dict)
    prep = Pose(p=tuple(prep_payload["p"]), q=tuple(prep_payload["q"]))
    model = LM3Model()

    monkeypatch.setattr(fake_acceptance, "ACCEPTANCE_START_Q", tuple(model.home_q))
    async with fake_acceptance.fake_real_harness() as harness:
        home = await fake_acceptance._prepare_active(harness)
        translation_scale = harness.settings.lebai.control.translation_scale  # type: ignore[union-attr]
        hand = Pose(p=(0.0, 0.0, 0.0), q=(0.0, 0.0, 0.0, 1.0))
        target_delta = Rotation.from_quat(prep.q) * Rotation.from_quat(home.actual_tcp.q).inv()
        hand_target = Pose(
            p=tuple(
                (np.asarray(prep.p) - np.asarray(home.actual_tcp.p)) / translation_scale
            ),
            q=tuple(target_delta.as_quat()),
        )

        def prep_frame(sequence: int):
            nonlocal hand
            if sequence % 2 == 1:
                hand = _advance_hand(hand, hand_target)
            return fake_acceptance._frame(
                sequence, grip=True, position=hand.p, rotation=hand.q
            )

        _, prep_elapsed = await _drive_until_confirmed(
            harness,
            frame_factory=prep_frame,
            target=prep,
            sequence=3,
            timeout_s=8.0,
        )
        assert prep_elapsed <= 8.0

    monkeypatch.setattr(fake_acceptance, "ACCEPTANCE_START_Q", prep_q)
    results: dict[str, float] = {}
    actions = [
        TranslationAction(axis, sign * 0.020)
        for axis in ("x", "y", "z")
        for sign in (1.0, -1.0)
    ] + [
        RotationAction(axis, sign * 8.0)
        for axis in ("roll", "pitch", "yaw")
        for sign in (1.0, -1.0)
    ]
    for action in actions:
        async with fake_acceptance.fake_real_harness() as harness:
            before = await fake_acceptance._prepare_active(harness)
            axis = action.axis
            label = f"{'+' if (action.distance_m if isinstance(action, TranslationAction) else action.angle_deg) > 0 else '-'}{axis}"
            if isinstance(action, TranslationAction):
                position = np.asarray(before.actual_tcp.p).copy()
                position[{"x": 0, "y": 1, "z": 2}[axis]] += action.distance_m
                target = Pose(p=tuple(position), q=before.actual_tcp.q)
            else:
                rotation_axis = {"roll": "x", "pitch": "y", "yaw": "z"}[axis]
                target = Pose(
                    p=before.actual_tcp.p,
                    q=tuple(
                        (
                            Rotation.from_euler(rotation_axis, action.angle_deg, degrees=True)
                            * Rotation.from_quat(before.actual_tcp.q)
                        ).as_quat()
                    ),
                )
            sequence, elapsed = await _drive_until_confirmed(
                harness,
                frame_factory=lambda seq, action=action: fake_acceptance._motion_frame(
                    action, harness.settings, seq
                ),
                target=target,
                sequence=3,
                timeout_s=6.5,
            )
            results[label] = elapsed
            _, return_elapsed = await _drive_until_confirmed(
                harness,
                frame_factory=lambda seq: fake_acceptance._frame(seq, grip=True),
                target=before.actual_tcp,
                sequence=sequence,
                timeout_s=6.5,
            )
            results[f"{label}_return"] = return_elapsed

    assert len(results) == 24
    assert max(results.values()) <= 6.5
