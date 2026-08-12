from __future__ import annotations

import asyncio
import json
import math
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

import app.acceptance.fake_lebai as fake_acceptance
from app.control.safety import SafetyLimiter
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


def _assert_safe_state(
    state: RobotStateMessage,
    harness: fake_acceptance.Harness,
) -> None:
    settings = harness.settings.lebai
    assert settings is not None
    actual_q = np.asarray(state.actual_q)
    assert state.mode is TeleopMode.ACTIVE
    assert state.fault is None
    assert state.constraint is None
    assert np.all(actual_q >= np.asarray(settings.soft_joint_min_rad))
    assert np.all(actual_q <= np.asarray(settings.soft_joint_max_rad))
    assert not is_self_colliding(state.actual_q, harness.client.model)


async def _drive_until_geometrically_confirmed(
    harness: fake_acceptance.Harness,
    *,
    frame_factory,
    target: Pose,
    sequence: int,
    max_cycles: int,
) -> int:
    """Characterize eventual pose reachability, not simulated or wall time."""

    period_s = 1 / harness.settings.lebai.control.loop_hz  # type: ignore[union-attr]
    confirmations = 0
    # The shared FakeClock can advance in both this driver and the PVAT pump.
    # This bound only prevents a hung test; cycles are not physical-time evidence.
    position_error = math.inf
    rotation_error = math.inf
    for _ in range(max_cycles):
        harness.latest.publish(frame_factory(sequence), harness.clock.now_ns())
        sequence += 1
        await harness.control.tick()
        await harness.clock.sleep(period_s)
        await asyncio.sleep(0)
        state = await fake_acceptance._publish_state(harness)
        _assert_safe_state(state, harness)
        position_error, rotation_error = _pose_error(state.actual_tcp, target)
        confirmations = (
            confirmations + 1
            if position_error <= 0.003 and rotation_error <= math.radians(2.0)
            else 0
        )
        if confirmations == 3:
            return sequence
    raise AssertionError(
        "geometric_confirmation_iteration_guard:"
        f"cycles={max_cycles}:position_error_m={position_error}:"
        f"rotation_error_rad={rotation_error}"
    )


def test_frozen_fake_prep_geometry_preserves_home_and_is_safe() -> None:
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


def test_fake_pick_place_workspace_projects_unconstrained_from_real_home_anchor() -> None:
    settings = fake_acceptance.load_digital_twin_settings(FAKE_CONFIG)
    assert settings.lebai is not None
    config = _config()
    workspace = config["workspace"]
    assert isinstance(workspace, dict)
    home = forward_pose(settings.lebai.home_q, LM3Model())
    prep_payload = config["prep_tcp"]
    assert isinstance(prep_payload, dict)
    prep = Pose(p=tuple(prep_payload["p"]), q=tuple(prep_payload["q"]))
    limiter = SafetyLimiter(
        anchor=home.p,
        workspace_half_extent_m=settings.lebai.control.max_relative_translation_m,
    )
    pick_center = np.asarray(home.p) + np.asarray(workspace["pick_block_center_from_home_m"])
    place_center = np.asarray(home.p) + np.asarray(workspace["place_block_center_from_home_m"])
    half = float(workspace["block_size_m"]) / 2
    lift = float(workspace["lift_m"])
    commanded = [
        (pick_center[0], pick_center[1] + half, pick_center[2]),
        (pick_center[0], pick_center[1] + half + lift, pick_center[2]),
        (place_center[0], pick_center[1] + half + lift, place_center[2]),
        (place_center[0], place_center[1] + half, place_center[2]),
    ]

    assert np.linalg.norm(np.asarray(commanded[0]) - np.asarray(prep.p)) < 0.10
    for target in commanded:
        projection = limiter.project_workspace(Pose(p=target, q=prep.q))
        assert projection.constrained is False
        assert projection.hold is False
        assert projection.pose.p == pytest.approx(target)


@pytest.mark.asyncio
async def test_fake_prep_and_full_local_envelope_are_geometrically_reachable_and_safe(
    monkeypatch,
) -> None:
    """Exercise full-pose reachability and safety without making timing claims."""

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

        sequence = await _drive_until_geometrically_confirmed(
            harness,
            frame_factory=prep_frame,
            target=prep,
            sequence=3,
            max_cycles=512,
        )
        workspace = config["workspace"]
        assert isinstance(workspace, dict)
        pick_center = np.asarray(home.actual_tcp.p) + np.asarray(
            workspace["pick_block_center_from_home_m"]
        )
        place_center = np.asarray(home.actual_tcp.p) + np.asarray(
            workspace["place_block_center_from_home_m"]
        )
        half = float(workspace["block_size_m"]) / 2
        lift = float(workspace["lift_m"])
        pick_place_targets = [
            (pick_center[0], pick_center[1] + half, pick_center[2]),
            (pick_center[0], pick_center[1] + half + lift, pick_center[2]),
            (place_center[0], pick_center[1] + half + lift, place_center[2]),
            (place_center[0], place_center[1] + half, place_center[2]),
        ]
        home_rotation = Rotation.from_quat(home.actual_tcp.q)
        hand_rotation = tuple(
            (Rotation.from_quat(prep.q) * home_rotation.inv()).as_quat()
        )
        for position in pick_place_targets:
            hand_target = Pose(
                p=tuple(
                    (np.asarray(position) - np.asarray(home.actual_tcp.p))
                    / translation_scale
                ),
                q=hand_rotation,
            )

            def pick_place_frame(seq: int):
                nonlocal hand
                hand = _advance_hand(hand, hand_target)
                return fake_acceptance._frame(
                    seq, grip=True, position=hand.p, rotation=hand.q
                )

            sequence = await _drive_until_geometrically_confirmed(
                harness,
                frame_factory=pick_place_frame,
                target=Pose(p=position, q=prep.q),
                sequence=sequence,
                max_cycles=512,
            )

    monkeypatch.setattr(fake_acceptance, "ACCEPTANCE_START_Q", prep_q)
    confirmed_targets: list[str] = []
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
            sequence = await _drive_until_geometrically_confirmed(
                harness,
                frame_factory=lambda seq, action=action: fake_acceptance._motion_frame(
                    action, harness.settings, seq
                ),
                target=target,
                sequence=3,
                max_cycles=512,
            )
            confirmed_targets.append(label)
            await _drive_until_geometrically_confirmed(
                harness,
                frame_factory=lambda seq: fake_acceptance._frame(seq, grip=True),
                target=before.actual_tcp,
                sequence=sequence,
                max_cycles=512,
            )
            confirmed_targets.append(f"{label}_return")

    assert len(confirmed_targets) == 24
