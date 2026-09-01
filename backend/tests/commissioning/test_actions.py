from __future__ import annotations

import asyncio
import math
from pathlib import Path

import pytest
from unittest.mock import AsyncMock

from app.commissioning.actions import (
    GripperAction,
    HomeAction,
    PrepareAction,
    RotationAction,
    SmokeAction,
    SmokeOptions,
    StopAction,
    TranslationAction,
)
from app.commissioning.smoke import parse_smoke_args, run_smoke
from app.config import REAL_ROBOT_CONFIRMATION
from tests.config.test_real_robot_config import REAL_CONFIG_TEMPLATE
from tests.robots.fake_lebai import FakeLebaiClient


ROOT = Path(__file__).resolve().parents[3]
CONFIG = ROOT / "config" / "fake-lebai.yaml"


@pytest.mark.parametrize("distance", [-0.005, 0.005])
def test_translate_accepts_signed_five_millimetres(distance: float) -> None:
    options = parse_smoke_args(
        [
            "translate",
            "--config",
            str(CONFIG),
            "--axis",
            "x",
            "--distance-m",
            str(distance),
            "--confirm",
            REAL_ROBOT_CONFIRMATION,
        ]
    )
    assert options.action == TranslationAction("x", distance)


@pytest.mark.parametrize("distance", [-0.0051, 0.0, 0.0051, math.nan])
def test_translate_rejects_out_of_bounds_values(distance: float) -> None:
    with pytest.raises(ValueError, match="smoke_translation_out_of_bounds"):
        parse_smoke_args(
            [
                "translate",
                "--config",
                str(CONFIG),
                "--axis",
                "x",
                "--distance-m",
                str(distance),
                "--confirm",
                REAL_ROBOT_CONFIRMATION,
            ]
        )


@pytest.mark.parametrize("angle", [-2.0, 2.0])
def test_rotate_accepts_signed_two_degrees(angle: float) -> None:
    options = parse_smoke_args(
        [
            "rotate",
            "--config",
            str(CONFIG),
            "--axis",
            "roll",
            "--angle-deg",
            str(angle),
            "--confirm",
            REAL_ROBOT_CONFIRMATION,
        ]
    )
    assert options.action == RotationAction("roll", angle)


def test_prepare_parses_as_an_explicit_action() -> None:
    options = parse_smoke_args(
        [
            "prepare",
            "--config",
            str(CONFIG),
            "--confirm",
            REAL_ROBOT_CONFIRMATION,
        ]
    )

    assert options.action == PrepareAction()


def _control_config(tmp_path: Path) -> Path:
    path = tmp_path / "control.yaml"
    path.write_text(
        REAL_CONFIG_TEMPLATE.format(mode="control"), encoding="utf-8"
    )
    return path


@pytest.mark.asyncio
@pytest.mark.parametrize(
    ("action", "error"),
    [
        (TranslationAction("x", 1.0), "smoke_translation_out_of_bounds"),
        (RotationAction("yaw", 90.0), "smoke_rotation_out_of_bounds"),
    ],
)
async def test_run_smoke_rejects_unbounded_constructed_actions_before_connecting(
    tmp_path: Path,
    action: SmokeAction,
    error: str,
) -> None:
    factory = AsyncMock()

    with pytest.raises(ValueError, match=rf"^{error}$"):
        await run_smoke(
            SmokeOptions(
                config_path=tmp_path / "does-not-need-to-exist.yaml",
                action=action,
                confirmation=REAL_ROBOT_CONFIRMATION,
            ),
            factory,
        )

    factory.assert_not_awaited()


@pytest.mark.asyncio
@pytest.mark.parametrize(
    ("action", "expected_methods"),
    [
        (TranslationAction("x", 0.005), {"move_pvat", "stop_move"}),
        (RotationAction("yaw", 2.0), {"move_pvat", "stop_move"}),
        (GripperAction("close"), {"set_claw", "stop_move"}),
        (PrepareAction(), {"movej"}),
        (HomeAction(), {"stop_move", "movej"}),
        (StopAction(), {"stop_move"}),
    ],
)
async def test_each_commissioning_action_uses_one_write_category(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    action: SmokeAction,
    expected_methods: set[str],
) -> None:
    client = FakeLebaiClient.idle()
    if isinstance(action, PrepareAction):
        client.kin_data["actual_joint_pose"] = [0.0, -1.0, 0.0, 0.0, 0.0, 0.0]
        client.kin_data["target_joint_pose"] = [0.0, -1.0, 0.0, 0.0, 0.0, 0.0]
        original_movej = client.movej

        async def converging_prepare_movej(
            p: list[float],
            a: float,
            v: float,
            t: float,
            r: float,
        ) -> object:
            result = await original_movej(p, a, v, t, r)
            client.kin_data["actual_joint_pose"] = list(p)
            client.kin_data["target_joint_pose"] = list(p)
            return result

        client.movej = converging_prepare_movej  # type: ignore[method-assign]
    if isinstance(action, (TranslationAction, RotationAction)):
        requested_pose: dict[str, float] | None = None

        async def converging_ik(
            pose: dict[str, float],
            joints: list[float],
        ) -> object:
            nonlocal requested_pose
            requested_pose = dict(pose)
            client.read_calls.append("kinematics_inverse")
            client.ik_calls.append((dict(pose), list(joints)))
            return list(joints)

        original_move_pvat = client.move_pvat

        async def converging_move_pvat(
            p: list[float],
            v: list[float],
            a: list[float],
            t: float,
        ) -> object:
            result = await original_move_pvat(p, v, a, t)
            assert requested_pose is not None
            client.kin_data["actual_tcp_pose"] = dict(requested_pose)
            return result

        client.kinematics_inverse = converging_ik  # type: ignore[method-assign]
        client.move_pvat = converging_move_pvat  # type: ignore[method-assign]

    monkeypatch.setattr(
        "app.commissioning.smoke.COMMISSIONING_LOG_ROOT",
        tmp_path / "logs",
    )

    result = await run_smoke(
        SmokeOptions(
            config_path=_control_config(tmp_path),
            action=action,
            confirmation=REAL_ROBOT_CONFIRMATION,
        ),
        AsyncMock(return_value=client),
    )

    methods = {str(call[0]) for call in client.write_calls}
    assert result.action in {
        "translate",
        "rotate",
        "gripper",
        "prepare",
        "home",
        "stop",
    }
    assert result.stable is True
    assert methods == expected_methods
    assert not methods & {
        "start_sys",
        "set_tcp",
        "init_claw",
        "estop",
        "speedl",
        "stop_sys",
    }
    claw_calls = [call for call in client.write_calls if call[0] == "set_claw"]
    assert all(call[1] <= 30 for call in claw_calls)
    if action == GripperAction("close"):
        assert claw_calls == [("set_claw", 30, 0)]


@pytest.mark.asyncio
async def test_commissioning_motion_times_out_without_authoritative_progress(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    client = FakeLebaiClient.idle()
    monkeypatch.setattr(
        "app.commissioning.smoke.COMMISSIONING_LOG_ROOT",
        tmp_path / "logs",
    )
    monkeypatch.setattr(
        "app.commissioning.smoke.COMMISSIONING_MOTION_TIMEOUT_S",
        0.06,
        raising=False,
    )

    real_sleep = asyncio.sleep

    async def no_wait(_delay: float) -> None:
        await real_sleep(0)

    monkeypatch.setattr("app.commissioning.smoke.asyncio.sleep", no_wait)

    with pytest.raises(RuntimeError, match="^smoke_motion_timeout$"):
        await run_smoke(
            SmokeOptions(
                config_path=_control_config(tmp_path),
                action=TranslationAction("x", 0.005),
                confirmation=REAL_ROBOT_CONFIRMATION,
            ),
            AsyncMock(return_value=client),
        )

    methods = [call[0] for call in client.write_calls]
    assert "move_pvat" in methods
    assert "stop_move" in methods


@pytest.mark.asyncio
async def test_home_commissioning_can_leave_a_singular_pose(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    client = FakeLebaiClient.idle(q=[0.0, -1.0, 0.0, 0.0, 0.2, 0.0])
    original_movej = client.movej

    async def converging_movej(
        p: list[float],
        a: float,
        v: float,
        t: float,
        r: float,
    ) -> object:
        result = await original_movej(p, a, v, t, r)
        client.kin_data["actual_joint_pose"] = list(p)
        client.kin_data["target_joint_pose"] = list(p)
        return result

    client.movej = converging_movej  # type: ignore[method-assign]
    monkeypatch.setattr(
        "app.commissioning.smoke.COMMISSIONING_LOG_ROOT",
        tmp_path / "logs",
    )

    result = await run_smoke(
        SmokeOptions(
            config_path=_control_config(tmp_path),
            action=HomeAction(),
            confirmation=REAL_ROBOT_CONFIRMATION,
        ),
        AsyncMock(return_value=client),
    )

    assert result.action == "home"
    assert any(call[0] == "movej" for call in client.write_calls)


@pytest.mark.asyncio
async def test_commissioning_motion_stops_on_authoritative_overshoot(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    client = FakeLebaiClient.idle()
    original_move_pvat = client.move_pvat

    async def overshooting_move_pvat(
        p: list[float],
        v: list[float],
        a: list[float],
        t: float,
    ) -> object:
        result = await original_move_pvat(p, v, a, t)
        actual_tcp = dict(client.kin_data["actual_tcp_pose"])
        actual_tcp["x"] = 0.306
        client.kin_data["actual_tcp_pose"] = actual_tcp
        return result

    client.move_pvat = overshooting_move_pvat  # type: ignore[method-assign]
    monkeypatch.setattr(
        "app.commissioning.smoke.COMMISSIONING_LOG_ROOT",
        tmp_path / "logs",
    )

    real_sleep = asyncio.sleep

    async def no_wait(_delay: float) -> None:
        await real_sleep(0)

    monkeypatch.setattr("app.commissioning.smoke.asyncio.sleep", no_wait)

    with pytest.raises(RuntimeError, match="^smoke_motion_overshoot$"):
        await run_smoke(
            SmokeOptions(
                config_path=_control_config(tmp_path),
                action=TranslationAction("x", 0.005),
                confirmation=REAL_ROBOT_CONFIRMATION,
            ),
            AsyncMock(return_value=client),
        )

    methods = [call[0] for call in client.write_calls]
    stop_index = methods.index("stop_move")
    assert "move_pvat" not in methods[stop_index + 1 :]
