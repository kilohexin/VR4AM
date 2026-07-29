from __future__ import annotations

import math
from pathlib import Path

import pytest
from unittest.mock import AsyncMock

from app.commissioning.actions import (
    GripperAction,
    HomeAction,
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


def _control_config(tmp_path: Path) -> Path:
    path = tmp_path / "control.yaml"
    path.write_text(
        REAL_CONFIG_TEMPLATE.format(mode="control"), encoding="utf-8"
    )
    return path


@pytest.mark.asyncio
@pytest.mark.parametrize(
    ("action", "expected_methods"),
    [
        (TranslationAction("x", 0.005), {"move_pvat", "stop_move"}),
        (RotationAction("yaw", 2.0), {"move_pvat", "stop_move"}),
        (GripperAction("close"), {"set_claw", "stop_move"}),
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
    assert result.action in {"translate", "rotate", "gripper", "home", "stop"}
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
