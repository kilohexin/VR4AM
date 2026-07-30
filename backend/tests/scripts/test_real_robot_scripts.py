from __future__ import annotations

import asyncio
import json
from pathlib import Path
from unittest.mock import AsyncMock

import pytest

from app.commissioning.preflight import run_preflight
from app.commissioning.smoke import parse_smoke_args, run_smoke
from tests.config.test_real_robot_config import REAL_CONFIG_TEMPLATE
from tests.robots.fake_lebai import FakeLebaiClient


def _config(tmp_path: Path, mode: str) -> Path:
    path = tmp_path / f"{mode}.yaml"
    path.write_text(
        REAL_CONFIG_TEMPLATE.format(mode=mode),
        encoding="utf-8",
    )
    return path


@pytest.mark.asyncio
async def test_preflight_script_never_arms_or_writes(tmp_path: Path) -> None:
    client = FakeLebaiClient.idle()
    output = tmp_path / "report.json"

    result = await run_preflight(
        config_path=_config(tmp_path, "readonly"),
        output_path=output,
        client_factory=AsyncMock(return_value=client),
    )

    report = json.loads(output.read_text(encoding="utf-8"))
    assert result == 0
    assert report["complete"] is True
    assert report["preflight_reason"] == "real_robot_readonly"
    assert isinstance(report["backend_version"], str)
    assert "lebai_sdk_version" in report
    assert len(report["actual_q"]) == 6
    assert report["actual_tcp"]["p"] == pytest.approx([0.3, 0.0, 0.4])
    assert client.write_calls == []


@pytest.mark.asyncio
async def test_preflight_requires_readonly_config_before_connect(
    tmp_path: Path,
) -> None:
    factory = AsyncMock()

    with pytest.raises(RuntimeError, match="^preflight_requires_readonly_mode$"):
        await run_preflight(
            config_path=_config(tmp_path, "control"),
            output_path=tmp_path / "report.json",
            client_factory=factory,
        )

    factory.assert_not_awaited()


def test_smoke_rejects_translation_over_five_mm(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="^smoke_translation_out_of_bounds$"):
        parse_smoke_args(
            [
                "translate",
                "--config",
                str(_config(tmp_path, "control")),
                "--axis",
                "x",
                "--distance-m",
                "0.006",
                "--confirm",
                "I_UNDERSTAND_REAL_ROBOT_MOTION",
            ]
        )


def test_smoke_rejects_wrong_confirmation(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="^smoke_confirmation_required$"):
        parse_smoke_args(
            [
                "translate",
                "--config",
                str(_config(tmp_path, "control")),
                "--axis",
                "z",
                "--distance-m",
                "0.005",
                "--confirm",
                "yes",
            ]
        )


@pytest.mark.asyncio
async def test_smoke_rejects_readonly_mode_without_connecting(
    tmp_path: Path,
) -> None:
    options = parse_smoke_args(
        [
            "translate",
            "--config",
            str(_config(tmp_path, "readonly")),
            "--axis",
            "x",
            "--distance-m",
            "0.005",
            "--confirm",
            "I_UNDERSTAND_REAL_ROBOT_MOTION",
        ]
    )
    factory = AsyncMock()

    with pytest.raises(RuntimeError, match="^smoke_requires_control_mode$"):
        await run_smoke(options, factory)

    factory.assert_not_awaited()


@pytest.mark.asyncio
async def test_smoke_rejects_missing_home_before_connecting(
    tmp_path: Path,
) -> None:
    path = _config(tmp_path, "control")
    path.write_text(
        path.read_text(encoding="utf-8").replace(
            "home_q: [0, -1.0, 1.0, 0, 1.57, 0]",
            "home_q: []",
        ),
        encoding="utf-8",
    )
    options = parse_smoke_args(
        [
            "translate",
            "--config",
            str(path),
            "--axis",
            "y",
            "--distance-m",
            "0.005",
            "--confirm",
            "I_UNDERSTAND_REAL_ROBOT_MOTION",
        ]
    )
    factory = AsyncMock()

    with pytest.raises(RuntimeError, match="^invalid_config:home_q$"):
        await run_smoke(options, factory)

    factory.assert_not_awaited()


@pytest.mark.asyncio
async def test_smoke_rejects_missing_tcp_before_connecting(
    tmp_path: Path,
) -> None:
    path = _config(tmp_path, "control")
    text = path.read_text(encoding="utf-8")
    start = text.index("  expected_tcp:")
    end = text.index("  home_q:")
    path.write_text(
        text[:start] + "  expected_tcp: {}\n" + text[end:],
        encoding="utf-8",
    )
    options = parse_smoke_args(
        [
            "translate",
            "--config",
            str(path),
            "--axis",
            "x",
            "--distance-m",
            "0.005",
            "--confirm",
            "I_UNDERSTAND_REAL_ROBOT_MOTION",
        ]
    )
    factory = AsyncMock()

    with pytest.raises(RuntimeError, match="^invalid_config:x$"):
        await run_smoke(options, factory)

    factory.assert_not_awaited()


@pytest.mark.asyncio
async def test_smoke_rejects_non_idle_robot_before_any_write(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    client = FakeLebaiClient.idle()
    client.robot_state = "MOVING"
    options = parse_smoke_args(
        [
            "translate",
            "--config",
            str(_config(tmp_path, "control")),
            "--axis",
            "x",
            "--distance-m",
            "0.005",
            "--confirm",
            "I_UNDERSTAND_REAL_ROBOT_MOTION",
        ]
    )
    monkeypatch.setattr(
        "app.commissioning.smoke.COMMISSIONING_LOG_ROOT",
        tmp_path / "logs",
        raising=False,
    )

    with pytest.raises(
        RuntimeError,
        match="^smoke_preflight_failed:robot_not_idle$",
    ):
        await run_smoke(options, AsyncMock(return_value=client))

    assert client.write_calls == []


@pytest.mark.asyncio
async def test_smoke_runs_until_authoritative_target_and_verified_stop(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    client = FakeLebaiClient.idle()
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
        actual_pose = dict(requested_pose)
        actual_pose["z"] = min(float(actual_pose["z"]), 0.405)
        client.kin_data["actual_tcp_pose"] = actual_pose
        return result

    client.kinematics_inverse = converging_ik  # type: ignore[method-assign]
    client.move_pvat = converging_move_pvat  # type: ignore[method-assign]
    real_sleep = asyncio.sleep

    async def no_wait(_delay: float) -> None:
        await real_sleep(0)

    monkeypatch.setattr("app.commissioning.smoke.asyncio.sleep", no_wait)
    options = parse_smoke_args(
        [
            "translate",
            "--config",
            str(_config(tmp_path, "control")),
            "--axis",
            "z",
            "--distance-m",
            "0.005",
            "--confirm",
            "I_UNDERSTAND_REAL_ROBOT_MOTION",
        ]
    )
    monkeypatch.setattr(
        "app.commissioning.smoke.COMMISSIONING_LOG_ROOT",
        tmp_path / "logs",
        raising=False,
    )

    result = await run_smoke(options, AsyncMock(return_value=client))

    methods = [call[0] for call in client.write_calls]
    assert result.action == "translate"
    assert result.stable is True
    assert result.after.robot_state.value == "IDLE"
    assert "move_pvat" in methods
    assert "stop_move" in methods
