from __future__ import annotations

import importlib.util
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
SOAK_PATH = ROOT / "scripts" / "soak_simulator.py"


def _load_soak_module():
    spec = importlib.util.spec_from_file_location("soak_simulator", SOAK_PATH)
    if spec is None or spec.loader is None:
        raise AssertionError("could not load soak runner")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _assert_safety_invariants(summary: dict[str, object]) -> None:
    assert summary["simulated_minutes"] == 10.0
    assert summary["control_steps"] == 30_000
    assert summary["virtual_steps"] == 30_000
    assert summary["controller_frame_hz"] == 10.0
    assert summary["nan_count"] == 0
    assert summary["max_queue_depth"] == 1
    assert summary["error_count"] == 0
    assert summary["injected_events"] == summary["verified_injected_events"]
    assert summary["injected_events"] == 7
    assert summary["verified_injected_events"] == 7
    assert summary["stop_expected_events"] == 5
    assert summary["verified_injected_stops"] == 5
    assert summary["injected_by_kind"] == {
        "tracking_loss": 1,
        "hidden": 1,
        "disconnect": 1,
        "command_fault": 1,
        "ik_boundary": 1,
        "workspace_boundary": 1,
        "visible_blurred": 1,
    }
    assert summary["final_mode"] == "DISARMED"


def test_ten_minute_soak_is_fast_deterministic_and_seeded() -> None:
    run_soak = _load_soak_module().run_soak

    started = time.perf_counter()
    first = run_soak(minutes=10, seed=42)
    elapsed = time.perf_counter() - started
    repeated = run_soak(minutes=10, seed=42)
    different = run_soak(minutes=10, seed=43)

    assert elapsed < 15.0
    assert first == repeated
    assert first["path_checksum"] != different["path_checksum"]
    assert first["frames"] < first["control_steps"]
    _assert_safety_invariants(first)
    _assert_safety_invariants(repeated)
    _assert_safety_invariants(different)


def test_soak_samples_the_storage_depth_observable(monkeypatch: pytest.MonkeyPatch) -> None:
    module = _load_soak_module()

    class FutureQueuedVRFrames(module.LatestVRFrame):
        @property
        def depth(self) -> int:
            return 0 if self.snapshot() is None else 2

    monkeypatch.setattr(module, "LatestVRFrame", FutureQueuedVRFrames)

    summary = module.run_soak(minutes=0.2, seed=42)

    assert summary["max_queue_depth"] == 2
    assert summary["error_count"] > 0


class FakeWallClock:
    def __init__(self) -> None:
        self.now = 100.0
        self.sleeps: list[float] = []

    def monotonic(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds

    async def sleep(self, seconds: float) -> None:
        self.sleeps.append(seconds)
        self.now += seconds


@pytest.mark.asyncio
async def test_realtime_pacer_uses_absolute_deadlines_without_processing_drift() -> None:
    module = _load_soak_module()
    wall_clock = FakeWallClock()
    pacer = module.AbsoluteDeadlinePacer(
        period_seconds=0.02,
        monotonic=wall_clock.monotonic,
        sleep=wall_clock.sleep,
    )

    wall_clock.advance(0.006)
    await pacer.wait_next()
    wall_clock.advance(0.006)
    await pacer.wait_next()
    wall_clock.advance(0.030)
    await pacer.wait_next()
    wall_clock.advance(0.002)
    await pacer.wait_next()

    assert wall_clock.sleeps == pytest.approx([0.014, 0.014, 0.0, 0.008])
