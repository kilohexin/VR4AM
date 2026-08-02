from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
SOAK_PATH = ROOT / "scripts" / "soak_simulator.py"


def _load_soak_module():
    spec = importlib.util.spec_from_file_location("soak_simulator", SOAK_PATH)
    if spec is None or spec.loader is None:
        raise AssertionError("could not load soak runner")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
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


def test_ten_minute_soak_preserves_every_safety_invariant() -> None:
    summary = _load_soak_module().run_soak(minutes=10, seed=42)

    _assert_safety_invariants(summary)


def test_short_soak_is_deterministic_and_seeded() -> None:
    run_soak = _load_soak_module().run_soak

    assert run_soak(minutes=0.2, seed=42) == run_soak(minutes=0.2, seed=42)
    assert (
        run_soak(minutes=0.2, seed=42)["path_checksum"]
        != run_soak(minutes=0.2, seed=43)["path_checksum"]
    )


def test_measure_soak_reports_elapsed_time_and_throughput() -> None:
    module = _load_soak_module()
    clock_values = iter((10.0, 12.0))

    def fake_counter() -> float:
        return next(clock_values)

    measurement = module.measure_soak(
        0.2,
        42,
        runner=lambda minutes, seed: {"control_steps": 600},
        perf_counter=fake_counter,
    )

    assert isinstance(measurement, module.SoakMeasurement)
    assert measurement.summary == {"control_steps": 600}
    assert measurement.duration_s == 2.0
    assert measurement.steps_per_second == 300.0


def test_benchmark_separates_warning_from_failure() -> None:
    module = _load_soak_module()
    calls: list[tuple[float, int]] = []
    clock_values = iter((0.0, 0.6, 0.6, 3.933333333333333))

    def fake_counter() -> float:
        return next(clock_values)

    def fake_runner(minutes: float, seed: int) -> dict[str, object]:
        calls.append((minutes, seed))
        return {"control_steps": round(minutes * 3000), "error_count": 0}

    benchmark = module.benchmark_soak(
        runner=fake_runner,
        perf_counter=fake_counter,
    )

    assert calls == [(0.1, 42), (0.5, 42), (2.0, 42)]
    assert benchmark["long_to_short_ratio"] >= 0.70
    assert benchmark["passed"] is True
    assert benchmark["warning"] is True


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
