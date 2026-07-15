from __future__ import annotations

import importlib.util
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SOAK_PATH = ROOT / "scripts" / "soak_simulator.py"


def _load_run_soak():
    spec = importlib.util.spec_from_file_location("soak_simulator", SOAK_PATH)
    if spec is None or spec.loader is None:
        raise AssertionError("could not load soak runner")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.run_soak


def _assert_safety_invariants(summary: dict[str, object]) -> None:
    assert summary["simulated_minutes"] == 10.0
    assert summary["nan_count"] == 0
    assert summary["max_queue_depth"] == 1
    assert summary["error_count"] == 0
    assert summary["injected_events"] == summary["verified_injected_stops"]
    assert summary["injected_by_kind"] == {
        "tracking_loss": 1,
        "hidden": 1,
        "disconnect": 1,
        "command_fault": 1,
        "safety_fault": 1,
        "visible_blurred": 1,
    }
    assert summary["final_mode"] == "DISARMED"


def test_ten_minute_soak_is_fast_deterministic_and_seeded() -> None:
    run_soak = _load_run_soak()

    started = time.perf_counter()
    first = run_soak(minutes=10, seed=42)
    elapsed = time.perf_counter() - started
    repeated = run_soak(minutes=10, seed=42)
    different = run_soak(minutes=10, seed=43)

    assert elapsed < 15.0
    assert first == repeated
    assert first["path_checksum"] != different["path_checksum"]
    _assert_safety_invariants(first)
    _assert_safety_invariants(repeated)
    _assert_safety_invariants(different)
