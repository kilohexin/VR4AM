# Task 1 — Offline soak rehearsal report

## Scope

- Branch: `codex/offline-rehearsal`
- Worktree: `D:\MyWork\VR4Arm\.worktrees\teleoperation-ux-recovery`
- Hardware and real Lebai SDK contact: none. The soak uses the existing virtual simulator only.

## TDD evidence

### Red: missing measurement and benchmark interfaces

Command:

```powershell
cd D:\MyWork\VR4Arm\.worktrees\teleoperation-ux-recovery\backend
python -m pytest tests/test_soak.py -q
```

Output:

```text
..FF..                                                                   [100%]
FAILED tests/test_soak.py::test_measure_soak_reports_elapsed_time_and_throughput
AttributeError: module 'soak_simulator' has no attribute 'measure_soak'
FAILED tests/test_soak.py::test_benchmark_separates_warning_from_failure
AttributeError: module 'soak_simulator' has no attribute 'benchmark_soak'
2 failed, 4 passed in 10.61s
```

### Red: missing isolated process interface

Command:

```powershell
cd D:\MyWork\VR4Arm\.worktrees\teleoperation-ux-recovery\backend
python -m pytest tests/scripts/test_accept_virtual_lm3.py -q
```

Output:

```text
...F..                                                                   [100%]
FAILED tests/scripts/test_accept_virtual_lm3.py::test_low_absolute_soak_throughput_warns_without_failing_gate
AttributeError: module 'accept_virtual_lm3_test_module' has no attribute 'run_full_soak_process'
1 failed, 5 passed in 0.62s
```

## Verification

### Focused Task 1 tests

Command:

```powershell
cd D:\MyWork\VR4Arm\.worktrees\teleoperation-ux-recovery\backend
python -m pytest tests/test_soak.py tests/scripts/test_accept_virtual_lm3.py -q
```

Output:

```text
.............                                                            [100%]
13 passed in 10.38s
```

### Full backend suite

Command:

```powershell
cd D:\MyWork\VR4Arm\.worktrees\teleoperation-ux-recovery\backend
python -m pytest -q --junitxml "$env:TEMP\vr4arm-task1-backend.xml"
```

Result read from the generated JUnit XML:

```text
tests=447 failures=0 errors=0 skipped=0 time=40.752
```

### Real isolated soak and warmed relative benchmark

Command:

```powershell
cd D:\MyWork\VR4Arm\.worktrees\teleoperation-ux-recovery
python -c "import json; from pathlib import Path; from scripts.accept_virtual_lm3 import run_full_soak_process; from scripts.soak_simulator import benchmark_soak; measurement=run_full_soak_process(Path('.').resolve()); print(json.dumps({'soak': {'control_steps': measurement.summary['control_steps'], 'injected_events': measurement.summary['injected_events'], 'error_count': measurement.summary['error_count'], 'duration_s': measurement.duration_s, 'steps_per_second': measurement.steps_per_second}, 'soak_performance': benchmark_soak()}, sort_keys=True))"
```

Output:

```json
{"soak": {"control_steps": 30000, "duration_s": 9.182796100009, "error_count": 0, "injected_events": 7, "steps_per_second": 3266.978780022198}, "soak_performance": {"long_steps_per_second": 3383.1788348265823, "long_to_short_ratio": 1.0794300471793685, "passed": true, "short_steps_per_second": 3134.227033671225, "warning": false}}
```

## Result

- The complete soak remains 30,000 control steps and retains all 7 scheduled safety events.
- The acceptance gate runs the full soak in an isolated subprocess with a 60-second timeout. Timeout, nonzero exit, malformed JSON, missing/invalid counts, and non-finite measured rates produce a failed gate report.
- The warmed 0.5/2.0-minute relative benchmark passes at `long_to_short_ratio >= 0.70`; absolute long-run throughput below 2,000 steps/s is reported as a warning, not a failure.
- The obsolete `<15.0` assertion is absent.
