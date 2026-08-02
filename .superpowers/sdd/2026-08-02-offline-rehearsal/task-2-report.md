# Task 2 report: Fake-only rehearsal reports

## Scope

Implemented `backend/app/rehearsal` and its focused tests. The store accepts
only `LEBAI_FAKE`, records server-owned state and diagnostics snapshots under
an owner-identity lock, enforces declared phase order, atomically writes JSON
and Markdown evidence, and always emits `hardware_verified: false` together
with the outstanding hardware checks.

## TDD evidence

1. `cd backend; python -m pytest tests/rehearsal/test_report.py -q`
   - Before implementation: exit 2, collection failed as expected with
     `ModuleNotFoundError: No module named 'app.rehearsal'`.
2. `cd backend; python -m pytest tests/rehearsal/test_report.py -q`
   - After implementation: exit 0, `9 passed in 0.07s`.
3. `cd backend; python -m pytest -q`
   - Backend regression: exit 0. Pytest completed all progress groups; this
     environment did not print its usual final summary line.
4. `git diff --check`
   - Exit 0; no whitespace errors.

## Caveats

These are offline, Fake-runtime reports only. They are not evidence of SDK,
robot, safety-stop, gripper, or physical-motion verification; the eight
hardware-pending checks remain explicit in every report.

## Review-fix evidence

The independent review found two P1 false-green/partial-publication cases.
Both were addressed with new focused regression tests.

1. `cd backend; python -m pytest tests/rehearsal/test_report.py -q`
   - Red phase: exit 1, `3 failed, 9 passed`. The exact contradictory case
     (`status: failed` plus a phase failure for all twelve phases followed by
     `outcome: passed`) did not raise; the two transaction tests also failed
     because the staging/publish boundary did not yet exist.
2. `cd backend; python -m pytest tests/rehearsal/test_report.py -q`
   - Green phase: exit 0, `12 passed in 0.10s`. A passed outcome now requires
     each phase to be passed with no recorded failure. Markdown-first/JSON-last
     staged publication removes temporary and final artifacts on either a
     Markdown staging failure or JSON final-marker publication failure, while
     preserving the active run for a successful retry.
3. `cd backend; python -m pytest -q`
   - Full backend regression: exit 0. Pytest completed all progress groups;
     this environment again omitted the usual final summary line.
