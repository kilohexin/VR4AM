# Tool Frame and IK Continuity Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make controller rotation follow the stereo-camera tool frame and prevent stale or intermediate IK states from producing persistent joint-boundary warnings.

**Architecture:** Keep translation in the captured user/head basis, but express rotation in the controller anchor's local frame and conjugate it through a fixed controller-to-tool calibration. Preserve target-seeded IK during continuous motion, synchronize the seed from actual simulator joints after stops, and project intermediate IK candidates into the configured joint window.

**Tech Stack:** Python 3.11+, NumPy, SciPy Rotation, FastAPI simulator backend.

## Global Constraints

- Keep `backend: simulator`; do not enable or contact a real robot.
- Do not add physics, collision, Cannon-es, or dependencies.
- Preserve the camera/time-sync and raw VR-pose protocol interfaces.
- Per the user's explicit instruction, do not create, edit, or run automated tests; do not run builds, type checks, or the application.

---

### Task 1: Camera-tool local rotation mapping

**Files:**
- Modify: `backend/app/control/coordinate_mapper.py`

**Interfaces:**
- Produces: `DEFAULT_R_TH`, a proper rotation mapping controller-local vectors to tool-local vectors.
- Preserves: `CoordinateMapper.capture(...)` and `CoordinateMapper.target(...)` public signatures.

- [x] **Step 1:** Define the default `diag(-1, 1, -1)` controller-to-tool calibration and validate it as a proper rotation in `CoordinateMapper.__init__`.
- [x] **Step 2:** Compute relative rotation as `R_hand_anchor.T @ R_hand_now`, conjugate through the tool calibration, apply the existing dead zone/scale, and right-multiply it onto the captured TCP orientation.
- [x] **Step 3:** Review the formulas without executing code and confirm identical anchor poses produce the captured TCP pose exactly.

### Task 2: Actual-joint IK synchronization and accurate boundary classification

**Files:**
- Modify: `backend/app/robots/sim_adapter.py`
- Modify: `backend/app/sim/ik.py`

**Interfaces:**
- Preserves: `SimRobotAdapter.command_tcp(...)`, `stop(...)`, and `solve_ik(...)` signatures.

- [x] **Step 1:** Synchronize `_ik_seed_q` from `robot.q` in `stop()` and immediately before solving whenever `robot.target_q` is absent.
- [x] **Step 2:** Replace immediate intermediate-candidate failure with projection into the joint window and remember whether projection occurred.
- [x] **Step 3:** After non-convergence, report `joint_safety_window` only when projection occurred and the final candidate remains at a joint boundary; otherwise report `ik_unreachable`.
- [x] **Step 4:** Review exception paths and ensure neither failure mutates the active robot target or command ID.

### Task 3: Operator documentation and handoff

**Files:**
- Modify: `README.md`

- [x] **Step 1:** Document that rotation follows the stereo-camera optical tool frame and Grip re-capture resets the simulator IK reference to actual joints.
- [x] **Step 2:** Inspect only the final diff for real-robot enablement, dependency changes, unrelated files, and accidental test edits.
- [x] **Step 3:** Hand off the focused Quest manual acceptance sequence and explicitly state that no tests, build, type-check, or launch were performed.
