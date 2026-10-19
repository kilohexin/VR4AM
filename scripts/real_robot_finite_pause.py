"""Preview by default: separately authorized one +Z50um finite window/pause.

This entry does not authorize a live test. Each attempt needs a new matching
preview/hash/confirmation, ten-minute expiry and current onsite approval.
Pause may retain a trajectory. Never resume or replace it with stop_move.
"""
import argparse
import asyncio
import importlib.util
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'backend'))
spec = importlib.util.spec_from_file_location('finite_pause_scope_helpers',
    Path(__file__).with_name('real_robot_finite_window.py'))
base = importlib.util.module_from_spec(spec)
spec.loader.exec_module(base)
sha = base.sha
CONFIRM = 'I_APPROVE_ONE_FINITE_Z50UM_WINDOW_AND_ONE_RESERVED_PAUSE_NO_RESUME'
PROFILE = dict(parallel_fk_audit=True, parallel_motion_reads=True, prelease_feedback_fk=True,
               qpc_timing=True, observe_motion=False, ready_joint_tolerance_rad=.001)


def runtime_hashes():
    values = base.runtime_hashes()
    path = Path(__file__).resolve()
    values[path.relative_to(ROOT).as_posix()] = sha(path)
    return values


def existing_rpc_connections():
    return base.existing_rpc_connections()


def validate_plan(plan):
    from app.commissioning.finite_pause_window import validate_settings
    validate_settings(plan['settings'])
    if any(type(plan.get(key)) is not type(value) or plan.get(key) != value
           for key, value in PROFILE.items()):
        raise ValueError('finite_pause_profile_changed')


def prepare(config, output, *, expires=None):
    plan = base.prepare(config, output, expires=expires, **PROFILE)
    validate_plan(plan)
    plan['runtime_hashes'] = runtime_hashes()
    return plan


def preview(plan):
    validate_plan(plan)
    scope = base.preview(plan)
    reservation = scope.pop('cleanup_stop_reservation_s')
    scope.update(scope='one_finite_z50um_window_and_reserved_cleanup_pause',
        allowed_writes=['move_pvat', 'pause_move'], max_stop_entries=0, max_pause_entries=1,
        max_reference_s=.08, termination_method='pause_move',
        cleanup_stop_on_motion_revocation=False, cleanup_stop_on_math_failure=False,
        cleanup_pause_on_motion_revocation=True, cleanup_pause_on_math_failure=True,
        cleanup_pause_reservation_s=reservation, paused_motion_may_be_retained=True,
        requires_finished_task_when_paused=True, requires_all_tail_samples_settled_for_complete=True,
        stop_move_fallback=False, required_confirmation=CONFIRM)
    return scope


class Inputs(base.Inputs):
    def confirmation(self):
        return CONFIRM

    def scope_preview(self):
        return preview(self.plan)

    def current_runtime_hashes(self):
        return runtime_hashes()


async def execute(plan, *, approved, expected_hash, expires, confirm, client_factory=None):
    from app.commissioning.finite_pause_window import run_pause_window
    validate_plan(plan)
    inputs = Inputs(plan, approved, expected_hash, expires, confirm)
    return await base._execute_scope(plan, inputs, run_pause_window, client_factory=client_factory,
        rpc_check=existing_rpc_connections, workflow='single_finite_pause_window',
        live_runtime='LIVE_FINITE_PAUSE_DIAGNOSTIC')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--execute-once', action='store_true')
    parser.add_argument('--approved-preview', type=Path)
    parser.add_argument('--approved-preview-sha256')
    parser.add_argument('--authorization-expires-utc')
    parser.add_argument('--confirm')
    args = parser.parse_args(argv)
    plan = prepare(args.config, args.output, expires=args.authorization_expires_utc)
    if not args.execute_once:
        print(json.dumps(preview(plan), indent=2))
        return 0
    if Path(sys.executable).resolve() != (ROOT / '.venv/Scripts/python.exe').resolve():
        raise ValueError('repository_venv_required')
    from app.commissioning.qpc_event_loop import new_qpc_loop
    loop = new_qpc_loop()
    task = loop.create_task(execute(plan, approved=args.approved_preview,
        expected_hash=args.approved_preview_sha256, expires=args.authorization_expires_utc, confirm=args.confirm))
    try:
        result = loop.run_until_complete(task)
        print(json.dumps({k: result[k] for k in ('complete', 'pvat_entries', 'pause_entries',
            'stop_entries', 'latched_fault', 'hardware_verified')}))
        return 0 if result['complete'] else 2
    except KeyboardInterrupt:
        task.cancel()
        try:
            loop.run_until_complete(task)
        except asyncio.CancelledError:
            pass
        return 130
    finally:
        loop.close()


if __name__ == '__main__':
    raise SystemExit(main())
