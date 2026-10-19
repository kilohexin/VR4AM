"""Preview by default: single +Z50um/pause with read-start-period sampling.

Requires its own matching authorization; old pause/stop previews cannot be used.
Pause may retain a trajectory. Never resume, add a target or retry automatically.
"""
import argparse
import asyncio
import importlib.util
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'backend'))
spec = importlib.util.spec_from_file_location('finite_sampling_pause_helpers',
    Path(__file__).with_name('real_robot_finite_pause.py'))
prior = importlib.util.module_from_spec(spec)
spec.loader.exec_module(prior)
base = prior.base
sha = base.sha
CONFIRM = 'I_APPROVE_ONE_FINITE_Z50UM_WINDOW_AND_ONE_START_PERIOD_PAUSE_NO_RESUME'
PROFILE = dict(prior.PROFILE)


def runtime_hashes():
    values = prior.runtime_hashes()
    path = Path(__file__).resolve()
    values[path.relative_to(ROOT).as_posix()] = sha(path)
    return values


def existing_rpc_connections():
    return base.existing_rpc_connections()


def validate_plan(plan):
    prior.validate_plan(plan)
    if plan.get('stop_sampling_policy') != 'start_period_20ms':
        raise ValueError('finite_pause_sampling_profile_changed')


def prepare(config, output, *, expires=None):
    plan = base.prepare(config, output, expires=expires, **PROFILE)
    plan['stop_sampling_policy'] = 'start_period_20ms'
    validate_plan(plan)
    plan['runtime_hashes'] = runtime_hashes()
    return plan


def preview(plan):
    validate_plan(plan)
    scope = prior.preview(plan)
    scope.update(scope='one_finite_z50um_window_and_reserved_start_period_pause',
        stop_sampling_policy='start_period_20ms', minimum_stop_read_start_period_ms=20,
        required_confirmation=CONFIRM)
    return scope


class Inputs(base.Inputs):
    def confirmation(self):
        return CONFIRM

    def scope_preview(self):
        return preview(self.plan)

    def current_runtime_hashes(self):
        return runtime_hashes()


async def execute(plan, *, approved, expected_hash, expires, confirm, client_factory=None):
    from app.commissioning.finite_pause_sampling import run_sampling_pause_window
    validate_plan(plan)
    inputs = Inputs(plan, approved, expected_hash, expires, confirm)
    return await base._execute_scope(plan, inputs, run_sampling_pause_window, client_factory=client_factory,
        rpc_check=existing_rpc_connections, workflow='single_finite_sampling_pause_window',
        live_runtime='LIVE_FINITE_SAMPLING_PAUSE_DIAGNOSTIC')


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
        print(json.dumps({k: result[k] for k in ('complete', 'stop_sampling_policy', 'pvat_entries',
            'pause_entries', 'stop_entries', 'latched_fault', 'hardware_verified')}))
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
