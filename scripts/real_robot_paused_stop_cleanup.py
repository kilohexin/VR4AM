"""Offline preview by default; separately approved PAUSED one-stop cleanup."""
import argparse
import asyncio
import copy
from datetime import UTC, datetime, timedelta
import hashlib
import importlib
import importlib.util
import json
import math
from pathlib import Path
import re
import sys
from time import monotonic_ns

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'backend'))
CONFIRM = 'I_APPROVE_ONE_PAUSED_FINISHED_ZERO_SPEED_STOP_MOVE_NO_RESUME'


def load_local(name, filename):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).with_name(filename))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


idle = load_local('paused_idle_helpers', 'real_robot_idle_stop.py')
readonly = load_local('paused_read_helpers', 'real_robot_readonly_kinematics.py')
sha = idle.sha


def runtime_hashes():
    value = readonly.runtime_hashes()
    for path in (Path(__file__).resolve(), ROOT / 'scripts/real_robot_idle_stop.py'):
        value[path.relative_to(ROOT).as_posix()] = sha(path)
    return value


def existing_rpc_connections():
    return readonly.existing_rpc_connections()


def prepare(config, output, *, expires=None):
    plan = idle.prepare(config, output, expires=expires)
    for key, cap in dict(max_joint_speed_radps=.05, max_joint_acceleration_radps2=.2, pvat_horizon_s=.08).items():
        value = getattr(plan['settings'].control, key)
        if not math.isfinite(value) or not 0 < value <= cap:
            raise ValueError('paused_cleanup_cap_invalid:' + key)
    plan['runtime_hashes'] = runtime_hashes()
    return plan


def preview(plan):
    from app.commissioning.idle_stop_baseline import StopOnlyClient
    from app.commissioning.paused_stop_cleanup import READ_TIMEOUT_S, READ_BUDGET_S, MAX_READ_ENTRIES
    return dict(schema=1, scope='paused_finished_zero_speed_one_stop_move', target_ip=plan['settings'].ip,
        output=str(plan['output']), config_sha256=plan['config_sha256'], runtime_hashes=plan['runtime_hashes'],
        authorization_expires_utc=plan['authorization_expires_utc'], allowed_writes=['stop_move'],
        allowed_queries=sorted(StopOnlyClient.READS), max_connections=1, max_sdk_stop_entries=1,
        requires_raw_paused=True, requires_reported_zero_velocity=True, requires_motion_finished=True,
        sample_max_age_ms=100, stop_rpc_timeout_ms=200, stable_window_ms=300, verification_timeout_ms=500,
        read_timeout_ms=READ_TIMEOUT_S * 1000, read_budget_s=READ_BUDGET_S, max_read_entries=MAX_READ_ENTRIES,
        readonly_tail_s=2., requires_all_tail_samples_idle_for_complete=True, automatic_movement=False,
        automatic_resume=False, automatic_reset=False, automatic_home=False, automatic_retry=False,
        packet_capture=False, motion_authorized=False, hardware_stop_verified=False,
        runtime_checks='full_sha_before_fresh_snapshot_metadata_at_stop_for_already_loaded_code',
        runtime_metadata_is_not_new_byte_hash=True,
        requires_fresh_onsite_authorization=True, native_connection_release='standalone_process_exit_required_not_verified',
        success_means='static_cleanup_evidence_not_motion_stop_or_vr_acceptance')


def check_scope(plan, *, approved, expected_hash, expires, confirm):
    if confirm != CONFIRM:
        raise ValueError('paused_cleanup_scope_confirmation_required')
    try:
        expiry = datetime.fromisoformat(expires.replace('Z', '+00:00'))
        now = datetime.now(UTC)
        if (expiry.tzinfo is None or expiry.utcoffset() != timedelta(0)
                or not now < expiry <= now + timedelta(minutes=10) or expires != plan['authorization_expires_utc']):
            raise ValueError
    except (ValueError, TypeError, AttributeError):
        raise ValueError('paused_cleanup_expiry_invalid_or_expired') from None
    if approved is None or type(expected_hash) is not str or re.fullmatch('[a-f0-9]{64}', expected_hash) is None:
        raise ValueError('paused_cleanup_preview_hash_required')
    try:
        raw = Path(approved).read_bytes()
        value = json.loads(raw)
    except (OSError, ValueError):
        raise ValueError('paused_cleanup_preview_unavailable') from None
    if hashlib.sha256(raw).hexdigest() != expected_hash or value != preview(plan):
        raise ValueError('paused_cleanup_scope_or_hash_changed')
    return expiry


def check(plan, *, approved, expected_hash, expires, confirm):
    expiry = check_scope(plan, approved=approved, expected_hash=expected_hash, expires=expires, confirm=confirm)
    if sha(plan['config']) != plan['config_sha256'] or runtime_hashes() != plan['runtime_hashes']:
        raise ValueError('paused_cleanup_inputs_changed')
    if datetime.now(UTC) >= expiry:
        raise ValueError('paused_cleanup_authorization_expired_during_checks')


def runtime_metadata(paths):
    try:
        values = []
        for path in paths:
            stat = path.stat()
            values.append((stat.st_size, stat.st_mtime_ns, stat.st_ctime_ns, stat.st_dev, stat.st_ino))
        return tuple(values)
    except OSError:
        raise ValueError('paused_cleanup_runtime_metadata_unavailable') from None


async def execute(plan, *, approved, expected_hash, expires, confirm, client_factory=None):
    from app.commissioning.paused_stop_cleanup import run_cleanup
    from app.commissioning.readonly_kinematics import _Queries, new_report
    from app.recording.commissioning import CommissioningRecorder

    def admit():
        nonlocal paths, fingerprints
        if paths is None:
            try:
                candidates = tuple((ROOT / name).resolve(strict=True) for name in plan['runtime_hashes'])
            except OSError:
                raise ValueError('paused_cleanup_runtime_metadata_unavailable') from None
            if not all(path.is_relative_to(ROOT.resolve()) for path in candidates):
                raise ValueError('paused_cleanup_runtime_path_invalid')
            paths = candidates
        before = runtime_metadata(paths)
        if fingerprints is not None and before != fingerprints:
            raise ValueError('paused_cleanup_loaded_runtime_metadata_changed')
        check(plan, approved=approved, expected_hash=expected_hash, expires=expires, confirm=confirm)
        read_admit()
        after = runtime_metadata(paths)
        if before != after:
            raise ValueError('paused_cleanup_runtime_changed_during_hashes')
        read_admit()  # Expiry/config must survive the final metadata sample too.
        fingerprints = after

    def read_admit():
        # Full SHA stays before fresh reads; native stop uses the pinned identity.
        expiry = datetime.fromisoformat(expires.replace('Z', '+00:00'))
        if datetime.now(UTC) >= expiry:
            raise ValueError('paused_cleanup_authorization_expired')
        if sha(plan['config']) != plan['config_sha256']:
            raise ValueError('paused_cleanup_configuration_changed')
        if copied is not None and sha(copied) != plan['config_sha256']:
            raise ValueError('paused_cleanup_copied_configuration_changed')
        if datetime.now(UTC) >= expiry:
            raise ValueError('paused_cleanup_authorization_expired_during_config_checks')

    def write_admit():
        check_scope(plan, approved=approved, expected_hash=expected_hash, expires=expires, confirm=confirm)
        read_admit()
        if paths is None or fingerprints is None or runtime_metadata(paths) != fingerprints:
            raise ValueError('paused_cleanup_loaded_runtime_metadata_changed')
        # Authorization/config must still be valid after the metadata check.
        check_scope(plan, approved=approved, expected_hash=expected_hash, expires=expires, confirm=confirm)
        read_admit()

    copied = None
    paths = fingerprints = None
    admit()
    if existing_rpc_connections() != 0:
        raise ValueError('existing_rpc_connections')
    output = plan['output']
    output.mkdir(parents=True, exist_ok=False)
    (output / 'admission.json').write_text(json.dumps(dict(scope=preview(plan), approved_preview_sha256=expected_hash,
        started_utc=datetime.now(UTC).isoformat()), indent=2), encoding='utf-8')
    copied = output / 'config-control.yaml'
    copied.write_bytes(plan['config'].read_bytes())
    connector = new_report(simulation=client_factory is not None)
    calls = _Queries(admit, connector)
    # Begins before recorder/adapter setup, hence never renews the adapter's 3s connection wait.
    calls.deadline = monotonic_ns() + 3_000_000_000
    connection_used = False
    report = dict(runtime='SIMULATION' if client_factory is not None else 'LIVE_STATIC_DIAGNOSTIC')
    recorder = CommissioningRecorder(output / 'sessions', metadata=dict(workflow='paused_stop_cleanup', **report))

    async def factory(ip):
        nonlocal connection_used
        calls.guard()
        if connection_used or ip != plan['settings'].ip or existing_rpc_connections() != 0:
            raise ValueError('paused_cleanup_connection_not_admitted')
        connection_used = True

        async def connect():
            calls.guard()
            if client_factory is not None:
                return await client_factory(ip)
            readonly.verify_sdk_source()
            calls.guard()
            module = importlib.import_module('lebai_sdk')
            calls.guard()
            connector['native_sdk_init_entered'] = True
            module.init()
            calls.guard()
            connector['native_sdk_connect_entered'] = True
            return await module.connect(ip, False)
        return await calls.call('connect', connect)

    try:
        await run_cleanup(plan['settings'], recorder, factory, admit, report=report,
                          read_authorization_check=read_admit, stop_authorization_check=write_admit)
    except asyncio.CancelledError:
        report.update(complete=False, interrupted=True)
        raise
    finally:
        calls.closed = True
        connector['pending_remote_result_unknown'] = any(c['remote_result_unknown'] for c in connector['calls'])
        report['connection_diagnostics'] = copy.deepcopy(connector)
        (output / 'result.json').write_text(json.dumps(report, allow_nan=False, indent=2) + '\n', encoding='utf-8')
    return report


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
    loop = asyncio.new_event_loop()
    task = loop.create_task(execute(plan, approved=args.approved_preview, expected_hash=args.approved_preview_sha256,
        expires=args.authorization_expires_utc, confirm=args.confirm))
    try:
        result = loop.run_until_complete(task)
        print(json.dumps({k: result[k] for k in ('complete', 'stop_confirmed', 'sdk_stop_entries', 'latched_fault', 'hardware_stop_verified')}))
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
