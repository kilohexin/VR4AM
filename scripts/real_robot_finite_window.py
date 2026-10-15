"""Preview by default; separately scoped single +Z finite window up to 0.2 mm.

All full SHA checks run before submission. The already loaded runtime's pinned
file identity/metadata is checked at PVAT entry; this is not a fresh byte hash
and cannot defend against deliberately preserved metadata or a compromised
process. Reads check expiry/config only. No normal VR/API selects this entry.
"""
import argparse
import asyncio
import copy
from datetime import UTC, datetime, timedelta
import hashlib
import importlib
import importlib.util
import json
from pathlib import Path
import re
import sys
from time import monotonic_ns

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'backend'))
spec = importlib.util.spec_from_file_location('finite_scope_helpers', Path(__file__).with_name('real_robot_paused_stop_cleanup.py'))
helper = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helper)
sha = helper.sha
CONFIRM = 'I_APPROVE_ONE_FINITE_WINDOW_AND_ONE_RESERVED_STOP_ON_REVOCATION'


def runtime_hashes():
    value = helper.runtime_hashes()
    value[Path(__file__).resolve().relative_to(ROOT).as_posix()] = sha(Path(__file__).resolve())
    return value


def existing_rpc_connections():
    return helper.existing_rpc_connections()


def prepare(config, output, *, expires=None, parallel_fk_audit=False, ready_joint_tolerance_rad=.0001,
            parallel_motion_reads=False):
    from app.commissioning.finite_window_run import validate_ready_tolerance
    if type(parallel_fk_audit) is not bool:
        raise TypeError('finite_parallel_fk_requires_boolean')
    if type(parallel_motion_reads) is not bool:
        raise TypeError('finite_parallel_snapshot_requires_boolean')
    plan = helper.prepare(config, output, expires=expires)
    plan['parallel_fk_audit'] = parallel_fk_audit
    plan['parallel_motion_reads'] = parallel_motion_reads
    plan['ready_joint_tolerance_rad'] = validate_ready_tolerance(ready_joint_tolerance_rad)
    if not .01 <= plan['settings'].control.pvat_horizon_s <= .08:
        raise ValueError('finite_window_horizon_invalid')
    plan['runtime_hashes'] = runtime_hashes()
    return plan


def preview(plan):
    from app.commissioning.finite_window_run import CLEANUP_STOP_S, DISTANCE_M, READY_Q_TOLERANCE_RAD, FiniteScopeClient
    from app.commissioning.readonly_kinematics import computational_distance
    distance_m=computational_distance(plan['settings'])
    return dict(schema=1, scope='one_finite_z_window_and_reserved_cleanup_stop', target_ip=plan['settings'].ip,
        output=str(plan['output']), config_sha256=plan['config_sha256'], runtime_hashes=plan['runtime_hashes'],
        authorization_expires_utc=plan['authorization_expires_utc'], max_connections=1, max_pvat_entries=1,
        max_stop_entries=1, max_ik_entries=1, max_fk_entries=4, allowed_queries=sorted(FiniteScopeClient.READS),
        allowed_writes=['move_pvat', 'stop_move'], target=dict(axis='z', distance_m=distance_m, rotation=False),
        requires_full_endpoint=distance_m<DISTANCE_M,
        parallel_fk_audit=plan['parallel_fk_audit'],
        parallel_motion_reads=plan['parallel_motion_reads'],
        snapshot_order='connected_then_parallel_six_then_dependent_motion_state' if plan['parallel_motion_reads'] else 'serial',
        ready_joint_tolerance_rad=plan['ready_joint_tolerance_rad'],
        fk_order='feedback_then_parallel_goal_midpoint_endpoint' if plan['parallel_fk_audit'] else 'serial',
        required_ready_joint_tolerance_rad=plan['ready_joint_tolerance_rad'] if distance_m<DISTANCE_M else None,
        requires_raw_idle=True, requires_zero_actual_qd_qdd=True, zero_terminal_velocity_acceleration=True,
        motion_ttl_ms=100, native_pvat_await_timeout_ms=60, stop_rpc_timeout_ms=200,
        stop_stable_ms=300, stop_verification_ms=500, readonly_tail_s=2.,
        cleanup_stop_on_motion_revocation=True, cleanup_stop_on_math_failure=True,
        cleanup_stop_reservation_s=CLEANUP_STOP_S, minimum_cleanup_reservation_at_pvat_s=1.,
        runtime_checks='full_sha_before_lease_metadata_at_pvat_for_already_loaded_code',
        runtime_metadata_is_not_new_byte_hash=True, reads_check_expiry_and_config=True,
        no_new_runtime_import_during_motion=True, automatic_retry=False, automatic_resume=False,
        automatic_reset=False, automatic_home=False, automatic_gripper=False, packet_capture=False,
        hardware_verified=False, motion_authorized=False, requires_fresh_onsite_authorization=True,
        prerequisites='fresh_readonly_fk_ik_evidence_and_separate_site_approval_before_live_use',
        native_connection_release='standalone_process_exit_required_not_verified')


class Inputs:
    def __init__(self, plan, approved, expected_hash, expires, confirm):
        self.plan, self.approved, self.expected_hash = plan, approved, expected_hash
        self.expires, self.confirm = expires, confirm
        self.copied = None
        self.paths = self.fingerprints = None

    def expiration(self):
        try:
            expiry = datetime.fromisoformat(self.expires.replace('Z', '+00:00'))
            now = datetime.now(UTC)
            if (expiry.tzinfo is None or expiry.utcoffset() != timedelta(0) or not now < expiry <= now + timedelta(minutes=10)
                    or self.expires != self.plan['authorization_expires_utc']): raise ValueError
        except (ValueError, TypeError, AttributeError):
            raise ValueError('finite_scope_expiry_invalid_or_expired') from None
        return expiry

    @staticmethod
    def metadata(paths):
        values = []
        for path in paths:
            stat = path.stat()
            values.append((stat.st_size, stat.st_mtime_ns, stat.st_ctime_ns, stat.st_dev, stat.st_ino))
        return tuple(values)

    def cold(self):
        if self.confirm != CONFIRM: raise ValueError('finite_scope_confirmation_required')
        expiry = self.expiration()
        if (self.approved is None or type(self.expected_hash) is not str
                or re.fullmatch('[a-f0-9]{64}', self.expected_hash) is None):
            raise ValueError('finite_scope_preview_hash_required')
        try:
            raw = Path(self.approved).read_bytes(); approved = json.loads(raw)
        except (OSError, ValueError): raise ValueError('finite_scope_preview_unavailable') from None
        if hashlib.sha256(raw).hexdigest() != self.expected_hash or approved != preview(self.plan):
            raise ValueError('finite_scope_preview_or_hash_changed')
        if self.paths is None:
            paths = tuple((ROOT / name).resolve(strict=True) for name in self.plan['runtime_hashes'])
            if not all(p.is_relative_to(ROOT.resolve()) for p in paths): raise ValueError('finite_scope_runtime_path_invalid')
        else: paths = self.paths
        before = self.metadata(paths)
        if (self.fingerprints is not None and before != self.fingerprints
                or runtime_hashes() != self.plan['runtime_hashes']):
            raise ValueError('finite_scope_runtime_changed')
        after = self.metadata(paths)
        if before != after: raise ValueError('finite_scope_runtime_changed_during_hashes')
        self.paths, self.fingerprints = paths, after
        self.hot()
        if datetime.now(UTC) >= expiry: raise ValueError('finite_scope_expired_during_hashes')

    def hot(self):
        expiry = self.expiration()
        if (sha(self.plan['config']) != self.plan['config_sha256']
                or self.copied is not None and sha(self.copied) != self.plan['config_sha256']):
            raise ValueError('finite_scope_configuration_changed')
        if datetime.now(UTC) >= expiry: raise ValueError('finite_scope_expired_during_configuration_checks')

    def write(self):
        self.hot()
        if self.paths is None or self.metadata(self.paths) != self.fingerprints:
            raise ValueError('finite_scope_loaded_runtime_metadata_changed')
        self.hot()  # Expiry/config must survive the metadata check too.


async def execute(plan, *, approved, expected_hash, expires, confirm, client_factory=None):
    from app.commissioning.finite_window_run import run_window
    from app.commissioning.readonly_kinematics import _Queries, new_report
    from app.recording.commissioning import CommissioningRecorder
    inputs = Inputs(plan, approved, expected_hash, expires, confirm)
    inputs.cold()
    if existing_rpc_connections() != 0: raise ValueError('existing_rpc_connections')
    output = plan['output']; output.mkdir(parents=True, exist_ok=False)
    (output / 'admission.json').write_text(json.dumps(dict(scope=preview(plan), approved_preview_sha256=expected_hash,
        started_utc=datetime.now(UTC).isoformat()), indent=2), encoding='utf-8')
    inputs.copied = output / 'config-control.yaml'
    inputs.copied.write_bytes(plan['config'].read_bytes())
    connector = new_report(simulation=client_factory is not None)
    calls = _Queries(inputs.cold, connector)
    calls.deadline = monotonic_ns() + 3_000_000_000
    connection_used = False
    report = dict(runtime='SIMULATION' if client_factory is not None else 'LIVE_FINITE_DIAGNOSTIC')
    recorder = CommissioningRecorder(output / 'sessions', metadata=dict(workflow='single_finite_window', **report))

    async def factory(ip):
        nonlocal connection_used
        calls.guard()
        if connection_used or ip != plan['settings'].ip or existing_rpc_connections() != 0:
            raise ValueError('finite_scope_connection_not_admitted')
        connection_used = True
        async def connect():
            calls.guard()
            if client_factory is not None: return await client_factory(ip)
            helper.readonly.verify_sdk_source(); calls.guard()
            module = importlib.import_module('lebai_sdk'); calls.guard()
            connector['native_sdk_init_entered'] = True; module.init(); calls.guard()
            connector['native_sdk_connect_entered'] = True
            return await module.connect(ip, False)
        return await calls.call('connect', connect)

    try:
        await run_window(plan['settings'], recorder, factory, inputs.cold, inputs.hot,
                         write_check=inputs.write, report=report, parallel_fk_audit=plan['parallel_fk_audit'],
                         ready_joint_tolerance_rad=plan['ready_joint_tolerance_rad'],
                         parallel_motion_reads=plan['parallel_motion_reads'])
    finally:
        calls.closed = True
        connector['pending_remote_result_unknown'] = any(c['remote_result_unknown'] for c in connector['calls'])
        report['connection_diagnostics'] = copy.deepcopy(connector)
        (output / 'result.json').write_text(json.dumps(report, allow_nan=False, indent=2) + '\n', encoding='utf-8')
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True, type=Path); parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--execute-once', action='store_true'); parser.add_argument('--approved-preview', type=Path)
    parser.add_argument('--parallel-fk-audit', action='store_true')
    parser.add_argument('--parallel-motion-reads', action='store_true')
    parser.add_argument('--ready-joint-tolerance-rad', type=float, default=.0001)
    parser.add_argument('--approved-preview-sha256'); parser.add_argument('--authorization-expires-utc'); parser.add_argument('--confirm')
    args = parser.parse_args(argv)
    plan = prepare(args.config, args.output, expires=args.authorization_expires_utc,
                   parallel_fk_audit=args.parallel_fk_audit, ready_joint_tolerance_rad=args.ready_joint_tolerance_rad,
                   parallel_motion_reads=args.parallel_motion_reads)
    if not args.execute_once:
        print(json.dumps(preview(plan), indent=2)); return 0
    if Path(sys.executable).resolve() != (ROOT / '.venv/Scripts/python.exe').resolve(): raise ValueError('repository_venv_required')
    loop = asyncio.new_event_loop()
    task = loop.create_task(execute(plan, approved=args.approved_preview, expected_hash=args.approved_preview_sha256,
        expires=args.authorization_expires_utc, confirm=args.confirm))
    try:
        result = loop.run_until_complete(task)
        print(json.dumps({k:result[k] for k in ('complete','pvat_entries','stop_entries','latched_fault','hardware_verified')}))
        return 0 if result['complete'] else 2
    except KeyboardInterrupt:
        task.cancel()
        try: loop.run_until_complete(task)
        except asyncio.CancelledError: pass
        return 130
    finally: loop.close()


if __name__ == '__main__': raise SystemExit(main())
