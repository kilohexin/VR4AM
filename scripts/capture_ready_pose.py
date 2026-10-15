"""Preview first; one separately approved read-only operator-ready-pose capture.

No adapter, IK/FK, startup, motion, gripper or stop path is constructed. Reported
zero velocity/acceleration and matching samples do not prove physical standstill.
Native resource release requires standalone process exit and remains unverified.
"""
import argparse
import asyncio
import copy
from datetime import UTC, datetime, timedelta
import hashlib
import importlib
import importlib.util
import ipaddress
import json
import math
from pathlib import Path
import re
import sys
from time import monotonic_ns

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'backend'))
CONFIRM = 'I_APPROVE_ONE_READONLY_OPERATOR_SELECTED_READY_POSE_CAPTURE_NO_WRITES'
READ_METHODS = ('is_connected', 'get_robot_state', 'get_estop_reason', 'get_kin_data', 'get_tcp')
MAX_CALLS = 13  # One connect, two observations of six fixed reads each.
CONNECT_TIMEOUT_S = 5.
READ_TIMEOUT_S = 1.
TOTAL_TIMEOUT_S = 15.
SAMPLE_SEPARATION_S = .1
POSE_FIELDS = ('x', 'y', 'z', 'rz', 'ry', 'rx')

_spec = importlib.util.spec_from_file_location('ready_capture_local_helpers',
                                            Path(__file__).with_name('real_robot_readonly_kinematics.py'))
_helpers = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_helpers)  # Definitions only; no commissioning or native SDK import.
sha = _helpers.sha
existing_rpc_connections = _helpers.existing_rpc_connections
verify_sdk_source = _helpers.verify_sdk_source


def runtime_hashes():
    value = _helpers.runtime_hashes()
    value[Path(__file__).resolve().relative_to(ROOT).as_posix()] = sha(__file__)
    value['python_executable:' + str(Path(sys.executable).resolve())] = sha(sys.executable)
    return value


def prepare(config, output, *, issued=None, expires=None, site=None):
    """Build an offline preview; omitted site/times leave execution inadmissible."""
    from app.config import Settings
    config, output = Path(config).resolve(strict=True), Path(output).resolve()
    raw = config.read_bytes()
    data = yaml.safe_load(raw)
    real = data.get('real_robot') if isinstance(data, dict) else None
    if not isinstance(real, dict) or data.get('backend') != 'lebai' or real.get('mode') != 'readonly':
        raise ValueError('ready_capture_readonly_config_required')
    settings = Settings.load(config)
    if settings.backend != 'lebai' or settings.lebai is None or settings.lebai.mode != 'readonly':
        raise ValueError('ready_capture_readonly_config_required')
    if config.read_bytes() != raw:
        raise ValueError('ready_capture_config_changed_during_parse')
    ip = ipaddress.IPv4Address(settings.lebai.ip)
    if ip.is_unspecified or ip.is_multicast or ip.is_loopback or not ip.is_private:
        raise ValueError('literal_private_ipv4_required')
    if output == (ROOT / '.tmp').resolve() or not output.is_relative_to((ROOT / '.tmp').resolve()):
        raise ValueError('output_under_repo_tmp_required')
    return dict(config=config, output=output, settings=settings.lebai,
                config_sha256=hashlib.sha256(raw).hexdigest(), runtime_hashes=runtime_hashes(),
                authorization_issued_utc=issued, authorization_expires_utc=expires,
                site_confirmation=copy.deepcopy(site))


def preview(plan):
    return dict(schema=1, scope='one_readonly_operator_selected_current_ready_pose_capture',
        mode='readonly', target_ip=plan['settings'].ip, output=str(plan['output']),
        config_sha256=plan['config_sha256'], runtime_hashes=copy.deepcopy(plan['runtime_hashes']),
        authorization_issued_utc=plan['authorization_issued_utc'],
        authorization_expires_utc=plan['authorization_expires_utc'],
        site_confirmation=copy.deepcopy(plan['site_confirmation']),
        allowed_queries=list(READ_METHODS), allowed_writes=[], max_connections=1,
        max_total_calls_including_connect=MAX_CALLS, observations=2,
        observation_queries=['is_connected', 'get_robot_state', 'get_estop_reason',
                             'get_kin_data', 'get_tcp', 'get_robot_state'],
        allowed_raw_states=['IDLE', 'PAUSED'], require_exact_zero_actual_qd_and_qdd=True,
        require_unchanged_state_pose_and_tool=True, joint_sample_tolerance_rad=1e-6,
        tcp_sample_tolerance_m_and_rad=1e-6, minimum_observation_separation_ms=SAMPLE_SEPARATION_S * 1000,
        connect_timeout_ms=CONNECT_TIMEOUT_S * 1000, read_timeout_ms=READ_TIMEOUT_S * 1000,
        total_budget_ms=TOTAL_TIMEOUT_S * 1000, motion_authorized=False, hardware_stop_verified=False,
        automatic_retry=False, automatic_resume=False, automatic_reset=False, automatic_stop=False,
        packet_capture=False, requires_fresh_onsite_authorization=True,
        selection='operator_selected_current_working_ready_pose',
        assessment='reported_telemetry_only_not_mathematical_optimality_or_motion_acceptance',
        native_connection_release='standalone_process_exit_required_not_verified')


def _utc(value):
    date = datetime.fromisoformat(value.replace('Z', '+00:00'))
    if date.tzinfo is None or date.utcoffset() != timedelta(0):
        raise ValueError('ready_capture_utc_required')
    return date


def check(plan, *, approved, expected_hash, confirm):
    if confirm != CONFIRM:
        raise ValueError('ready_capture_scope_confirmation_required')
    now = datetime.now(UTC)
    try:
        issued, expires = _utc(plan['authorization_issued_utc']), _utc(plan['authorization_expires_utc'])
        site = plan['site_confirmation']
        confirmed = _utc(site['confirmed_utc'])
        if (not now - timedelta(minutes=10) <= issued <= now < expires <= issued + timedelta(minutes=10)
                or not now - timedelta(minutes=10) <= confirmed <= issued
                or site['target_ip'] != plan['settings'].ip
                or any(site[key] is not True for key in ('robot_stationary', 'no_concurrent_control',
                                                         'operator_selected_ready_pose'))):
            raise ValueError
    except (ValueError, KeyError, TypeError, AttributeError):
        raise ValueError('ready_capture_expiry_or_site_confirmation_invalid') from None
    if approved is None or type(expected_hash) is not str or re.fullmatch('[a-f0-9]{64}', expected_hash) is None:
        raise ValueError('ready_capture_preview_hash_required')
    try:
        raw = Path(approved).read_bytes()
        value = json.loads(raw)
    except (OSError, ValueError):
        raise ValueError('ready_capture_preview_unavailable') from None
    if hashlib.sha256(raw).hexdigest() != expected_hash or value != preview(plan):
        raise ValueError('ready_capture_scope_or_hash_changed')
    if sha(plan['config']) != plan['config_sha256'] or runtime_hashes() != plan['runtime_hashes']:
        raise ValueError('ready_capture_inputs_changed')
    if datetime.now(UTC) >= expires:
        raise ValueError('ready_capture_authorization_expired_during_checks')


class CaptureError(ValueError):
    """Only fixed local reasons are saved; vendor exception messages are redacted."""


class _Queries:
    def __init__(self, admit, report):
        self.admit, self.report = admit, report
        self.deadline = monotonic_ns() + int(TOTAL_TIMEOUT_S * 1e9)
        self.pending = None
        self.closed = False

    def guard(self):
        self.admit()
        if self.closed or monotonic_ns() >= self.deadline:
            raise CaptureError('capture_total_budget_expired')

    async def call(self, name, invoke):
        self.guard()
        if name not in ('connect', *READ_METHODS) or len(self.report['calls']) >= MAX_CALLS:
            raise CaptureError('capture_call_budget_or_allowlist')
        if self.pending is not None and not self.pending.done():
            raise CaptureError('capture_query_pending')
        item = dict(method=name, started_utc=datetime.now(UTC).isoformat(), started_ns=monotonic_ns(),
                    entered=False, outcome='not_started', remote_result_unknown=False)
        self.report['calls'].append(item)
        timeout = CONNECT_TIMEOUT_S if name == 'connect' else READ_TIMEOUT_S
        deadline = min(self.deadline, item['started_ns'] + int(timeout * 1e9))
        if name == 'connect': self.report['connect_deadline_ns'] = deadline

        async def query():
            self.guard()
            if monotonic_ns() >= deadline:
                raise CaptureError('capture_query_expired_before_entry')
            item['entered'] = True
            return await invoke(), monotonic_ns()

        task = self.pending = asyncio.create_task(query(), name='ready-pose-' + name)
        def consume_exception(done):
            if not done.cancelled(): done.exception()
        task.add_done_callback(consume_exception)
        try:
            done, _ = await asyncio.wait({task}, timeout=max(0., (deadline - monotonic_ns()) / 1e9))
            if not done:
                task.cancel()
                item.update(outcome='timeout', remote_result_unknown=item['entered'])
                raise CaptureError('capture_query_timeout')
            raw, completed = task.result()
            item.update(outcome='returned', returned_ns=completed)
            if completed >= deadline or monotonic_ns() >= deadline:
                raise CaptureError('capture_late_return')
            self.guard()
            if monotonic_ns() >= deadline:
                raise CaptureError('capture_late_return')
            if name == 'connect': return raw
            frozen = json.loads(json.dumps(raw, allow_nan=False))
            item['raw'] = frozen
            return copy.deepcopy(frozen)
        except asyncio.CancelledError:
            item.update(outcome='cancelled', remote_result_unknown=item['entered'])
            if not task.done() and not task.cancelling(): task.cancel()
            raise
        except CaptureError:
            raise
        except Exception:
            item['outcome'] = 'error' if item['entered'] else 'not_started'
            raise
        finally:
            item.update(completed_ns=monotonic_ns(), completed_utc=datetime.now(UTC).isoformat())


def _state(raw):
    value = raw.strip().upper() if type(raw) is str else raw
    if type(value) is int and value in (5, 6): return 'IDLE' if value == 5 else 'PAUSED'
    if type(value) is str and value in ('IDLE', 'PAUSED'): return value
    raise CaptureError('capture_state_not_idle_or_paused')


def _number(value):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise CaptureError('capture_nonfinite_or_invalid_numeric_telemetry')
    return float(value)


def _joints(raw):
    if not isinstance(raw, (list, tuple)) or len(raw) != 6:
        raise CaptureError('capture_invalid_six_joint_vector')
    return [_number(value) for value in raw]


def _pose(raw):
    if not isinstance(raw, dict) or any(key not in raw for key in POSE_FIELDS):
        raise CaptureError('capture_invalid_tcp_pose')
    return {key: _number(raw[key]) for key in POSE_FIELDS}


async def _observe(client, calls):
    async def read(name):
        return await calls.call(name, lambda: getattr(client, name)())
    if await read('is_connected') is not True:
        raise CaptureError('capture_not_connected')
    state = _state(await read('get_robot_state'))
    estop = await read('get_estop_reason')
    if not ((type(estop) is int and estop == 0)
            or (type(estop) is str and estop.strip().lower() in ('0', 'none'))):
        raise CaptureError('capture_estop_or_unknown')
    raw = await read('get_kin_data')
    if not isinstance(raw, dict): raise CaptureError('capture_invalid_kinematics')
    q = _joints(raw.get('actual_joint_pose'))
    qd = _joints(raw.get('actual_joint_speed'))
    qdd = _joints(raw.get('actual_joint_acc'))
    if any(value != 0. for value in (*qd, *qdd)):
        raise CaptureError('capture_reported_velocity_or_acceleration_nonzero')
    tcp = _pose(raw.get('actual_tcp_pose'))
    tool = _pose(await read('get_tcp'))
    if _state(await read('get_robot_state')) != state:
        raise CaptureError('capture_state_changed_during_observation')
    return dict(state=state, q_rad=q, qd_radps=qd, qdd_radps2=qdd,
                actual_tcp_pose=tcp, tool_tcp_setting=tool,
                observed_utc=datetime.now(UTC).isoformat(), observed_ns=monotonic_ns())


def _unchanged(initial, final):
    return (initial['state'] == final['state']
            and max(abs(a - b) for a, b in zip(initial['q_rad'], final['q_rad'])) <= 1e-6
            and all(abs(initial[key][axis] - final[key][axis]) <= 1e-6
                    for key in ('actual_tcp_pose', 'tool_tcp_setting') for axis in POSE_FIELDS))


async def execute(plan, *, approved, expected_hash, confirm, client_factory=None):
    def admit():
        check(plan, approved=approved, expected_hash=expected_hash, confirm=confirm)
    admit()
    if existing_rpc_connections() != 0: raise ValueError('existing_rpc_connections')
    output = plan['output']
    output.mkdir(parents=True, exist_ok=False)  # Every admitted attempt consumes its directory.
    (output / 'admission.json').write_text(json.dumps(dict(scope=preview(plan),
        approved_preview_sha256=expected_hash, started_utc=datetime.now(UTC).isoformat()),
        allow_nan=False, indent=2) + '\n', encoding='utf-8')
    report = dict(schema=1, complete=False, runtime='SIMULATION' if client_factory is not None else 'LIVE_READONLY',
                  started_utc=datetime.now(UTC).isoformat(), motion_authorized=False,
                  hardware_stop_verified=False, native_sdk_init_entered=False, native_sdk_connect_entered=False,
                  native_connection_release='standalone_process_exit_required_not_verified',
                  pending_remote_result_unknown=False, calls=[],
                  assessment='reported_telemetry_only_not_mathematical_optimality_or_motion_acceptance')
    calls = _Queries(admit, report)
    connection_used = False

    async def factory():
        nonlocal connection_used
        calls.guard()
        if connection_used or existing_rpc_connections() != 0:
            raise CaptureError('capture_connection_not_admitted')
        connection_used = True
        def before_sdk_entry():
            calls.guard()
            if monotonic_ns() >= report['connect_deadline_ns']:
                raise CaptureError('capture_connect_deadline_expired')
        before_sdk_entry()  # Local table/approval checks cannot renew the connection window.
        if client_factory is not None: return await client_factory(plan['settings'].ip)
        verify_sdk_source()
        before_sdk_entry()
        module = importlib.import_module('lebai_sdk')
        before_sdk_entry()
        report['native_sdk_init_entered'] = True
        module.init()
        before_sdk_entry()
        report['native_sdk_connect_entered'] = True
        return await module.connect(plan['settings'].ip, False)

    try:
        client = await calls.call('connect', factory)
        if any(not callable(getattr(client, method, None)) for method in READ_METHODS):
            raise CaptureError('capture_read_capability_missing')
        initial = report['initial'] = await _observe(client, calls)
        await asyncio.sleep(SAMPLE_SEPARATION_S)
        calls.guard()
        final = report['final'] = await _observe(client, calls)
        if not _unchanged(initial, final): raise CaptureError('capture_observation_changed')
        calls.guard()
        telemetry_hash = hashlib.sha256(json.dumps(dict(initial=initial, final=final),
            sort_keys=True, allow_nan=False, separators=(',', ':')).encode('utf-8')).hexdigest()
        report['operator_selected_ready_pose'] = dict(selection='operator_selected_current_working_ready_pose',
            q_rad=final['q_rad'], actual_tcp_pose=final['actual_tcp_pose'], tool_tcp_setting=final['tool_tcp_setting'],
            raw_state=final['state'], captured_utc=final['observed_utc'],
            tcp_units=dict(position='m', orientation='rad_ZYX_rz_ry_rx'),
            source=dict(target_ip=plan['settings'].ip, config_sha256=plan['config_sha256'],
                        telemetry_sha256=telemetry_hash,
                        runtime_hashes=copy.deepcopy(plan['runtime_hashes']), approved_preview_sha256=expected_hash,
                        authorization_issued_utc=plan['authorization_issued_utc'],
                        authorization_expires_utc=plan['authorization_expires_utc'],
                        site_confirmation=copy.deepcopy(plan['site_confirmation'])))
        report['complete'] = True
    except asyncio.CancelledError:
        report['error'] = 'interrupted'
        raise
    except Exception as error:
        report['error'] = str(error) if isinstance(error, CaptureError) else 'capture_query_or_admission_rejected'
        report['error_type'] = type(error).__name__
    finally:
        calls.closed = True
        if calls.pending is not None and not calls.pending.done() and not calls.pending.cancelling():
            calls.pending.cancel()
        report['pending_remote_result_unknown'] = any(row['remote_result_unknown'] for row in report['calls'])
        report['calls'] = copy.deepcopy(report['calls'])
        report['completed_utc'] = datetime.now(UTC).isoformat()
        (output / 'result.json').write_text(json.dumps(report, allow_nan=False, indent=2) + '\n', encoding='utf-8')
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--execute-once', action='store_true')
    parser.add_argument('--approved-preview', type=Path)
    parser.add_argument('--approved-preview-sha256')
    parser.add_argument('--authorization-issued-utc')
    parser.add_argument('--authorization-expires-utc')
    parser.add_argument('--site-confirmed-utc')
    parser.add_argument('--confirmed-target-ip')
    parser.add_argument('--confirm-robot-stationary', action='store_true')
    parser.add_argument('--confirm-no-concurrent-control', action='store_true')
    parser.add_argument('--confirm-operator-selected-ready-pose', action='store_true')
    parser.add_argument('--confirm')
    args = parser.parse_args(argv)
    site = None
    if any((args.site_confirmed_utc, args.confirmed_target_ip, args.confirm_robot_stationary,
            args.confirm_no_concurrent_control, args.confirm_operator_selected_ready_pose)):
        site = dict(confirmed_utc=args.site_confirmed_utc, target_ip=args.confirmed_target_ip,
                    robot_stationary=args.confirm_robot_stationary, no_concurrent_control=args.confirm_no_concurrent_control,
                    operator_selected_ready_pose=args.confirm_operator_selected_ready_pose)
    plan = prepare(args.config, args.output, issued=args.authorization_issued_utc,
                   expires=args.authorization_expires_utc, site=site)
    if not args.execute_once:
        print(json.dumps(preview(plan), allow_nan=False, indent=2))
        return 0
    if Path(sys.executable).resolve() != (ROOT / '.venv/Scripts/python.exe').resolve():
        raise ValueError('repository_venv_required')
    loop = asyncio.new_event_loop()
    task = loop.create_task(execute(plan, approved=args.approved_preview,
        expected_hash=args.approved_preview_sha256, confirm=args.confirm))
    try:
        result = loop.run_until_complete(task)
        print(json.dumps({key: result[key] for key in ('complete', 'motion_authorized', 'hardware_stop_verified',
                                                      'pending_remote_result_unknown')}))
        return 0 if result['complete'] else 2
    except KeyboardInterrupt:
        task.cancel()
        try: loop.run_until_complete(task)
        except asyncio.CancelledError: pass
        return 130
    finally:
        loop.close()  # No native close proof; no stop/reset/disconnect/cleanup robot commands.


if __name__ == '__main__':
    raise SystemExit(main())
