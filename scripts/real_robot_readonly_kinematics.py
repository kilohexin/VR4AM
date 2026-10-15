"""Offline preview by default; one separately authorized FK/IK read-only survey."""
import argparse
import asyncio
from datetime import UTC, datetime, timedelta
import hashlib
import importlib
import importlib.machinery
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
CONFIRM = 'I_APPROVE_ONE_READONLY_FK_IK_SURVEY_NO_ROBOT_WRITES'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def runtime_hashes():
    paths = [*sorted((ROOT / 'backend/app').rglob('*.py')), Path(__file__).resolve(),
             ROOT / 'scripts/local_rpc_connections.py']
    paths += [ROOT / '.venv/Lib/site-packages/lebai_sdk' / name for name in ('__init__.py', 'lebai_sdk.pyd')]
    return {p.relative_to(ROOT).as_posix(): sha(p) for p in paths}


def existing_rpc_connections():
    spec = importlib.util.spec_from_file_location('kinematics_local_rpc', ROOT / 'scripts/local_rpc_connections.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.read_rpc_connections()  # Numeric local table only; no network probes.


def verify_sdk_source():
    package = (ROOT / '.venv/Lib/site-packages/lebai_sdk').resolve()
    if 'lebai_sdk' in sys.modules or 'lebai_sdk.lebai_sdk' in sys.modules:
        raise ValueError('sdk_already_loaded_before_admission')
    spec = importlib.machinery.PathFinder.find_spec('lebai_sdk', sys.path)
    extension = importlib.machinery.PathFinder.find_spec('lebai_sdk.lebai_sdk', [str(package)])
    if (spec is None or spec.origin is None or Path(spec.origin).resolve() != package / '__init__.py'
            or list(spec.submodule_search_locations or ()) != [str(package)]
            or extension is None or extension.origin is None
            or Path(extension.origin).resolve() != package / 'lebai_sdk.pyd'
            or not isinstance(extension.loader, importlib.machinery.ExtensionFileLoader)):
        raise ValueError('sdk_import_source_not_bound')


def prepare(config, output, *, expires=None):
    from app.config import Settings
    config, output = Path(config).resolve(strict=True), Path(output).resolve()
    raw = config.read_bytes()
    data = yaml.safe_load(raw)
    real = data.get('real_robot') if isinstance(data, dict) else None
    if not isinstance(real, dict) or data.get('backend') != 'lebai' or real.get('mode') != 'readonly':
        raise ValueError('kinematics_readonly_config_required')
    settings = Settings.load(config)
    if settings.backend != 'lebai' or settings.lebai is None or settings.lebai.mode != 'readonly':
        raise ValueError('kinematics_readonly_config_required')
    if config.read_bytes() != raw:
        raise ValueError('kinematics_config_changed_during_parse')
    robot = settings.lebai
    ip = ipaddress.IPv4Address(robot.ip)
    if ip.is_unspecified or ip.is_multicast or ip.is_loopback or not ip.is_private:
        raise ValueError('literal_private_ipv4_required')
    if output == (ROOT / '.tmp').resolve() or not output.is_relative_to((ROOT / '.tmp').resolve()):
        raise ValueError('output_under_repo_tmp_required')
    for key, cap in dict(max_joint_speed_radps=.05, max_joint_acceleration_radps2=.2, pvat_horizon_s=.08).items():
        value = getattr(robot.control, key)
        if not math.isfinite(value) or not 0 < value <= cap:
            raise ValueError('kinematics_candidate_cap_invalid:' + key)
    if robot.control.pvat_horizon_s < .01:
        raise ValueError('kinematics_candidate_horizon_too_short')
    return dict(config=config, output=output, settings=robot,
                config_sha256=hashlib.sha256(raw).hexdigest(), runtime_hashes=runtime_hashes(),
                authorization_expires_utc=expires)


def preview(plan):
    from app.commissioning.readonly_kinematics import READ_METHODS, MAX_CALLS, computational_distance
    return dict(schema=1, scope='one_readonly_fk_ik_survey_no_robot_writes', mode='readonly',
                target_ip=plan['settings'].ip, output=str(plan['output']),
                config_sha256=plan['config_sha256'], runtime_hashes=plan['runtime_hashes'],
                authorization_expires_utc=plan['authorization_expires_utc'],
                allowed_queries=list(READ_METHODS), allowed_writes=[], max_connections=1,
                max_total_calls_including_connect=MAX_CALLS, max_fk_calls=4, max_ik_calls=1,
                computational_target=dict(axis='z', distance_m=computational_distance(plan['settings']), executed=False),
                allowed_raw_states=['IDLE', 'PAUSED'], motion_authorized=False,
                hardware_stop_verified=False, automatic_retry=False, automatic_resume=False,
                automatic_reset=False, automatic_stop=False, packet_capture=False,
                connect_timeout_ms=5000, read_timeout_ms=1000, total_budget_ms=15000,
                fk_feedback_position_tolerance_m=.0001, fk_rotation_tolerance_deg=.1,
                requires_fresh_onsite_authorization=True,
                completion_means='read_only_evidence_not_motion_or_stop_acceptance',
                native_connection_release='standalone_process_exit_required_not_verified')


def check(plan, *, approved, expected_hash, expires, confirm):
    if confirm != CONFIRM:
        raise ValueError('kinematics_scope_confirmation_required')
    try:
        expiration = datetime.fromisoformat(expires.replace('Z', '+00:00'))
        now = datetime.now(UTC)
        if (expiration.tzinfo is None or expiration.utcoffset() != timedelta(0)
                or not now < expiration <= now + timedelta(minutes=10)
                or expires != plan['authorization_expires_utc']):
            raise ValueError
    except (ValueError, TypeError, AttributeError):
        raise ValueError('kinematics_expiry_invalid_or_expired') from None
    if approved is None or type(expected_hash) is not str or re.fullmatch('[a-f0-9]{64}', expected_hash) is None:
        raise ValueError('kinematics_preview_hash_required')
    try:
        raw = Path(approved).read_bytes()
        approved_value = json.loads(raw)
    except (OSError, ValueError):
        raise ValueError('kinematics_preview_unavailable_or_invalid') from None
    if hashlib.sha256(raw).hexdigest() != expected_hash or approved_value != preview(plan):
        raise ValueError('kinematics_scope_or_hash_changed')
    if sha(plan['config']) != plan['config_sha256'] or runtime_hashes() != plan['runtime_hashes']:
        raise ValueError('kinematics_inputs_changed')
    if datetime.now(UTC) >= expiration:
        raise ValueError('kinematics_authorization_expired_during_input_checks')


async def execute(plan, *, approved, expected_hash, expires, confirm, client_factory=None):
    from app.commissioning.readonly_kinematics import collect, new_report

    def admit():
        check(plan, approved=approved, expected_hash=expected_hash, expires=expires, confirm=confirm)

    admit()
    if existing_rpc_connections() != 0:
        raise ValueError('existing_rpc_connections')
    output = plan['output']
    output.mkdir(parents=True, exist_ok=False)  # One attempt, including failed SDK connection.
    (output / 'admission.json').write_text(json.dumps(dict(scope=preview(plan),
        approved_preview_sha256=expected_hash, started_utc=datetime.now(UTC).isoformat()), indent=2), encoding='utf-8')
    report = new_report(simulation=client_factory is not None)
    connection_used = False

    async def admitted_factory(ip):
        nonlocal connection_used
        admit()
        if connection_used or ip != plan['settings'].ip or existing_rpc_connections() != 0:
            raise ValueError('kinematics_connection_not_admitted')
        connection_used = True
        def before_sdk_entry():
            admit()
            if monotonic_ns() >= report['connect_deadline_ns']:
                raise ValueError('kinematics_connect_deadline_expired')

        before_sdk_entry()  # The synchronous local table can consume the lease.
        if client_factory is not None:
            return await client_factory(ip)
        verify_sdk_source()
        before_sdk_entry()
        module = importlib.import_module('lebai_sdk')
        before_sdk_entry()
        report.update(native_sdk_init_entered=True, native_sdk_init_started_ns=monotonic_ns())
        module.init()
        before_sdk_entry()  # Import/init cannot renew or outlive the admitted connection window.
        report.update(native_sdk_connect_entered=True, native_sdk_connect_started_ns=monotonic_ns())
        return await module.connect(ip, False)

    try:
        await collect(plan['settings'], admitted_factory, admit, report)
    finally:
        # No adapter disconnect, stop, reset, resume, startup or gripper cleanup.
        (output / 'result.json').write_text(json.dumps(report, ensure_ascii=False, allow_nan=False, indent=2) + '\n', encoding='utf-8')
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
    # asyncio.run waits for all cancelled tasks during shutdown. An SDK read
    # ignoring cancellation must not prevent writing the report and process exit.
    loop = asyncio.new_event_loop()
    task = loop.create_task(execute(plan, approved=args.approved_preview,
        expected_hash=args.approved_preview_sha256, expires=args.authorization_expires_utc, confirm=args.confirm))
    try:
        result = loop.run_until_complete(task)
        print(json.dumps({key: result[key] for key in ('complete', 'kinematics_consistent', 'motion_authorized',
            'hardware_stop_verified', 'pending_remote_result_unknown')}))
        return 0 if result['complete'] else 2
    except KeyboardInterrupt:
        task.cancel()
        try:
            loop.run_until_complete(task)  # Flush owned local evidence, without awaiting the SDK read.
        except asyncio.CancelledError:
            pass
        return 130
    finally:
        loop.close()  # Native connection release is only assessed after standalone process exit.


if __name__ == '__main__':
    raise SystemExit(main())
