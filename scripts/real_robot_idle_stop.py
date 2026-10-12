"""Offline preview by default; separately authorized one IDLE stop_move baseline."""
import argparse
import asyncio
from datetime import UTC, datetime, timedelta
import hashlib
import ipaddress
import json
import os
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'backend'))
CONFIRM = 'I_APPROVE_ONE_IDLE_STOP_MOVE_BASELINE'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def runtime_hashes():
    files = [*sorted((ROOT / 'backend/app').rglob('*.py')), Path(__file__).resolve()]
    files += [ROOT / '.venv/Lib/site-packages/lebai_sdk/lebai_sdk.pyd']
    return {str(path.relative_to(ROOT)): sha(path) for path in files}


def prepare(config, output, *, expires=None):
    from app.config import Settings, REAL_ROBOT_CONFIRMATION
    config, output = Path(config).resolve(strict=True), Path(output).resolve()
    config_bytes = config.read_bytes()
    if not output.is_relative_to((ROOT / '.tmp').resolve()) or output == (ROOT / '.tmp').resolve():
        raise ValueError('output_under_repo_tmp_required')
    previous = os.environ.get('VR4ARM_REAL_ROBOT_CONFIRM')
    os.environ['VR4ARM_REAL_ROBOT_CONFIRM'] = REAL_ROBOT_CONFIRMATION
    try:
        settings = Settings.load(config)
    finally:
        if previous is None:
            os.environ.pop('VR4ARM_REAL_ROBOT_CONFIRM', None)
        else:
            os.environ['VR4ARM_REAL_ROBOT_CONFIRM'] = previous
    if settings.backend != 'lebai' or settings.lebai is None or settings.lebai.mode != 'control':
        raise ValueError('idle_baseline_control_config_required')
    if config.read_bytes() != config_bytes:
        raise ValueError('idle_baseline_inputs_changed_during_parse')
    ip = ipaddress.IPv4Address(settings.lebai.ip)
    if ip.is_unspecified or ip.is_multicast or ip.is_loopback or not ip.is_private:
        raise ValueError('literal_private_ipv4_required')
    return {'config': config, 'output': output, 'settings': settings.lebai,
            'config_sha256': hashlib.sha256(config_bytes).hexdigest(), 'runtime_hashes': runtime_hashes(),
            'authorization_expires_utc': expires}


def preview(inputs):
    return {'schema': 1, 'scope': 'one_idle_stop_move_baseline', 'target_ip': inputs['settings'].ip,
            'config_sha256': inputs['config_sha256'], 'runtime_hashes': inputs['runtime_hashes'],
            'authorization_expires_utc': inputs['authorization_expires_utc'],
            'output': str(inputs['output']), 'allowed_writes': ['stop_move'], 'max_sdk_stop_entries': 1,
            'requires_raw_idle': True, 'requires_reported_zero_velocity': True,
            'sample_max_age_ms': 100, 'stop_rpc_timeout_ms': 200, 'readonly_tail_s': 2,
            'automatic_movement': False, 'hardware_stop_verified': False,
            'requires_fresh_onsite_authorization': True,
            'native_connection_release': 'standalone_process_exit_required_not_verified'}


def check(inputs, approved, expected_hash, expires, confirm):
    if confirm != CONFIRM:
        raise ValueError('idle_baseline_scope_confirmation_required')
    if expires != inputs['authorization_expires_utc']:
        raise ValueError('idle_baseline_expiry_not_bound')
    try:
        expiration = datetime.fromisoformat(expires.replace('Z', '+00:00'))
        now = datetime.now(UTC)
        if expiration.tzinfo is None or expiration.utcoffset() != timedelta(0) or not now < expiration <= now + timedelta(minutes=10):
            raise ValueError
    except (ValueError, TypeError, AttributeError):
        raise ValueError('idle_baseline_expiry_invalid_or_expired') from None
    if approved is None or type(expected_hash) is not str or not re.fullmatch('[a-f0-9]{64}', expected_hash):
        raise ValueError('idle_baseline_preview_hash_mismatch')
    try:
        approved_bytes = Path(approved).read_bytes()
    except OSError:
        raise ValueError('idle_baseline_preview_unavailable') from None
    if hashlib.sha256(approved_bytes).hexdigest() != expected_hash:
        raise ValueError('idle_baseline_preview_hash_mismatch')
    if json.loads(approved_bytes) != preview(inputs):
        raise ValueError('idle_baseline_scope_changed')
    if sha(inputs['config']) != inputs['config_sha256'] or runtime_hashes() != inputs['runtime_hashes']:
        raise ValueError('idle_baseline_inputs_changed')


async def execute(inputs, *, approved, expected_hash, expires, confirm, client_factory=None):
    from app.commissioning.idle_stop_baseline import run_baseline
    from app.recording.commissioning import CommissioningRecorder
    def admitted():
        check(inputs, approved, expected_hash, expires, confirm)
    admitted()  # Invalid admission cannot create output, load SDK or connect.
    output = inputs['output']
    output.mkdir(parents=True, exist_ok=False)  # Single attempt, including failures.
    with (output / 'admission.json').open('x', encoding='utf-8') as file:
        json.dump({'scope': preview(inputs), 'approved_preview_sha256': expected_hash,
                   'authorization_expires_utc': expires, 'hardware_stop_verified': False}, file, indent=2)
    (output / 'config-control.yaml').write_bytes(inputs['config'].read_bytes())
    if client_factory is None:
        from app.robots.lebai_sdk_bridge import connect_real_client
        factory = connect_real_client
        metadata = {}
    else:
        factory = client_factory
        metadata = {'runtime': 'SIMULATION', 'hardware_verified': False}
    recorder = CommissioningRecorder(output / 'sessions', metadata={
        'backend': 'LEBAI', 'real_robot_mode': 'control', 'workflow': 'idle_only_stop_baseline',
        'observe_stop_seconds': 2, **metadata})
    result = await run_baseline(inputs['settings'], recorder, factory, admitted)
    with (output / 'result.json').open('x', encoding='utf-8') as file:
        json.dump(result, file, indent=2)
        file.write('\n')
    return result


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
    inputs = prepare(args.config, args.output, expires=args.authorization_expires_utc)
    if not args.execute_once:
        print(json.dumps(preview(inputs), indent=2))
        return 0
    # The CLI is standalone; native connection teardown requires process exit.
    if Path(sys.executable).resolve() != (ROOT / '.venv/Scripts/python.exe').resolve():
        raise ValueError('repository_venv_required')
    result = asyncio.run(execute(inputs, approved=args.approved_preview,
        expected_hash=args.approved_preview_sha256, expires=args.authorization_expires_utc, confirm=args.confirm))
    print(json.dumps({key: result[key] for key in
        ('complete', 'stop_confirmed', 'sdk_stop_entries', 'latched_fault', 'hardware_stop_verified')}))
    return 0 if result['complete'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
