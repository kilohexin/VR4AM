"""Only offline CLI guards and harmless Windows preview processes."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
from datetime import datetime, UTC, timedelta
from uuid import uuid4

import pytest
from tests.scripts.test_readonly_wire_capture import FakeNative

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location('capture_launch_guards', ROOT / 'scripts/readonly_wire_capture.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


@pytest.fixture
def launch_args(tmp_path, monkeypatch):
    config = tmp_path / 'readonly.yaml'
    config.write_text('real_robot:\n  mode: readonly\n  ip: 192.0.2.1\n')
    output = ROOT / '.tmp' / ('offline-capture-guard-' + uuid4().hex)
    plan = module.prepare(config, output, component_id=3)
    approved = tmp_path / 'approved-preview.json'
    approved.write_text(json.dumps(module.preview(plan)), encoding='utf-8')
    def forbidden():
        pytest.fail('invalid approval must not construct Native')
    monkeypatch.setattr(module, 'Native', forbidden)
    return ['--config', str(config), '--output', str(output), '--component-id', '3', '--execute',
            '--confirm', module.CONFIRM, '--expected-config-sha256', plan['config_sha256'],
            '--approved-preview', str(approved), '--approved-preview-sha256', hashlib.sha256(approved.read_bytes()).hexdigest(),
            '--authorization-expires-utc', '2000-01-01T00:00:00Z'], plan, approved


def test_expired_launch_rejected_before_native_and_directory_creation(launch_args):
    args, plan, _ = launch_args
    with pytest.raises(ValueError, match='authorization_expired'):
        module.main(args)
    assert not plan['output'].exists()


@pytest.mark.parametrize('value', ['bad', '2999-01-01T00:00:00', '2999-01-01T00:00:00+08:00'])
def test_approval_deadline_requires_explicit_utc(launch_args, value):
    args, plan, _ = launch_args
    args[-1] = value
    with pytest.raises(ValueError, match='invalid_authorization_expiry'):
        module.main(args)
    assert not plan['output'].exists()


@pytest.mark.parametrize('change', ['digest', 'runtime', 'component', 'boolean_component', 'port', 'config', 'episode'])
def test_approved_preview_changes_reject_before_native(launch_args, change):
    args, plan, approved = launch_args
    args[-1] = '2999-01-01T00:00:00Z'
    preview = json.loads(approved.read_text())
    if change == 'digest':
        args[args.index('--approved-preview-sha256') + 1] = '0' * 64
    elif change == 'episode':
        args[args.index('--output') + 1] = str(ROOT / '.tmp' / ('different-episode-' + uuid4().hex))
    else:
        if change == 'runtime':
            preview['runtime_hashes'] = {}
        elif change in ('component', 'boolean_component'):
            preview['component_id'] = 4 if change == 'component' else True
        elif change == 'port':
            preview['port'] = 1234
        else:
            preview['config_sha256'] = '0' * 64
        approved.write_text(json.dumps(preview))
        args[args.index('--approved-preview-sha256') + 1] = hashlib.sha256(approved.read_bytes()).hexdigest()
    with pytest.raises(ValueError, match='approved_preview'):
        module.main(args)
    assert not plan['output'].exists()


def test_cli_requires_approval_artifacts_even_with_confirm_and_config_hash(launch_args):
    args, plan, _ = launch_args
    args = args[:args.index('--approved-preview')]
    with pytest.raises(ValueError, match='approved_preview_and_expiry_required'):
        module.main(args)
    assert not plan['output'].exists()


def test_valid_cli_approval_reaches_only_fake_capture_and_reader(launch_args, monkeypatch):
    args, plan, approved = launch_args
    args[-1] = '2999-01-01T00:00:00Z'
    baseline = approved.parent / 'baseline.json'
    baseline.write_text(json.dumps(dict(reviewed_idle=True, reviewed_empty_filters=True,
                                        idle_status_sha256=module.digest_text('IDLE'),
                                        empty_filters_sha256=module.digest_text('EMPTY'))))
    args.extend(['--tool-baseline', str(baseline)])
    native = FakeNative()
    monkeypatch.setattr(module, 'Native', lambda: native)
    assert module.main(args) == 2  # Acquisition succeeds; explicit filter cleanup remains.
    report = json.loads((plan['output'] / 'execution.json').read_text())
    assert report['acquisition_complete'] and report['manual_filter_cleanup_required']
    assert sum(call[0] == 'readonly' for call in native.calls) == 1
    assert not report['hardware_stop_verified']


@pytest.mark.parametrize('expire_after', ['filter_add', 'capture_start'])
def test_expiry_during_preparation_never_connects_and_preserves_cleanup(launch_args, monkeypatch, expire_after):
    _, plan, _ = launch_args
    plan['baseline'] = dict(reviewed_idle=True, reviewed_empty_filters=True,
                           idle_status_sha256=module.digest_text('IDLE'),
                           empty_filters_sha256=module.digest_text('EMPTY'))
    expired = False
    native = FakeNative()
    original_command = native.command
    def command(args):
        nonlocal expired
        result = original_command(args)
        if (expire_after == 'filter_add' and args[:2] == ['filter', 'add']) or (
                expire_after == 'capture_start' and args[0] == 'start'):
            expired = True
        return result
    original_check = module.check_expiry
    def check(expiry):
        original_check(datetime(2000, 1, 1, tzinfo=UTC) if expired else expiry)
    native.command = command
    monkeypatch.setattr(module, 'check_expiry', check)
    report = module.execute(plan, native, confirm=module.CONFIRM,
                            authorization_expires=datetime.now(UTC) + timedelta(minutes=10))
    assert not native.child_started
    assert not report['acquisition_complete']
    assert report['manual_filter_cleanup_required']
    assert report['authorization_expired']
    assert sum(call[0] == 'start' for call in native.calls) == int(expire_after == 'capture_start')
    assert sum(call[0] == 'stop' for call in native.calls) == int(expire_after == 'capture_start')


def test_expiry_is_passed_to_the_fixed_readonly_child(launch_args):
    _, plan, _ = launch_args
    plan['baseline'] = dict(reviewed_idle=True, reviewed_empty_filters=True,
                           idle_status_sha256=module.digest_text('IDLE'),
                           empty_filters_sha256=module.digest_text('EMPTY'))
    native = FakeNative()
    expires = datetime.now(UTC) + timedelta(minutes=10)
    module.execute(plan, native, confirm=module.CONFIRM, authorization_expires=expires)
    child_args = next(call for call in native.calls if call[0] == 'readonly')
    assert child_args[child_args.index('--authorization-expires-utc') + 1] == expires.isoformat()


@pytest.mark.asyncio
async def test_actual_child_expiry_prevents_sdk_factory_and_preserves_report(tmp_path):
    from app.commissioning.readonly_state import collect_state
    config = tmp_path / 'child-readonly.yaml'
    config.write_text('real_robot:\n  mode: readonly\n  ip: 192.0.2.1\n')
    report_path = tmp_path / 'child-report.json'
    async def forbidden(_ip):
        pytest.fail('expired child must not enter SDK factory')
    result = await collect_state(config, report_path, client_factory=forbidden,
                                 authorization_expires_utc='2000-01-01T00:00:00Z')
    assert result == 2
    report = json.loads(report_path.read_text())
    assert not report['complete'] and report['authorization_expired']
    assert report['connection_attempted'] is False
    assert report['reads'] == []


@pytest.mark.skipif(os.name != 'nt', reason='Windows launcher')
@pytest.mark.parametrize('valid', [True, False])
def test_actual_windows_launcher_preserves_preview_result_and_error(tmp_path, valid):
    config = tmp_path / 'config with spaces.yaml'
    config.write_text('real_robot:\n  mode: readonly\n  ip: 192.0.2.1\n' if valid else 'bad config')
    unique = uuid4().hex
    record = ROOT / '.tmp' / ('offline-launch-record-' + unique)
    capture = ROOT / '.tmp' / ('offline-launch-capture-' + unique)
    result = subprocess.run(['powershell.exe', '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File',
                             str(ROOT / 'scripts/launch_readonly_wire_capture.ps1'),
                             '-Config', str(config), '-Output', str(capture), '-ComponentId', '3',
                             '-RecordDirectory', str(record)], capture_output=True, timeout=30)
    assert (result.returncode == 0) is valid
    metadata = json.loads((record / 'launch.json').read_text(encoding='utf-8-sig'))
    assert metadata['mode'] == 'offline_preview'
    assert metadata['python_exit'] == result.returncode
    assert metadata['process_exited'] is True
    assert not capture.exists()
    if valid:
        assert json.loads((record / 'stdout.txt').read_text(encoding='utf-8-sig'))['mode'] == 'offline_preview'
    else:
        assert 'readonly_configuration_required' in (record / 'stderr.txt').read_text(encoding='utf-8-sig')
