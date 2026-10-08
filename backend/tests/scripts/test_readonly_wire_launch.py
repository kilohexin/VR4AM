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


@pytest.mark.skipif(os.name != 'nt', reason='Windows launcher')
@pytest.mark.parametrize('valid', [True, False])
def test_actual_windows_motion_switch_only_previews_fixed_scope_and_binds_baseline(tmp_path, valid):
    import yaml
    from tests.config.test_real_robot_config import REAL_CONFIG_TEMPLATE
    spec = importlib.util.spec_from_file_location('motion_preview_launch', ROOT/'scripts/motion_stop_wire_capture.py')
    motion = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(motion)
    data = yaml.safe_load(REAL_CONFIG_TEMPLATE.format(mode='control'))
    data['real_robot']['ip'] = '192.0.2.1'
    data['real_robot']['control'].update(motion.CAPS)
    if not valid:
        data['real_robot']['control']['max_joint_speed_radps'] = .5
    config = tmp_path/'motion config with spaces.yaml'
    config.write_text(yaml.safe_dump(data), encoding='utf-8')
    baseline = tmp_path/'reviewed baseline.json'
    baseline_data = dict(reviewed_idle=True, reviewed_empty_filters=True,
                         idle_status_sha256='1'*64, empty_filters_sha256='2'*64)
    baseline.write_text(json.dumps(baseline_data), encoding='utf-8')
    unique = uuid4().hex
    record = ROOT/'.tmp'/('offline-motion-launch-record-'+unique)
    capture = ROOT/'.tmp'/('offline-motion-launch-capture-'+unique)
    result = subprocess.run(['powershell.exe', '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File',
                             str(ROOT/'scripts/launch_readonly_wire_capture.ps1'), '-MotionStop',
                             '-Config', str(config), '-Output', str(capture), '-ComponentId', '3',
                             '-RecordDirectory', str(record), '-ToolBaseline', str(baseline)],
                            capture_output=True, timeout=30)
    assert (result.returncode == 0) is valid
    metadata = json.loads((record/'launch.json').read_text(encoding='utf-8-sig'))
    assert metadata['mode'] == 'motion_stop_offline_preview'
    assert metadata['python_exit'] == result.returncode and metadata['process_exited']
    assert not capture.exists()
    if valid:
        report = json.loads((record/'stdout.txt').read_text(encoding='utf-8-sig'))
        assert report['mode'] == 'motion_stop_wire_offline_preview'
        assert report['action'] == motion.ACTION
        assert report['tool_baseline_sha256'] == hashlib.sha256(json.dumps(baseline_data, sort_keys=True).encode()).hexdigest()
        assert report['runtime_hashes']['scripts/launch_readonly_wire_capture.ps1']
        assert report['hardware_stop_verified'] is False
    else:
        assert 'diagnostic_cap_invalid' in (record/'stderr.txt').read_text(encoding='utf-8-sig')


@pytest.mark.skipif(os.name != 'nt', reason='Windows launcher')
def test_motion_launcher_forwards_attention_while_harmless_stub_still_runs(tmp_path):
    """Isolated launcher copy; child is a stdlib-only stub, never robot code."""
    import shutil
    root = tmp_path/'isolated'
    scripts = root/'scripts'
    scripts.mkdir(parents=True)
    python_dir = root/'.venv/Scripts'
    python_dir.mkdir(parents=True)
    shutil.copyfile(ROOT/'.venv/Scripts/python.exe', python_dir/'python.exe')
    shutil.copyfile(ROOT/'.venv/pyvenv.cfg', root/'.venv/pyvenv.cfg')
    launcher = scripts/'launch_readonly_wire_capture.ps1'
    shutil.copyfile(ROOT/'scripts/launch_readonly_wire_capture.ps1', launcher)
    (scripts/'local_rpc_connections.py').write_text('print(0)\n', encoding='utf-8')
    (scripts/'motion_stop_wire_capture.py').write_text(
        "import pathlib,sys,time\np=pathlib.Path(sys.argv[sys.argv.index('--output')+1])\n"
        "p.mkdir(parents=True)\n(p/'attention-required.json').write_text('{}')\n"
        "time.sleep(2)\n(p/'stub-completed').write_text('done')\nsys.exit(2)\n", encoding='utf-8')
    # Replace the local connection-table command only; real launcher process,
    # redirection, polling, warning and exit-code handling remain exercised.
    driver = root/'driver.ps1'
    driver.write_text("function Get-NetTCPConnection { return @() }\n"
                      ". '"+str(launcher)+"' -MotionStop -Execute -Config 'stub.yaml' "
                      "-Output '"+str(root/'.tmp/capture')+"' -ComponentId 3 -RecordDirectory '"+
                      str(root/'.tmp/launch')+"' -ToolBaseline 'stub.json' -ExpectedConfigSha256 'stub' "
                      "-ApprovedPreview 'stub.json' -ApprovedPreviewSha256 'stub' "
                      "-AuthorizationExpiresUtc 'stub' -Confirm 'I_APPROVE_ONE_Z3MM_MOTION_STOP_WIRE_CAPTURE'\nexit $LASTEXITCODE\n",
                      encoding='utf-8')
    child = subprocess.Popen(['powershell.exe', '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', str(driver)],
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    seen_while_running = False
    for line in child.stdout:
        if b'VR4ARM_STOP_UNKNOWN' in line:
            seen_while_running = not (root/'.tmp/capture/stub-completed').exists()
            break
    stdout, stderr = child.communicate(timeout=30)
    assert child.returncode == 2, stderr.decode(errors='replace')
    assert seen_while_running, stdout.decode(errors='replace')
    record = json.loads((root/'.tmp/launch/launch.json').read_text(encoding='utf-8-sig'))
    assert record['operator_attention_required'] and record['python_exit'] == 2
    assert record['process_exited']


@pytest.mark.skipif(os.name != 'nt', reason='Windows launcher')
@pytest.mark.parametrize('child_exit', [0, 2])
def test_post_child_table_failure_cannot_report_success(tmp_path, child_exit):
    """Both subprocesses are local stdlib stubs; no SDK or system query."""
    import shutil
    root = tmp_path/'isolated-post-query'
    scripts = root/'scripts'
    scripts.mkdir(parents=True)
    python_dir = root/'.venv/Scripts'
    python_dir.mkdir(parents=True)
    shutil.copyfile(ROOT/'.venv/Scripts/python.exe', python_dir/'python.exe')
    shutil.copyfile(ROOT/'.venv/pyvenv.cfg', root/'.venv/pyvenv.cfg')
    launcher = scripts/'launch_readonly_wire_capture.ps1'
    shutil.copyfile(ROOT/'scripts/launch_readonly_wire_capture.ps1', launcher)
    (scripts/'local_rpc_connections.py').write_text(
        "import pathlib,sys\np=pathlib.Path(__file__).with_suffix('.count')\n"
        "if p.exists(): sys.exit(1)\np.write_text('queried')\nprint(0)\n", encoding='utf-8')
    (scripts/'motion_stop_wire_capture.py').write_text(
        f'import sys\nsys.exit({child_exit})\n', encoding='utf-8')
    result = subprocess.run([
        'powershell.exe', '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', str(launcher),
        '-MotionStop', '-Execute', '-Config', 'stub.yaml', '-Output', str(root/'.tmp/capture'),
        '-ComponentId', '3', '-RecordDirectory', str(root/'.tmp/launch'),
        '-ToolBaseline', 'stub.json', '-ExpectedConfigSha256', 'stub',
        '-ApprovedPreview', 'stub.json', '-ApprovedPreviewSha256', 'stub',
        '-AuthorizationExpiresUtc', 'stub', '-Confirm', 'I_APPROVE_ONE_Z3MM_MOTION_STOP_WIRE_CAPTURE'],
        capture_output=True, timeout=30)
    record = json.loads((root/'.tmp/launch/launch.json').read_text(encoding='utf-8-sig'))
    assert result.returncode == (child_exit or 1)
    assert record['python_exit'] == child_exit and record['process_exited']
    assert record['rpc_connections_after_status'] == 'unknown'
    assert 'rpc_connections_after' not in record
    assert record['launcher_exit'] == result.returncode
    assert record['error_type']
    assert not (root/'.tmp/capture').exists()
