"""Synthetic clients only: never imports or connects to the native robot SDK."""
import asyncio
import builtins
from datetime import UTC, datetime, timedelta
import importlib.util
import hashlib
import json
from pathlib import Path
from unittest.mock import AsyncMock

import pytest
import yaml

from tests.config.test_real_robot_config import REAL_CONFIG_TEMPLATE
from tests.robots.fake_lebai import FakeLebaiClient

SOURCE = Path(__file__).resolve().parents[3] / 'scripts/capture_ready_pose.py'


@pytest.fixture
def context(tmp_path, monkeypatch):
    spec = importlib.util.spec_from_file_location('ready_pose_entry', SOURCE)
    entry = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(entry)
    monkeypatch.setattr(entry, 'ROOT', tmp_path)
    monkeypatch.setattr(entry, 'runtime_hashes', lambda: {'synthetic': 'a' * 64})
    monkeypatch.setattr(entry, 'existing_rpc_connections', lambda: 0)
    config = tmp_path / 'readonly.yaml'
    config.write_text(REAL_CONFIG_TEMPLATE.format(mode='readonly'))
    now = datetime.now(UTC)
    site = dict(confirmed_utc=now.isoformat(), target_ip='192.168.10.20',
                robot_stationary=True, no_concurrent_control=True,
                operator_selected_ready_pose=True)
    plan = entry.prepare(config, tmp_path / '.tmp/once', issued=now.isoformat(),
                         expires=(now + timedelta(minutes=5)).isoformat(), site=site)
    approved = tmp_path / 'preview.json'
    approved.write_text(json.dumps(entry.preview(plan)))
    kwargs = dict(approved=approved, expected_hash=entry.sha(approved), confirm=entry.CONFIRM)
    return entry, plan, kwargs


def test_preview_has_no_sdk_import_init_network_or_output_reservation(context, monkeypatch, capsys):
    entry, plan, _ = context
    original = builtins.__import__
    def reject_sdk(name, *args, **kwargs):
        if name.startswith('lebai_sdk'):
            raise AssertionError('native SDK import forbidden in preview')
        return original(name, *args, **kwargs)
    monkeypatch.setattr(builtins, '__import__', reject_sdk)
    monkeypatch.setattr(entry, 'existing_rpc_connections', lambda: pytest.fail('preview inspected RPC table'))
    monkeypatch.setattr(entry, 'execute', AsyncMock(side_effect=AssertionError('preview executed')))
    assert entry.main(['--config', str(plan['config']), '--output', str(plan['output'])]) == 0
    value = json.loads(capsys.readouterr().out)
    assert value['allowed_writes'] == []
    assert value['allowed_queries'] == ['is_connected', 'get_robot_state', 'get_estop_reason', 'get_kin_data', 'get_tcp']
    assert value['max_total_calls_including_connect'] == 13
    assert value['site_confirmation'] is None and value['authorization_issued_utc'] is None
    assert not value['motion_authorized'] and not plan['output'].exists()


@pytest.mark.parametrize('state', ['IDLE', 'PAUSED', 5, 6])
async def test_operator_selected_pose_capture_is_one_connection_and_only_fixed_reads(context, state):
    entry, plan, kwargs = context
    client = FakeLebaiClient()
    client.robot_state = state
    factory = AsyncMock(return_value=client)
    result = await entry.execute(plan, **kwargs, client_factory=factory)
    assert result['complete'] and not result['motion_authorized']
    assert result['runtime'] == 'SIMULATION'
    pose = result['operator_selected_ready_pose']
    assert pose['selection'] == 'operator_selected_current_working_ready_pose'
    assert pose['q_rad'] == client.kin_data['actual_joint_pose']
    assert pose['actual_tcp_pose'] == client.kin_data['actual_tcp_pose']
    assert pose['tool_tcp_setting'] == client.tcp
    assert pose['source']['approved_preview_sha256'] == kwargs['expected_hash']
    assert pose['source']['config_sha256'] == plan['config_sha256']
    assert pose['source']['telemetry_sha256'] == hashlib.sha256(json.dumps(
        dict(initial=result['initial'], final=result['final']), sort_keys=True,
        allow_nan=False, separators=(',', ':')).encode('utf-8')).hexdigest()
    assert result['native_connection_release'] == 'standalone_process_exit_required_not_verified'
    assert set(client.read_calls) <= set(entry.READ_METHODS) and client.write_calls == []
    assert len(result['calls']) == 13 and factory.await_count == 1
    assert json.loads((plan['output'] / 'result.json').read_text()) == result
    with pytest.raises(FileExistsError):
        await entry.execute(plan, **kwargs, client_factory=factory)
    factory.assert_awaited_once()


@pytest.mark.parametrize('change', ['confirm', 'hash', 'scope', 'config', 'runtime', 'rpc', 'expired',
                                  'site', 'site_ip', 'future_site', 'missing_issued', 'future_issued'])
async def test_bad_admission_never_reserves_output_or_connects(context, monkeypatch, change):
    entry, plan, kwargs = context
    if change == 'confirm': kwargs['confirm'] = 'yes'
    elif change == 'hash': kwargs['expected_hash'] = 'b' * 64
    elif change == 'scope':
        value = json.loads(kwargs['approved'].read_text())
        value['allowed_writes'] = ['stop_move']
        kwargs['approved'].write_text(json.dumps(value))
        kwargs['expected_hash'] = entry.sha(kwargs['approved'])
    elif change == 'config': plan['config'].write_text(plan['config'].read_text() + '\n# changed')
    elif change == 'runtime': monkeypatch.setattr(entry, 'runtime_hashes', lambda: {'synthetic': 'b' * 64})
    elif change == 'rpc': monkeypatch.setattr(entry, 'existing_rpc_connections', lambda: 1)
    else:
        if change == 'expired': plan['authorization_expires_utc'] = '2000-01-01T00:00:00Z'
        elif change == 'site': plan['site_confirmation']['no_concurrent_control'] = False
        elif change == 'site_ip': plan['site_confirmation']['target_ip'] = '192.168.10.21'
        elif change == 'future_site': plan['site_confirmation']['confirmed_utc'] = '2099-01-01T00:00:00Z'
        elif change == 'missing_issued': plan['authorization_issued_utc'] = None
        else: plan['authorization_issued_utc'] = '2099-01-01T00:00:00Z'
        kwargs['approved'].write_text(json.dumps(entry.preview(plan)))
        kwargs['expected_hash'] = entry.sha(kwargs['approved'])
    factory = AsyncMock()
    with pytest.raises(ValueError):
        await entry.execute(plan, **kwargs, client_factory=factory)
    factory.assert_not_called()
    assert not plan['output'].exists()


@pytest.mark.parametrize('change', ['MOVING', 'unknown', 'estop', 'unknown_estop', 'disconnected',
                                  'qd', 'qdd', 'tiny_qd', 'tiny_qdd', 'missing_qdd', 'bad_q', 'nan_tcp',
                                  'bool_q', 'changed_q', 'changed_tcp', 'changed_tool', 'changed_state'])
async def test_bad_or_nonstatic_feedback_does_not_publish_ready_pose_or_write(context, change):
    entry, plan, kwargs = context
    client = FakeLebaiClient()
    if change in ('MOVING', 'unknown'): client.robot_state = change
    elif change == 'estop': client.estop_reason = 4
    elif change == 'unknown_estop': client.estop_reason = None
    elif change == 'disconnected': client.connected = False
    elif change in ('qd', 'tiny_qd'): client.kin_data['actual_joint_speed'][0] = 1e-12 if change == 'tiny_qd' else .1
    elif change in ('qdd', 'tiny_qdd'): client.kin_data['actual_joint_acc'][0] = 1e-12 if change == 'tiny_qdd' else .1
    elif change == 'missing_qdd': del client.kin_data['actual_joint_acc']
    elif change == 'bad_q': client.kin_data['actual_joint_pose'] = [0] * 5
    elif change == 'bool_q': client.kin_data['actual_joint_pose'][0] = True
    elif change == 'nan_tcp': client.kin_data['actual_tcp_pose']['x'] = float('nan')
    else:
        original = client.get_kin_data
        count = 0
        async def changed():
            nonlocal count
            count += 1
            if count == 2:
                if change == 'changed_q': client.kin_data['actual_joint_pose'][0] += .001
                elif change == 'changed_tcp': client.kin_data['actual_tcp_pose']['x'] += .001
                elif change == 'changed_tool': client.tcp['x'] += .001
                else: client.robot_state = 'PAUSED'
            return await original()
        client.get_kin_data = changed
    result = await entry.execute(plan, **kwargs, client_factory=AsyncMock(return_value=client))
    assert not result['complete'] and 'operator_selected_ready_pose' not in result
    assert client.write_calls == [] and set(client.read_calls) <= set(entry.READ_METHODS)


async def test_failure_is_redacted_and_consumes_attempt(context):
    entry, plan, kwargs = context
    factory = AsyncMock(side_effect=RuntimeError('private-vendor-message'))
    result = await entry.execute(plan, **kwargs, client_factory=factory)
    assert not result['complete'] and 'private-vendor-message' not in json.dumps(result)
    with pytest.raises(FileExistsError):
        await entry.execute(plan, **kwargs, client_factory=factory)
    factory.assert_awaited_once()


async def test_cancelled_connection_persists_unknown_remote_result_without_cleanup(context):
    entry, plan, kwargs = context
    started = asyncio.Event()
    async def blocked(ip):
        started.set()
        await asyncio.Event().wait()
    task = asyncio.create_task(entry.execute(plan, **kwargs, client_factory=blocked))
    await started.wait()
    task.cancel()
    with pytest.raises(asyncio.CancelledError): await task
    result = json.loads((plan['output'] / 'result.json').read_text())
    assert not result['complete'] and result['pending_remote_result_unknown']
    assert result['error'] == 'interrupted'


async def test_runtime_change_during_read_ends_chain_without_pose(context, monkeypatch):
    entry, plan, kwargs = context
    client = FakeLebaiClient()
    original = client.get_robot_state
    async def changed():
        value = await original()
        monkeypatch.setattr(entry, 'runtime_hashes', lambda: {'synthetic': 'b' * 64})
        return value
    client.get_robot_state = changed
    result = await entry.execute(plan, **kwargs, client_factory=AsyncMock(return_value=client))
    assert not result['complete'] and 'get_kin_data' not in client.read_calls and client.write_calls == []


@pytest.mark.parametrize('change', ['public_ip', 'mode', 'simulator', 'output'])
def test_prepare_rejects_inappropriate_config_or_output(context, change):
    entry, plan, _ = context
    data = yaml.safe_load(plan['config'].read_text())
    if change == 'public_ip': data['real_robot']['ip'] = '8.8.8.8'
    elif change == 'mode': data['real_robot']['mode'] = 'control'
    elif change == 'simulator': data['backend'] = 'simulator'
    plan['config'].write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        entry.prepare(plan['config'], plan['config'].parent / 'outside' if change == 'output' else plan['output'])


@pytest.mark.parametrize('change', ['runtime', 'deadline'])
async def test_local_table_cannot_outlive_connection_admission(context, monkeypatch, change):
    entry, plan, kwargs = context
    clock = [1_000_000_000]
    monkeypatch.setattr(entry, 'monotonic_ns', lambda: clock[0])
    count = 0
    def table():
        nonlocal count
        count += 1
        if count == 2:
            if change == 'runtime': monkeypatch.setattr(entry, 'runtime_hashes', lambda: {'synthetic': 'b' * 64})
            else: clock[0] += int(entry.CONNECT_TIMEOUT_S * 1e9)
        return 0
    monkeypatch.setattr(entry, 'existing_rpc_connections', table)
    factory = AsyncMock(return_value=FakeLebaiClient())
    result = await entry.execute(plan, **kwargs, client_factory=factory)
    factory.assert_not_called()
    assert not result['complete']


async def test_native_init_deadline_expiry_cannot_enter_connect(context, monkeypatch):
    from types import SimpleNamespace
    entry, plan, kwargs = context
    clock = [1_000_000_000]
    monkeypatch.setattr(entry, 'monotonic_ns', lambda: clock[0])
    def init(): clock[0] += int(entry.CONNECT_TIMEOUT_S * 1e9)
    fake_sdk = SimpleNamespace(init=init, connect=AsyncMock(return_value=FakeLebaiClient()))
    monkeypatch.setattr(entry, 'verify_sdk_source', lambda: None)
    monkeypatch.setattr(entry.importlib, 'import_module', lambda name: fake_sdk)
    result = await entry.execute(plan, **kwargs)
    fake_sdk.connect.assert_not_called()
    assert result['native_sdk_init_entered'] and not result['native_sdk_connect_entered']
    assert not result['complete']


async def test_read_timeout_ends_queries_and_does_not_publish_pose(context, monkeypatch):
    entry, plan, kwargs = context
    monkeypatch.setattr(entry, 'READ_TIMEOUT_S', .01)
    # Timing is scope bound, so reapprove the synthetic preview after altering it.
    kwargs['approved'].write_text(json.dumps(entry.preview(plan)))
    kwargs['expected_hash'] = entry.sha(kwargs['approved'])
    client = FakeLebaiClient()
    async def blocked(): await asyncio.Event().wait()
    client.get_kin_data = blocked
    result = await entry.execute(plan, **kwargs, client_factory=AsyncMock(return_value=client))
    assert not result['complete'] and result['error'] == 'capture_query_timeout'
    assert result['pending_remote_result_unknown']
    assert 'get_tcp' not in client.read_calls and client.write_calls == []


def test_cli_process_exits_with_cancellation_resistant_synthetic_connection(context, tmp_path):
    import subprocess
    import sys
    entry, plan, kwargs = context
    # Isolated interpreter verifies the standalone loop does not await a native-
    # like task that ignores cancellation. Every SDK-facing object is synthetic.
    script = tmp_path / 'stubborn_fake.py'
    script.write_text('''
import asyncio, importlib.util, sys
from pathlib import Path
spec = importlib.util.spec_from_file_location('entry', sys.argv[1])
entry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(entry)
entry.ROOT = Path(sys.argv[2])
entry.runtime_hashes = lambda: {'synthetic': 'a' * 64}
entry.existing_rpc_connections = lambda: 0
entry.CONNECT_TIMEOUT_S = .02
async def stubborn(ip):
    try:
        await asyncio.Event().wait()
    except asyncio.CancelledError:
        await asyncio.Event().wait()
original = entry.execute
def execute(*args, **kwargs):
    return original(*args, **kwargs, client_factory=stubborn)
entry.execute = execute
entry.sys.executable = str(entry.ROOT / '.venv/Scripts/python.exe')
raise SystemExit(entry.main(sys.argv[3:]))
''')
    entry.CONNECT_TIMEOUT_S = .02
    kwargs['approved'].write_text(json.dumps(entry.preview(plan)))
    kwargs['expected_hash'] = entry.sha(kwargs['approved'])
    site = plan['site_confirmation']
    argv = [sys.executable, str(script), str(SOURCE), str(entry.ROOT), '--config', str(plan['config']),
            '--output', str(plan['output']), '--execute-once', '--approved-preview', str(kwargs['approved']),
            '--approved-preview-sha256', kwargs['expected_hash'], '--authorization-issued-utc',
            plan['authorization_issued_utc'], '--authorization-expires-utc', plan['authorization_expires_utc'],
            '--site-confirmed-utc', site['confirmed_utc'], '--confirmed-target-ip', site['target_ip'],
            '--confirm-robot-stationary', '--confirm-no-concurrent-control',
            '--confirm-operator-selected-ready-pose', '--confirm', kwargs['confirm']]
    result = subprocess.run(argv, capture_output=True, text=True, timeout=8)
    assert result.returncode == 2, result.stderr
    saved = json.loads((plan['output'] / 'result.json').read_text())
    assert not saved['complete'] and saved['pending_remote_result_unknown'] and saved['runtime'] == 'SIMULATION'
