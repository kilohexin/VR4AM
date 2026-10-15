from datetime import UTC, datetime, timedelta
import importlib.util
import json
from pathlib import Path
from unittest.mock import AsyncMock

import pytest
import yaml

from tests.config.test_real_robot_config import REAL_CONFIG_TEMPLATE
from tests.commissioning.test_readonly_kinematics import SurveyClient

SOURCE = Path(__file__).resolve().parents[3] / 'scripts/real_robot_readonly_kinematics.py'


@pytest.fixture
def context(tmp_path, monkeypatch):
    spec = importlib.util.spec_from_file_location('kinematics_entry', SOURCE)
    entry = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(entry)
    monkeypatch.setattr(entry, 'ROOT', tmp_path)
    monkeypatch.setattr(entry, 'runtime_hashes', lambda: {'synthetic': 'a' * 64})
    monkeypatch.setattr(entry, 'existing_rpc_connections', lambda: 0)
    data = yaml.safe_load(REAL_CONFIG_TEMPLATE.format(mode='readonly'))
    data['real_robot']['control']['max_joint_speed_radps'] = .05
    data['real_robot']['control']['max_joint_acceleration_radps2'] = .2
    config = tmp_path / 'readonly.yaml'
    config.write_text(yaml.safe_dump(data))
    expires = (datetime.now(UTC) + timedelta(minutes=5)).isoformat()
    plan = entry.prepare(config, tmp_path / '.tmp/once', expires=expires)
    approved = tmp_path / 'preview.json'
    approved.write_text(json.dumps(entry.preview(plan)))
    kwargs = dict(approved=approved, expected_hash=entry.sha(approved), expires=expires, confirm=entry.CONFIRM)
    return entry, plan, kwargs


def test_preview_never_executes_or_reserves_output(context, monkeypatch, capsys):
    entry, plan, _ = context
    monkeypatch.setattr(entry, 'execute', AsyncMock(side_effect=AssertionError('must not execute')))
    assert entry.main(['--config', str(plan['config']), '--output', str(plan['output'])]) == 0
    data = json.loads(capsys.readouterr().out)
    assert data['allowed_writes'] == [] and not data['motion_authorized']
    assert data['computational_target'] == {'axis': 'z', 'distance_m': .0002, 'executed': False}
    assert data['max_fk_calls'] == 4 and data['allowed_raw_states'] == ['IDLE', 'PAUSED']
    assert not plan['output'].exists()


def test_preview_binds_a_smaller_configured_computational_target(context):
    entry,plan,_=context
    data=yaml.safe_load(plan['config'].read_text())
    data['real_robot']['control']['max_tcp_step_m']=.00005
    plan['config'].write_text(yaml.safe_dump(data))
    new=entry.prepare(plan['config'],plan['output'],expires=plan['authorization_expires_utc'])
    assert entry.preview(new)['computational_target']==dict(axis='z',distance_m=.00005,executed=False)


@pytest.mark.parametrize('change', [
    {'confirm': 'yes'}, {'approved': None}, {'expected_hash': 'b' * 64},
    {'expires': '2000-01-01T00:00:00Z'}, {'expires': '2099-01-01T00:00:00Z'},
    {'expires': 'invalid'}, {'expires': None},
])
async def test_invalid_admission_cannot_connect_or_reserve(context, change):
    entry, plan, kwargs = context
    factory = AsyncMock()
    with pytest.raises(ValueError):
        await entry.execute(plan, **{**kwargs, **change}, client_factory=factory)
    factory.assert_not_called()
    assert not plan['output'].exists()


@pytest.mark.parametrize('change', ['config', 'runtime', 'scope', 'rpc'])
async def test_changed_scope_or_inputs_cannot_connect(context, monkeypatch, change):
    entry, plan, kwargs = context
    if change == 'config':
        plan['config'].write_text(plan['config'].read_text() + '\n# changed')
    elif change == 'runtime':
        monkeypatch.setattr(entry, 'runtime_hashes', lambda: {'synthetic': 'b' * 64})
    elif change == 'rpc':
        monkeypatch.setattr(entry, 'existing_rpc_connections', lambda: 1)
    else:
        value = json.loads(kwargs['approved'].read_text())
        value['allowed_writes'] = ['stop_move']
        kwargs['approved'].write_text(json.dumps(value))
        kwargs['expected_hash'] = entry.sha(kwargs['approved'])
    factory = AsyncMock()
    with pytest.raises(ValueError):
        await entry.execute(plan, **kwargs, client_factory=factory)
    factory.assert_not_called()
    assert not plan['output'].exists()


@pytest.mark.parametrize('change', ['control_mode', 'simulator', 'cap', 'public_ip', 'output'])
def test_prepare_rejects_inappropriate_inputs(context, change):
    entry, plan, _ = context
    data = yaml.safe_load(plan['config'].read_text())
    output = plan['output']
    if change == 'control_mode':
        data['real_robot']['mode'] = 'control'
    elif change == 'simulator':
        data['backend'] = 'simulator'
    elif change == 'cap':
        data['real_robot']['control']['max_joint_speed_radps'] = .06
    elif change == 'public_ip':
        data['real_robot']['ip'] = '8.8.8.8'
    else:
        output = plan['config'].parent / 'outside'
    plan['config'].write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        entry.prepare(plan['config'], output)


async def test_one_attempt_persists_evidence_and_no_control_cleanup(context):
    entry, plan, kwargs = context
    client = SurveyClient()
    client.robot_state = 'PAUSED'
    factory = AsyncMock(return_value=client)
    result = await entry.execute(plan, **kwargs, client_factory=factory)
    assert result['complete'] and result['kinematics_consistent']
    assert not result['motion_authorized'] and result['runtime'] == 'SIMULATION'
    saved = json.loads((plan['output'] / 'result.json').read_text())
    assert saved == result
    assert client.write_calls == [] and factory.await_count == 1
    with pytest.raises(FileExistsError):
        await entry.execute(plan, **kwargs, client_factory=factory)
    assert factory.await_count == 1


async def test_failed_connection_consumes_attempt(context):
    entry, plan, kwargs = context
    factory = AsyncMock(side_effect=RuntimeError('secret-token'))
    result = await entry.execute(plan, **kwargs, client_factory=factory)
    assert not result['complete'] and 'secret-token' not in json.dumps(result)
    with pytest.raises(FileExistsError):
        await entry.execute(plan, **kwargs, client_factory=factory)
    factory.assert_awaited_once()


async def test_changed_inputs_during_read_end_query_chain(context, monkeypatch):
    entry, plan, kwargs = context
    client = SurveyClient()
    original = client.get_robot_state

    async def changed():
        result = await original()
        monkeypatch.setattr(entry, 'runtime_hashes', lambda: {'synthetic': 'b' * 64})
        return result

    client.get_robot_state = changed
    result = await entry.execute(plan, **kwargs, client_factory=AsyncMock(return_value=client))
    assert not result['complete'] and 'kinematics_inverse' not in client.read_calls
    assert client.write_calls == []


async def test_cancellation_keeps_incomplete_saved_report(context):
    import asyncio
    entry, plan, kwargs = context
    started = asyncio.Event()

    async def blocked(ip):
        started.set()
        await asyncio.Event().wait()

    task = asyncio.create_task(entry.execute(plan, **kwargs, client_factory=blocked))
    await started.wait()
    task.cancel()
    with pytest.raises(asyncio.CancelledError):
        await task
    result = json.loads((plan['output'] / 'result.json').read_text())
    assert not result['complete'] and result['error'] == 'interrupted'
    assert result['pending_remote_result_unknown'] and not result['motion_authorized']


def test_main_keyboard_interrupt_saves_report_before_loop_closes(context, monkeypatch):
    import asyncio
    entry, plan, kwargs = context
    class InterruptLoop:
        def __init__(self):
            self.inner = asyncio.new_event_loop()
            self.first = True

        def create_task(self, awaitable):
            return self.inner.create_task(awaitable)

        def run_until_complete(self, task):
            if self.first:
                self.first = False
                self.inner.call_soon(self.inner.stop)
                self.inner.run_forever()
                raise KeyboardInterrupt
            return self.inner.run_until_complete(task)

        def close(self):
            self.inner.close()

    original_execute = entry.execute
    def injected(*args, **kw):
        return original_execute(*args, **kw, client_factory=AsyncMock(return_value=SurveyClient()))
    loop = InterruptLoop()
    monkeypatch.setattr(entry, 'execute', injected)
    monkeypatch.setattr(entry.asyncio, 'new_event_loop', lambda: loop)
    monkeypatch.setattr(entry.sys, 'executable', str(entry.ROOT / '.venv/Scripts/python.exe'))
    argv = ['--config', str(plan['config']), '--output', str(plan['output']), '--execute-once',
            '--approved-preview', str(kwargs['approved']), '--approved-preview-sha256', kwargs['expected_hash'],
            '--authorization-expires-utc', kwargs['expires'], '--confirm', kwargs['confirm']]
    assert entry.main(argv) == 130
    result = json.loads((plan['output'] / 'result.json').read_text())
    assert not result['complete'] and result['error'] == 'interrupted'


@pytest.mark.parametrize('change', ['entry', 'extension', 'missing', 'already_loaded'])
def test_unbound_native_source_is_rejected_without_import(context, monkeypatch, change):
    from types import SimpleNamespace
    entry, _, _ = context
    package = (entry.ROOT / '.venv/Lib/site-packages/lebai_sdk').resolve()
    def resolve(name, paths):
        if change == 'missing':
            return None
        if name == 'lebai_sdk':
            origin = package / ('other.py' if change == 'entry' else '__init__.py')
            return SimpleNamespace(origin=str(origin), submodule_search_locations=[str(package)])
        origin = package / ('other.pyd' if change == 'extension' else 'lebai_sdk.pyd')
        loader = entry.importlib.machinery.ExtensionFileLoader(name, str(origin))
        return SimpleNamespace(origin=str(origin), loader=loader)
    monkeypatch.setattr(entry.importlib.machinery.PathFinder, 'find_spec', resolve)
    if change == 'already_loaded':
        monkeypatch.setitem(entry.sys.modules, 'lebai_sdk', object())
    with pytest.raises(ValueError):
        entry.verify_sdk_source()


def test_cli_process_exits_with_stubborn_fake_connection(context, tmp_path):
    import subprocess
    import sys
    entry, plan, kwargs = context
    # A separate fake-only process checks actual interpreter shutdown, not merely
    # returning from an async function. Its fixture bypasses live admission only
    # to use the synthetic config and fake factory. No native SDK import.
    script = tmp_path / 'stubborn_fake.py'
    script.write_text('''
import asyncio, importlib.util, json, sys
from pathlib import Path
spec = importlib.util.spec_from_file_location('entry', sys.argv[1])
entry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(entry)
entry.ROOT = Path(sys.argv[2])
entry.runtime_hashes = lambda: {'synthetic': 'a' * 64}
entry.existing_rpc_connections = lambda: 0
from app.commissioning import readonly_kinematics as survey
survey.CONNECT_TIMEOUT_S = .02
async def stubborn(ip):
    try:
        await asyncio.Event().wait()
    except asyncio.CancelledError:
        await asyncio.Event().wait()
original = entry.execute
def execute(*args, **kw):
    return original(*args, **kw, client_factory=stubborn)
entry.execute = execute
entry.sys.executable = str(entry.ROOT / '.venv/Scripts/python.exe')
raise SystemExit(entry.main(sys.argv[3:]))
''')
    argv = [sys.executable, str(script), str(SOURCE), str(entry.ROOT), '--config', str(plan['config']),
            '--output', str(plan['output']), '--execute-once', '--approved-preview', str(kwargs['approved']),
            '--approved-preview-sha256', kwargs['expected_hash'], '--authorization-expires-utc', kwargs['expires'],
            '--confirm', kwargs['confirm']]
    result = subprocess.run(argv, capture_output=True, text=True, timeout=8)
    assert result.returncode == 2
    saved = json.loads((plan['output'] / 'result.json').read_text())
    assert not saved['complete'] and saved['pending_remote_result_unknown']
    assert saved['runtime'] == 'SIMULATION' and not saved['motion_authorized']


@pytest.mark.parametrize('change', ['scope', 'deadline'])
async def test_permission_and_connect_deadline_rechecked_after_local_table(context, monkeypatch, change):
    from app.commissioning import readonly_kinematics as survey
    entry, plan, kwargs = context
    count = 0
    clock = [1_000_000_000]
    monkeypatch.setattr(survey, 'monotonic_ns', lambda: clock[0])
    monkeypatch.setattr(entry, 'monotonic_ns', lambda: clock[0], raising=False)
    def local_table():
        nonlocal count
        count += 1
        if count == 2:
            if change == 'scope':
                monkeypatch.setattr(entry, 'runtime_hashes', lambda: {'synthetic': 'b' * 64})
            else:
                clock[0] += 5_000_000_000
        return 0
    monkeypatch.setattr(entry, 'existing_rpc_connections', local_table)
    factory = AsyncMock(return_value=SurveyClient())
    result = await entry.execute(plan, **kwargs, client_factory=factory)
    factory.assert_not_called()
    assert not result['complete']


async def test_sdk_init_expiry_prevents_native_connect(context, monkeypatch):
    from types import SimpleNamespace
    from app.commissioning import readonly_kinematics as survey
    from app.robots import lebai_sdk_bridge
    entry, plan, kwargs = context
    clock = [1_000_000_000]
    monkeypatch.setattr(survey, 'monotonic_ns', lambda: clock[0])
    monkeypatch.setattr(entry, 'monotonic_ns', lambda: clock[0], raising=False)
    def init():
        clock[0] += 5_000_000_000
    module = SimpleNamespace(init=init, connect=AsyncMock(return_value=SurveyClient()))
    monkeypatch.setattr(entry, 'verify_sdk_source', lambda: None)
    # Stub the bridge and proposed isolated loader; neither touches native SDK.
    monkeypatch.setattr(lebai_sdk_bridge.importlib, 'import_module', lambda name: module)
    result = await entry.execute(plan, **kwargs)
    module.connect.assert_not_called()
    assert not result['complete']
    assert result['native_sdk_connect_entered'] is False
    assert result['connection_factory_entered'] is True


async def test_utc_expiry_during_runtime_hashing_refuses_before_reservation(context, monkeypatch):
    from types import SimpleNamespace
    entry, plan, kwargs = context
    current = [datetime.now(UTC)]
    monkeypatch.setattr(entry, 'datetime', SimpleNamespace(now=lambda zone: current[0], fromisoformat=datetime.fromisoformat))
    def hashes():
        current[0] = datetime.fromisoformat(kwargs['expires'])
        return {'synthetic': 'a' * 64}
    monkeypatch.setattr(entry, 'runtime_hashes', hashes)
    factory = AsyncMock()
    with pytest.raises(ValueError, match='expired'):
        await entry.execute(plan, **kwargs, client_factory=factory)
    factory.assert_not_called()
    assert not plan['output'].exists()
