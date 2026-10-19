"""No real transport: the separately scoped pause entry with injected Fake."""
import asyncio
from datetime import UTC, datetime, timedelta
import importlib.util
import json
from pathlib import Path
import sys
from unittest.mock import AsyncMock

import pytest
import yaml

from app.commissioning.qpc_event_loop import new_qpc_loop
from tests.commissioning.test_finite_pause_window import PauseZClient
from tests.config.test_real_robot_config import REAL_CONFIG_TEMPLATE
from tests.robots.test_lebai_adapter_control import FakeClock

SOURCE = Path(__file__).resolve().parents[3] / 'scripts/real_robot_finite_pause.py'


@pytest.fixture
def context(tmp_path, monkeypatch):
    def build():
        assert SOURCE.exists(), 'independent finite pause entry not implemented'
        spec = importlib.util.spec_from_file_location('finite_pause_entry', SOURCE)
        entry = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(entry)
        monkeypatch.setattr(entry, 'ROOT', tmp_path)
        monkeypatch.setattr(entry.base, 'ROOT', tmp_path)
        monkeypatch.setattr(entry.base.helper.idle, 'ROOT', tmp_path)
        monkeypatch.setattr(entry.base.helper.idle, 'runtime_hashes', lambda: {})
        monkeypatch.setattr(entry.base.helper, 'runtime_hashes', lambda: {})
        code = tmp_path / 'code.py'
        code.write_text('trusted fixture\n')
        hashes = lambda: {'code.py': entry.sha(code)}
        monkeypatch.setattr(entry.base, 'runtime_hashes', hashes)
        monkeypatch.setattr(entry, 'runtime_hashes', hashes)
        monkeypatch.setattr(entry, 'existing_rpc_connections', lambda: 0)
        data = yaml.safe_load(REAL_CONFIG_TEMPLATE.format(mode='control'))
        data['real_robot']['control'].update(max_joint_speed_radps=.05,
            max_joint_acceleration_radps2=.2, max_tcp_step_m=.00005)
        data['real_robot']['teleop_ready_q'] = [0., -1., 1., 0., 1.57, 0.]
        config = tmp_path / 'config.yaml'
        config.write_text(yaml.safe_dump(data))
        expires = (datetime.now(UTC) + timedelta(minutes=5)).isoformat()
        plan = entry.prepare(config, tmp_path / '.tmp/once', expires=expires)
        approved = tmp_path / 'preview.json'
        approved.write_text(json.dumps(entry.preview(plan)))
        kwargs = dict(approved=approved, expected_hash=entry.sha(approved),
                      expires=expires, confirm=entry.CONFIRM)
        return entry, plan, kwargs, code
    return build


def execute(entry, plan, kwargs, client_factory=None):
    loop = new_qpc_loop()
    try:
        return loop.run_until_complete(entry.execute(plan, **kwargs, client_factory=client_factory))
    finally:
        loop.close()


def test_default_cli_only_previews_independent_pause_scope(context, monkeypatch, capsys):
    entry, plan, _, _ = context()
    monkeypatch.setattr(entry, 'execute', AsyncMock(side_effect=AssertionError('preview connected')))
    assert entry.main(['--config', str(plan['config']), '--output', str(plan['output'])]) == 0
    value = json.loads(capsys.readouterr().out)
    assert value['scope'] == 'one_finite_z50um_window_and_reserved_cleanup_pause'
    assert value['allowed_writes'] == ['move_pvat', 'pause_move']
    assert value['max_pvat_entries'] == value['max_pause_entries'] == value['max_connections'] == 1
    assert value['max_stop_entries'] == 0 and value['max_ik_entries'] == 1 and value['max_fk_entries'] == 4
    assert value['target'] == dict(axis='z', distance_m=.00005, rotation=False)
    assert value['max_reference_s'] == .08
    assert value['motion_ttl_ms'] == 100 and value['native_pvat_await_timeout_ms'] == 60
    assert value['stop_rpc_timeout_ms'] == 200 and value['stop_stable_ms'] == 300
    assert value['stop_verification_ms'] == 500 and value['readonly_tail_s'] == 2.
    assert value['paused_motion_may_be_retained'] and value['requires_finished_task_when_paused']
    assert value['cleanup_pause_on_motion_revocation'] and not value['cleanup_stop_on_motion_revocation']
    assert not any(value[k] for k in ('automatic_resume', 'automatic_retry', 'stop_move_fallback', 'motion_authorized'))
    assert value['authorization_expires_utc'] is None
    assert not plan['output'].exists() and 'lebai_sdk' not in sys.modules


@pytest.mark.parametrize('step', [.0002, .0001, .00004])
def test_scope_rejects_other_window_sizes_without_editing_config(context, step):
    entry, plan, _, _ = context()
    data = yaml.safe_load(plan['config'].read_text())
    data['real_robot']['control']['max_tcp_step_m'] = step
    plan['config'].write_text(yaml.safe_dump(data))
    saved = plan['config'].read_bytes()
    with pytest.raises(ValueError, match='pause.*50um'):
        entry.prepare(plan['config'], plan['output'])
    assert plan['config'].read_bytes() == saved and not plan['output'].exists()


@pytest.mark.parametrize('kind', ['confirm', 'stop_preview', 'expired', 'hash', 'config', 'runtime', 'rpc'])
def test_changed_or_wrong_scope_cannot_create_output_or_call_factory(context, monkeypatch, kind):
    entry, plan, kwargs, code = context()
    if kind == 'confirm':
        kwargs['confirm'] = entry.base.CONFIRM
    elif kind == 'stop_preview':
        kwargs['approved'].write_text(json.dumps(entry.base.preview(plan)))
        kwargs['expected_hash'] = entry.sha(kwargs['approved'])
    elif kind == 'expired':
        kwargs['expires'] = '2000-01-01T00:00:00Z'
    elif kind == 'hash':
        kwargs['expected_hash'] = 'a' * 64
    elif kind == 'config':
        plan['config'].write_text(plan['config'].read_text() + '\n# changed')
    elif kind == 'runtime':
        code.write_text('changed fixture\n')
    else:
        monkeypatch.setattr(entry, 'existing_rpc_connections', lambda: 1)
    factory = AsyncMock(side_effect=AssertionError('invalid authorization connected'))
    with pytest.raises(ValueError):
        execute(entry, plan, kwargs, factory)
    factory.assert_not_called()
    assert not plan['output'].exists()


def test_pause_preview_cannot_authorize_original_stop_entry(context):
    entry, plan, kwargs, _ = context()
    factory = AsyncMock(side_effect=AssertionError('pause approval used for stop'))
    with pytest.raises(ValueError):
        execute(entry.base, plan, {**kwargs, 'confirm': entry.base.CONFIRM}, factory)
    factory.assert_not_called()
    assert not plan['output'].exists()


def test_one_fake_execution_persists_actual_pause_and_consumes_attempt(context):
    entry, plan, kwargs, _ = context()
    client = PauseZClient(FakeClock())
    factory = AsyncMock(return_value=client)
    result = execute(entry, plan, kwargs, factory)
    assert result['complete'] and result['stop_confirmed']
    assert result['runtime'] == 'SIMULATION' and result['site_pause_comparison']
    assert not result['offline_pause_comparison']
    assert result['pvat_entries'] == result['pause_entries'] == 1 and result['stop_entries'] == 0
    assert result['math_entries'] == dict(kinematics_inverse=1, kinematics_forward=4)
    assert [c[0] for c in client.write_calls] == ['move_pvat', 'pause_move']
    assert client.write_calls[0][4] <= .08 and client.write_calls[0][2] == client.write_calls[0][3] == [0.] * 6
    assert not result['hardware_verified'] and not result['motion_authorized']
    assert json.loads((plan['output'] / 'result.json').read_text()) == result
    admission = json.loads((plan['output'] / 'admission.json').read_text())
    assert admission['scope']['allowed_writes'] == ['move_pvat', 'pause_move']
    with pytest.raises(FileExistsError):
        execute(entry, plan, kwargs, factory)
    factory.assert_awaited_once()
    assert client.network_calls == client.discovery_calls == 0


@pytest.mark.parametrize('kind', ['running', 'error', 'drift', 'late_rpc'])
def test_pause_failures_never_send_fallback_or_clear_fault(context, kind):
    entry, plan, kwargs, _ = context()
    client = PauseZClient(FakeClock(), mode=kind if kind != 'late_rpc' else 'finished')
    if kind == 'late_rpc':
        original = client.pause_move
        async def delayed():
            await original()
            await asyncio.sleep(.25)
        client.pause_move = delayed
    if kind == 'drift':
        original_read = client.get_kin_data
        async def read():
            if client.pause_ns is not None:
                client.kin_data['actual_joint_pose'][0] += .002
            return await original_read()
        client.get_kin_data = read
    result = execute(entry, plan, kwargs, AsyncMock(return_value=client))
    assert not result['complete'] and not result['stop_confirmed']
    assert result['latched_fault'] in ('stop_unverified', 'stop_incomplete')
    assert result['pause_entries'] == 1 and result['stop_entries'] == 0
    assert [c[0] for c in client.write_calls] == ['move_pvat', 'pause_move']


def test_source_change_before_native_motion_allows_only_reserved_pause(context):
    entry, plan, kwargs, code = context()
    client = PauseZClient(FakeClock())
    original = client.kinematics_forward
    async def mutate(joints):
        value = await original(joints)
        if len(client.fk_calls) == 4:
            code.write_text('changed after cold hashes\n')
        return value
    client.kinematics_forward = mutate
    result = execute(entry, plan, kwargs, AsyncMock(return_value=client))
    assert not result['complete'] and result['pvat_entries'] == 0
    assert result['pause_entries'] == 1 and result['stop_entries'] == 0
    assert client.write_calls == [('pause_move',)]


def test_connection_failure_is_single_consumed_attempt_without_pause(context):
    entry, plan, kwargs, _ = context()
    factory = AsyncMock(side_effect=RuntimeError('private injected details'))
    result = execute(entry, plan, kwargs, factory)
    assert not result['complete']
    assert result['pvat_entries'] == result['pause_entries'] == result['stop_entries'] == 0
    assert 'private injected details' not in json.dumps(result)
    with pytest.raises(FileExistsError):
        execute(entry, plan, kwargs, factory)
    factory.assert_awaited_once()


@pytest.mark.parametrize('field', ['qpc_timing', 'parallel_fk_audit', 'parallel_motion_reads', 'prelease_feedback_fk'])
def test_fixed_profile_cannot_be_changed_after_approval(context, field):
    entry, plan, kwargs, _ = context()
    plan[field] = False
    factory = AsyncMock(side_effect=AssertionError('changed profile connected'))
    with pytest.raises(ValueError, match='finite_pause_profile_changed'):
        execute(entry, plan, kwargs, factory)
    factory.assert_not_called()
    assert not plan['output'].exists()


def test_missing_pause_capability_has_zero_motion_and_no_stop_fallback(context):
    from tests.commissioning.test_finite_window_run import ZClient
    entry, plan, kwargs, _ = context()
    client = ZClient()
    result = execute(entry, plan, kwargs, AsyncMock(return_value=client))
    assert not result['complete']
    assert result['pvat_entries'] == result['pause_entries'] == result['stop_entries'] == 0
    assert client.write_calls == []


def test_unknown_pvat_result_keeps_fault_and_one_reserved_pause(context):
    entry, plan, kwargs, _ = context()
    client = PauseZClient(FakeClock())
    original = client.move_pvat
    async def unknown(*args):
        await original(*args)
        raise RuntimeError('injected unknown result')
    client.move_pvat = unknown
    result = execute(entry, plan, kwargs, AsyncMock(return_value=client))
    assert result['pvat_remote_result_unknown']
    assert not result['complete'] and not result['stop_confirmed']
    assert result['latched_fault'] == 'stop_unverified'
    assert [c[0] for c in client.write_calls] == ['move_pvat', 'pause_move']


def test_copied_config_change_before_pvat_only_uses_reserved_pause(context):
    entry, plan, kwargs, _ = context()
    client = PauseZClient(FakeClock())
    original = client.kinematics_forward
    async def mutate(joints):
        value = await original(joints)
        if len(client.fk_calls) == 4:
            copied = plan['output'] / 'config-control.yaml'
            copied.write_text(copied.read_text() + '\n# changed')
        return value
    client.kinematics_forward = mutate
    result = execute(entry, plan, kwargs, AsyncMock(return_value=client))
    assert not result['complete'] and result['pvat_entries'] == 0
    assert result['pause_entries'] == 1 and result['stop_entries'] == 0
    assert client.write_calls == [('pause_move',)]


def test_new_rpc_before_connection_blocks_factory_and_consumes_attempt(context, monkeypatch):
    entry, plan, kwargs, _ = context()
    checks = []
    def rpc_count():
        checks.append(True)
        return 0 if len(checks) == 1 else 1
    monkeypatch.setattr(entry, 'existing_rpc_connections', rpc_count)
    factory = AsyncMock(side_effect=AssertionError('conflicting RPC connected'))
    result = execute(entry, plan, kwargs, factory)
    factory.assert_not_called()
    assert not result['complete']
    assert result['pvat_entries'] == result['pause_entries'] == result['stop_entries'] == 0
    assert (plan['output'] / 'result.json').exists()


def test_pause_entry_does_not_repeat_full_hashes_inside_motion_lease(context, monkeypatch):
    entry, plan, kwargs, _ = context()
    client = PauseZClient(FakeClock())
    original = entry.runtime_hashes
    def guarded_hashes():
        assert not client.ik_calls, 'full hashes repeated inside the 100 ms motion lease'
        return original()
    monkeypatch.setattr(entry, 'runtime_hashes', guarded_hashes)
    result = execute(entry, plan, kwargs, AsyncMock(return_value=client))
    assert result['complete'] and result['pvat_entries'] == result['pause_entries'] == 1


@pytest.mark.parametrize('phase', ['connect', 'reference'])
def test_owner_cancellation_records_unknown_without_repeating_writes(context, phase):
    entry, plan, kwargs, _ = context()
    entered, release = asyncio.Event(), asyncio.Event()
    client = PauseZClient(FakeClock())
    async def factory(ip):
        if phase == 'connect':
            entered.set()
            await release.wait()
        return client
    original = client.move_pvat
    async def move(*args):
        result = await original(*args)
        entered.set()
        return result
    client.move_pvat = move
    async def scenario():
        task = asyncio.create_task(entry.execute(plan, **kwargs, client_factory=factory))
        try:
            await asyncio.wait_for(entered.wait(), 1.)
            task.cancel()
            with pytest.raises(asyncio.CancelledError):
                await asyncio.wait_for(task, 1.)
        finally:
            release.set()
            if not task.done():
                task.cancel()
                await asyncio.gather(task, return_exceptions=True)
    loop = new_qpc_loop()
    try:
        loop.run_until_complete(scenario())
    finally:
        loop.close()
    result = json.loads((plan['output'] / 'result.json').read_text())
    assert result['interrupted'] and not result['complete']
    assert result['stop_entries'] == 0
    if phase == 'connect':
        assert client.write_calls == []
        assert result['connection_diagnostics']['pending_remote_result_unknown']
    else:
        assert [c[0] for c in client.write_calls] == ['move_pvat', 'pause_move']
        assert result['pause_entries'] == 1


async def test_site_entry_requires_qpc_before_connect(context):
    entry, plan, kwargs, _ = context()
    factory = AsyncMock(side_effect=AssertionError('must reject before connection'))
    with pytest.raises(ValueError, match='qpc_event_loop_required'):
        await entry.execute(plan, **kwargs, client_factory=factory)
    factory.assert_not_called()
    assert not plan['output'].exists()
