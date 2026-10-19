"""Independent admission for start-period pause sampling; Fake transport only."""
import importlib.util
import json
from pathlib import Path
import sys
from unittest.mock import AsyncMock

import pytest

from tests.scripts import test_finite_pause_entry as legacy
from tests.commissioning.test_finite_pause_window import PauseZClient
from tests.robots.test_lebai_adapter_control import FakeClock

SOURCE = Path(__file__).resolve().parents[3] / 'scripts/real_robot_finite_pause_sampling.py'


@pytest.fixture
def context(tmp_path, monkeypatch):
    monkeypatch.setattr(legacy, 'SOURCE', SOURCE)
    return legacy.context.__wrapped__(tmp_path, monkeypatch)


def test_preview_has_independent_sampling_scope_and_no_execution(context, monkeypatch, capsys):
    entry, plan, _, _ = context()
    monkeypatch.setattr(entry, 'execute', AsyncMock(side_effect=AssertionError('preview connected')))
    assert entry.main(['--config', str(plan['config']), '--output', str(plan['output'])]) == 0
    value = json.loads(capsys.readouterr().out)
    assert value['scope'] == 'one_finite_z50um_window_and_reserved_start_period_pause'
    assert value['required_confirmation'] == 'I_APPROVE_ONE_FINITE_Z50UM_WINDOW_AND_ONE_START_PERIOD_PAUSE_NO_RESUME'
    assert value['stop_sampling_policy'] == 'start_period_20ms'
    assert value['minimum_stop_read_start_period_ms'] == 20
    assert value['allowed_writes'] == ['move_pvat', 'pause_move']
    assert value['max_connections'] == value['max_pvat_entries'] == value['max_pause_entries'] == 1
    assert value['max_stop_entries'] == 0 and value['max_ik_entries'] == 1 and value['max_fk_entries'] == 4
    assert value['stop_rpc_timeout_ms'] == 200 and value['stop_stable_ms'] == 300
    assert value['stop_verification_ms'] == 500 and value['readonly_tail_s'] == 2.
    assert not plan['output'].exists() and 'lebai_sdk' not in sys.modules


@pytest.mark.parametrize('case', ['token', 'legacy_preview', 'expired', 'hash', 'config', 'runtime', 'rpc', 'policy'])
def test_invalid_scope_never_calls_factory_or_reserves_output(context, monkeypatch, case):
    entry, plan, kwargs, code = context()
    if case == 'token':
        kwargs['confirm'] = 'I_APPROVE_ONE_FINITE_Z50UM_WINDOW_AND_ONE_RESERVED_PAUSE_NO_RESUME'
    elif case == 'legacy_preview':
        old_path = SOURCE.with_name('real_robot_finite_pause.py')
        spec = importlib.util.spec_from_file_location('original_pause', old_path)
        old = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(old)
        kwargs['approved'].write_text(json.dumps(old.preview(plan)))
        kwargs['expected_hash'] = entry.sha(kwargs['approved'])
    elif case == 'expired':
        kwargs['expires'] = '2000-01-01T00:00:00Z'
    elif case == 'hash':
        kwargs['expected_hash'] = 'a' * 64
    elif case == 'config':
        plan['config'].write_text(plan['config'].read_text() + '\n# changed')
    elif case == 'runtime':
        code.write_text('changed\n')
    elif case == 'rpc':
        monkeypatch.setattr(entry, 'existing_rpc_connections', lambda: 1)
    else:
        plan['stop_sampling_policy'] = 'fixed_delay_after_read_20ms'
    factory = AsyncMock(side_effect=AssertionError('invalid scope connected'))
    with pytest.raises(ValueError):
        legacy.execute(entry, plan, kwargs, factory)
    factory.assert_not_called()
    assert not plan['output'].exists()


def test_new_preview_cannot_authorize_original_pause(context):
    entry, plan, kwargs, _ = context()
    spec = importlib.util.spec_from_file_location('old_pause', SOURCE.with_name('real_robot_finite_pause.py'))
    old = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(old)
    factory = AsyncMock(side_effect=AssertionError('new approval used for original scope'))
    with pytest.raises(ValueError):
        legacy.execute(old, plan, {**kwargs, 'confirm': old.CONFIRM}, factory)
    factory.assert_not_called()
    assert not plan['output'].exists()


def test_sampling_entry_executes_once_and_persists_actual_policy(context):
    entry, plan, kwargs, _ = context()
    client = PauseZClient(FakeClock())
    factory = AsyncMock(return_value=client)
    result = legacy.execute(entry, plan, kwargs, factory)
    assert result['complete'] and result['stop_confirmed']
    assert result['site_pause_start_period'] and result['stop_sampling_policy'] == 'start_period_20ms'
    assert result['pvat_entries'] == result['pause_entries'] == 1 and result['stop_entries'] == 0
    assert [c[0] for c in client.write_calls] == ['move_pvat', 'pause_move']
    assert json.loads((plan['output'] / 'result.json').read_text()) == result
    assert not result['hardware_verified'] and not result['motion_authorized']
    with pytest.raises(FileExistsError):
        legacy.execute(entry, plan, kwargs, factory)
    factory.assert_awaited_once()
    assert client.network_calls == client.discovery_calls == 0


def test_runtime_identity_change_prevents_motion_and_uses_only_reserved_pause(context):
    entry, plan, kwargs, code = context()
    client = PauseZClient(FakeClock())
    original = client.kinematics_forward
    async def change(joints):
        value = await original(joints)
        if len(client.fk_calls) == 4:
            code.write_text('changed before native write\n')
        return value
    client.kinematics_forward = change
    result = legacy.execute(entry, plan, kwargs, AsyncMock(return_value=client))
    assert not result['complete'] and result['pvat_entries'] == 0
    assert result['pause_entries'] == 1 and result['stop_entries'] == 0
    assert client.write_calls == [('pause_move',)]


async def test_non_qpc_loop_is_rejected_before_factory(context):
    entry, plan, kwargs, _ = context()
    factory = AsyncMock(side_effect=AssertionError('non QPC loop connected'))
    with pytest.raises(ValueError, match='qpc_event_loop_required'):
        await entry.execute(plan, **kwargs, client_factory=factory)
    factory.assert_not_called()
    assert not plan['output'].exists()


# Run the shared lifecycle contracts through this new entry and its own Inputs,
# rather than substituting the runner or authorization logic.
test_owner_cancellation_records_unknown_without_repeating_writes = legacy.test_owner_cancellation_records_unknown_without_repeating_writes
test_copied_config_change_before_pvat_only_uses_reserved_pause = legacy.test_copied_config_change_before_pvat_only_uses_reserved_pause
test_new_rpc_before_connection_blocks_factory_and_consumes_attempt = legacy.test_new_rpc_before_connection_blocks_factory_and_consumes_attempt
test_pause_entry_does_not_repeat_full_hashes_inside_motion_lease = legacy.test_pause_entry_does_not_repeat_full_hashes_inside_motion_lease
test_missing_pause_capability_has_zero_motion_and_no_stop_fallback = legacy.test_missing_pause_capability_has_zero_motion_and_no_stop_fallback
test_scope_rejects_other_window_sizes_without_editing_config = legacy.test_scope_rejects_other_window_sizes_without_editing_config
