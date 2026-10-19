"""New scope/token/protocol binding; all execution factories are Fake."""
import importlib.util
import json
from pathlib import Path
import sys
from unittest.mock import AsyncMock

import pytest

from tests.scripts import test_finite_pause_entry as legacy
from tests.commissioning.test_finite_pause_window import PauseZClient
from tests.robots.test_lebai_adapter_control import FakeClock

SOURCE=Path(__file__).resolve().parents[3]/'scripts/real_robot_finite_pause_parallel.py'


@pytest.fixture
def context(tmp_path,monkeypatch):
    monkeypatch.setattr(legacy,'SOURCE',SOURCE)
    return legacy.context.__wrapped__(tmp_path,monkeypatch)


def test_default_preview_binds_parallel_order_and_original_motion_limits(context,monkeypatch,capsys):
    entry,plan,_,_=context()
    monkeypatch.setattr(entry,'execute',AsyncMock(side_effect=AssertionError('preview executed')))
    assert entry.main(['--config',str(plan['config']),'--output',str(plan['output'])])==0
    p=json.loads(capsys.readouterr().out)
    assert p['scope']=='one_finite_z50um_window_and_reserved_parallel_sampling_pause'
    assert p['required_confirmation']=='I_APPROVE_ONE_FINITE_Z50UM_WINDOW_AND_ONE_PARALLEL_SAMPLING_PAUSE_NO_RESUME'
    assert p['stop_snapshot_read_policy']=='six_fields_parallel'
    assert p['stop_snapshot_order']=='connected_then_parallel_six_then_dependent_motion_state'
    assert p['stop_sampling_policy']=='start_period_20ms'
    assert p['target']==dict(axis='z',distance_m=.00005,rotation=False)
    assert p['max_connections']==p['max_pvat_entries']==p['max_pause_entries']==1
    assert p['allowed_writes']==['move_pvat','pause_move'] and p['max_stop_entries']==0
    assert (p['motion_ttl_ms'],p['native_pvat_await_timeout_ms'])==(100,60)
    assert (p['stop_rpc_timeout_ms'],p['stop_stable_ms'],p['stop_verification_ms'])==(200,300,500)
    assert not plan['output'].exists() and 'lebai_sdk' not in sys.modules


@pytest.mark.parametrize('case',['token','prior_preview','expired','hash','config','runtime','rpc','policy'])
def test_wrong_material_never_creates_output_or_calls_factory(context,monkeypatch,case):
    entry,plan,kwargs,code=context()
    if case=='token':kwargs['confirm']=entry.prior.CONFIRM
    elif case=='prior_preview':
        kwargs['approved'].write_text(json.dumps(entry.prior.preview(plan)))
        kwargs['expected_hash']=entry.sha(kwargs['approved'])
    elif case=='expired':kwargs['expires']='2000-01-01T00:00:00Z'
    elif case=='hash':kwargs['expected_hash']='a'*64
    elif case=='config':plan['config'].write_text(plan['config'].read_text()+'\n# changed')
    elif case=='runtime':code.write_text('changed\n')
    elif case=='rpc':monkeypatch.setattr(entry,'existing_rpc_connections',lambda:1)
    else:plan['stop_snapshot_read_policy']='serial'
    factory=AsyncMock(side_effect=AssertionError('invalid scope connected'))
    with pytest.raises(ValueError):legacy.execute(entry,plan,kwargs,factory)
    factory.assert_not_called()
    assert not plan['output'].exists()


def test_new_preview_cannot_authorize_prior_sampling_entry(context):
    entry,plan,kwargs,_=context()
    factory=AsyncMock(side_effect=AssertionError('new material used for old scope'))
    with pytest.raises(ValueError):
        legacy.execute(entry.prior,plan,{**kwargs,'confirm':entry.prior.CONFIRM},factory)
    factory.assert_not_called()


def test_fake_execute_persists_actual_policy_and_consumes_one_attempt(context):
    entry,plan,kwargs,_=context()
    client=PauseZClient(FakeClock())
    factory=AsyncMock(return_value=client)
    result=legacy.execute(entry,plan,kwargs,factory)
    assert result['complete'] and result['stop_confirmed']
    assert result['site_parallel_stop_reads'] and not result['offline_parallel_stop_reads']
    assert result['stop_snapshot_read_policy']=='six_fields_parallel'
    assert [item[0] for item in client.write_calls]==['move_pvat','pause_move']
    assert json.loads((plan['output']/'result.json').read_text())==result
    with pytest.raises(FileExistsError):legacy.execute(entry,plan,kwargs,factory)
    factory.assert_awaited_once()


async def test_non_qpc_loop_rejected_before_factory(context):
    entry,plan,kwargs,_=context()
    factory=AsyncMock(side_effect=AssertionError('non QPC loop connected'))
    with pytest.raises(ValueError,match='qpc_event_loop_required'):
        await entry.execute(plan,**kwargs,client_factory=factory)
    factory.assert_not_called()
    assert not plan['output'].exists()


test_owner_cancellation_records_unknown_without_repeating_writes=legacy.test_owner_cancellation_records_unknown_without_repeating_writes
test_copied_config_change_before_pvat_only_uses_reserved_pause=legacy.test_copied_config_change_before_pvat_only_uses_reserved_pause
test_new_rpc_before_connection_blocks_factory_and_consumes_attempt=legacy.test_new_rpc_before_connection_blocks_factory_and_consumes_attempt
test_pause_entry_does_not_repeat_full_hashes_inside_motion_lease=legacy.test_pause_entry_does_not_repeat_full_hashes_inside_motion_lease
test_missing_pause_capability_has_zero_motion_and_no_stop_fallback=legacy.test_missing_pause_capability_has_zero_motion_and_no_stop_fallback
test_scope_rejects_other_window_sizes_without_editing_config=legacy.test_scope_rejects_other_window_sizes_without_editing_config
