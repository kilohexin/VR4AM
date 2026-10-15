from datetime import UTC, datetime, timedelta
import importlib.util
import json
from pathlib import Path
from unittest.mock import AsyncMock

import pytest
import yaml

from tests.config.test_real_robot_config import REAL_CONFIG_TEMPLATE
from tests.commissioning.test_finite_window_run import ZClient

SOURCE=Path(__file__).resolve().parents[3]/'scripts/real_robot_finite_window.py'


@pytest.fixture
def context(tmp_path,monkeypatch):
    def build():
        assert SOURCE.exists(),'finite window entry not implemented'
        spec=importlib.util.spec_from_file_location('finite_entry',SOURCE);entry=importlib.util.module_from_spec(spec);spec.loader.exec_module(entry)
        monkeypatch.setattr(entry,'ROOT',tmp_path)
        monkeypatch.setattr(entry.helper.idle,'ROOT',tmp_path)
        monkeypatch.setattr(entry.helper.idle,'runtime_hashes',lambda:{})
        monkeypatch.setattr(entry.helper,'runtime_hashes',lambda:{})
        code=tmp_path/'code.py';code.write_text('trusted fixture\n')
        monkeypatch.setattr(entry,'runtime_hashes',lambda:{'code.py':entry.sha(code)})
        monkeypatch.setattr(entry,'existing_rpc_connections',lambda:0)
        data=yaml.safe_load(REAL_CONFIG_TEMPLATE.format(mode='control'))
        data['real_robot']['control']['max_joint_speed_radps']=.05
        data['real_robot']['control']['max_joint_acceleration_radps2']=.2
        config=tmp_path/'config.yaml';config.write_text(yaml.safe_dump(data))
        expires=(datetime.now(UTC)+timedelta(minutes=5)).isoformat()
        plan=entry.prepare(config,tmp_path/'.tmp/once',expires=expires)
        approved=tmp_path/'preview.json';approved.write_text(json.dumps(entry.preview(plan)))
        return entry,plan,dict(approved=approved,expected_hash=entry.sha(approved),expires=expires,confirm=entry.CONFIRM),code
    return build


def test_default_preview_reserves_nothing_and_has_one_window_cleanup_contract(context,monkeypatch,capsys):
    entry,plan,_,_=context()
    monkeypatch.setattr(entry,'execute',AsyncMock(side_effect=AssertionError('no execution')))
    assert entry.main(['--config',str(plan['config']),'--output',str(plan['output'])])==0
    value=json.loads(capsys.readouterr().out)
    assert value['max_pvat_entries']==value['max_stop_entries']==value['max_connections']==1
    assert value['allowed_writes']==['move_pvat','stop_move']
    assert value['cleanup_stop_on_motion_revocation'] and value['cleanup_stop_reservation_s']==5
    assert value['runtime_checks']=='full_sha_before_lease_metadata_at_pvat_for_already_loaded_code'
    assert value['motion_ttl_ms']==100 and not plan['output'].exists()


def test_smaller_step_is_bound_in_the_motion_scope(context):
    entry,plan,_,_=context()
    data=yaml.safe_load(plan['config'].read_text())
    data['real_robot']['control']['max_tcp_step_m']=.00005
    plan['config'].write_text(yaml.safe_dump(data))
    new=entry.prepare(plan['config'],plan['output'],expires=plan['authorization_expires_utc'])
    preview=entry.preview(new)
    assert preview['target']['distance_m']==.00005
    assert preview['requires_full_endpoint'] is True


def test_parallel_fk_opt_in_is_bound_to_preview_and_cli(context,monkeypatch,capsys):
    entry,plan,_,_=context()
    candidate=entry.prepare(plan['config'],plan['output'],expires=plan['authorization_expires_utc'],
        parallel_fk_audit=True)
    assert entry.preview(plan)['parallel_fk_audit'] is False
    assert entry.preview(candidate)['parallel_fk_audit'] is True
    monkeypatch.setattr(entry,'execute',AsyncMock(side_effect=AssertionError('no execution')))
    assert entry.main(['--config',str(plan['config']),'--output',str(plan['output']),
        '--parallel-fk-audit'])==0
    value=json.loads(capsys.readouterr().out)
    assert value['parallel_fk_audit'] and value['fk_order']=='feedback_then_parallel_goal_midpoint_endpoint'
    assert value['max_fk_entries']==4 and value['motion_ttl_ms']==100


async def test_parallel_plan_cannot_reuse_serial_approval(context):
    entry,plan,kw,_=context();factory=AsyncMock()
    candidate=entry.prepare(plan['config'],plan['output'],expires=plan['authorization_expires_utc'],
        parallel_fk_audit=True)
    with pytest.raises(ValueError):await entry.execute(candidate,**kw,client_factory=factory)
    factory.assert_not_called()


async def test_parallel_approved_fake_entry_executes_once(context):
    entry,plan,kw,_=context()
    candidate=entry.prepare(plan['config'],plan['output'],expires=plan['authorization_expires_utc'],
        parallel_fk_audit=True)
    kw['approved'].write_text(json.dumps(entry.preview(candidate)))
    kw['expected_hash']=entry.sha(kw['approved'])
    result=await entry.execute(candidate,**kw,client_factory=AsyncMock(return_value=ZClient()))
    assert result['complete'] and result['parallel_fk_audit']
    assert result['math_entries']['kinematics_forward']==4


def test_ready_tolerance_explicitly_bound_to_preview_and_cli(context,monkeypatch,capsys):
    entry,plan,_,_=context()
    candidate=entry.prepare(plan['config'],plan['output'],expires=plan['authorization_expires_utc'],
        ready_joint_tolerance_rad=.001)
    assert entry.preview(plan)['ready_joint_tolerance_rad']==.0001
    assert entry.preview(candidate)['ready_joint_tolerance_rad']==.001
    monkeypatch.setattr(entry,'execute',AsyncMock(side_effect=AssertionError('no execution')))
    assert entry.main(['--config',str(plan['config']),'--output',str(plan['output']),
        '--ready-joint-tolerance-rad','0.001'])==0
    assert json.loads(capsys.readouterr().out)['ready_joint_tolerance_rad']==.001


async def test_larger_ready_tolerance_cannot_reuse_strict_approval(context):
    entry,plan,kw,_=context();factory=AsyncMock()
    candidate=entry.prepare(plan['config'],plan['output'],expires=plan['authorization_expires_utc'],
        ready_joint_tolerance_rad=.001)
    with pytest.raises(ValueError):await entry.execute(candidate,**kw,client_factory=factory)
    factory.assert_not_called()


def test_parallel_motion_reads_are_explicit_and_bound_to_preview(context,monkeypatch,capsys):
    entry,plan,_,_=context()
    new=entry.prepare(plan['config'],plan['output'],expires=plan['authorization_expires_utc'],
        parallel_motion_reads=True)
    assert entry.preview(plan)['parallel_motion_reads'] is False
    assert entry.preview(new)['parallel_motion_reads'] is True
    monkeypatch.setattr(entry,'execute',AsyncMock(side_effect=AssertionError('no execution')))
    assert entry.main(['--config',str(plan['config']),'--output',str(plan['output']),
        '--parallel-motion-reads'])==0
    value=json.loads(capsys.readouterr().out)
    assert value['snapshot_order']=='connected_then_parallel_six_then_dependent_motion_state'
    assert value['motion_ttl_ms']==100 and value['parallel_motion_reads']


async def test_parallel_motion_reads_cannot_use_old_approval(context):
    entry,plan,kw,_=context();factory=AsyncMock()
    new=entry.prepare(plan['config'],plan['output'],expires=plan['authorization_expires_utc'],
        parallel_motion_reads=True)
    with pytest.raises(ValueError):await entry.execute(new,**kw,client_factory=factory)
    factory.assert_not_called()


async def test_parallel_snapshot_and_fk_fake_entry_completes_once(context):
    entry,plan,kw,_=context()
    new=entry.prepare(plan['config'],plan['output'],expires=plan['authorization_expires_utc'],
        parallel_motion_reads=True,parallel_fk_audit=True,ready_joint_tolerance_rad=.001)
    kw['approved'].write_text(json.dumps(entry.preview(new)))
    kw['expected_hash']=entry.sha(kw['approved'])
    r=await entry.execute(new,**kw,client_factory=AsyncMock(return_value=ZClient()))
    assert r['complete'] and r['parallel_motion_reads'] and r['parallel_fk_audit']


@pytest.mark.parametrize('bad',[dict(confirm='yes'),dict(approved=None),dict(expected_hash='b'*64),dict(expires=None),dict(expires='2000-01-01T00:00:00Z')])
async def test_bad_scope_cannot_connect_or_create_output(context,bad):
    entry,plan,kw,_=context();factory=AsyncMock()
    with pytest.raises(ValueError):await entry.execute(plan,**{**kw,**bad},client_factory=factory)
    factory.assert_not_called();assert not plan['output'].exists()


@pytest.mark.parametrize('changed',['config','runtime','rpc'])
async def test_initial_input_change_is_rejected_before_connection(context,monkeypatch,changed):
    entry,plan,kw,code=context();factory=AsyncMock()
    if changed=='config':plan['config'].write_text(plan['config'].read_text()+'\n# changed')
    elif changed=='runtime':code.write_text('changed\n')
    else:monkeypatch.setattr(entry,'existing_rpc_connections',lambda:1)
    with pytest.raises(ValueError):await entry.execute(plan,**kw,client_factory=factory)
    factory.assert_not_called()


async def test_fake_execution_completes_once_and_persists_report(context):
    entry,plan,kw,_=context();client=ZClient();factory=AsyncMock(return_value=client)
    result=await entry.execute(plan,**kw,client_factory=factory)
    assert result['complete'] and result['runtime']=='SIMULATION' and not result['hardware_verified']
    assert [c[0] for c in client.write_calls]==['move_pvat','stop_move']
    assert json.loads((plan['output']/'result.json').read_text())==result
    with pytest.raises(FileExistsError):await entry.execute(plan,**kw,client_factory=factory)
    factory.assert_awaited_once()


async def test_runtime_metadata_change_during_math_blocks_native_pvat(context):
    entry,plan,kw,code=context();client=ZClient();original=client.kinematics_forward
    async def mutate(joints):
        value=await original(joints)
        if len(client.fk_calls)==4:code.write_text('changed after full hashes\n')
        return value
    client.kinematics_forward=mutate
    result=await entry.execute(plan,**kw,client_factory=AsyncMock(return_value=client))
    assert not result['complete'] and result['pvat_entries']==0 and result['stop_entries']==1
    assert client.write_calls==[('stop_move',)]


async def test_source_change_between_queue_admission_and_native_child_is_rejected(context,monkeypatch):
    import asyncio
    entry,plan,kw,code=context();create=asyncio.create_task
    def schedule(coro,**kwargs):
        if kwargs.get('name')=='scoped-native-pvat':code.write_text('changed at native scheduling\n')
        return create(coro,**kwargs)
    monkeypatch.setattr(asyncio,'create_task',schedule)
    client=ZClient();r=await entry.execute(plan,**kw,client_factory=AsyncMock(return_value=client))
    assert r['pvat_entries']==0 and r['stop_entries']==1
    assert r['local_motion_rejection']=='finite_scope_loaded_runtime_metadata_changed'
    assert client.write_calls==[('stop_move',)]


async def test_full_runtime_hashes_are_not_repeated_inside_math_to_pvat_lease(context,monkeypatch):
    entry,plan,kw,_=context();client=ZClient();original_hashes=entry.runtime_hashes
    def guarded():
        assert not client.ik_calls,'full hashes inside motion lease'
        return original_hashes()
    monkeypatch.setattr(entry,'runtime_hashes',guarded)
    result=await entry.execute(plan,**kw,client_factory=AsyncMock(return_value=client))
    assert result['complete'] and result['pvat_entries']==1


async def test_sdk_connection_failure_consumes_output_with_no_robot_cleanup(context):
    entry,plan,kw,_=context();factory=AsyncMock(side_effect=RuntimeError('secret injected'))
    result=await entry.execute(plan,**kw,client_factory=factory)
    assert not result['complete'] and result['pvat_entries']==result['stop_entries']==0
    assert 'secret injected' not in json.dumps(result)
    with pytest.raises(FileExistsError):await entry.execute(plan,**kw,client_factory=factory)


async def test_cancelled_connection_records_unknown_and_no_cleanup_write(context):
    import asyncio
    entry,plan,kw,_=context();started=asyncio.Event()
    async def pending(ip):started.set();await asyncio.Event().wait()
    task=asyncio.create_task(entry.execute(plan,**kw,client_factory=pending));await started.wait();task.cancel()
    with pytest.raises(asyncio.CancelledError):await task
    result=json.loads((plan['output']/'result.json').read_text())
    assert result['interrupted'] and not result['complete'] and result['pvat_entries']==result['stop_entries']==0
    assert result['connection_diagnostics']['pending_remote_result_unknown']


async def test_sdk_init_cannot_renew_original_connection_deadline(context,monkeypatch):
    from types import SimpleNamespace
    from app.commissioning import readonly_kinematics
    entry,plan,kw,_=context();clock=[1_000_000_000]
    monkeypatch.setattr(entry,'monotonic_ns',lambda:clock[0]);monkeypatch.setattr(readonly_kinematics,'monotonic_ns',lambda:clock[0])
    def init():clock[0]+=3_000_000_000
    sdk=SimpleNamespace(init=init,connect=AsyncMock())
    monkeypatch.setattr(entry.helper.readonly,'verify_sdk_source',lambda:None)
    original=entry.importlib.import_module
    monkeypatch.setattr(entry.importlib,'import_module',lambda name:sdk if name=='lebai_sdk' else original(name))
    result=await entry.execute(plan,**kw)
    sdk.connect.assert_not_called()
    assert not result['complete'] and not result['connection_diagnostics']['native_sdk_connect_entered']


@pytest.mark.parametrize('changed',['copied_config','expires_during_metadata'])
async def test_last_write_input_checks_block_pvat(context,monkeypatch,changed):
    from types import SimpleNamespace
    entry,plan,kw,_=context();client=ZClient()
    if changed=='copied_config':
        original=client.kinematics_forward
        async def mutate(joints):
            result=await original(joints)
            if len(client.fk_calls)==4:
                path=plan['output']/'config-control.yaml';path.write_text(path.read_text()+'\n# changed')
            return result
        client.kinematics_forward=mutate
    else:
        now=[datetime.now(UTC)]
        monkeypatch.setattr(entry,'datetime',SimpleNamespace(now=lambda zone:now[0],fromisoformat=datetime.fromisoformat))
        original=entry.Inputs.metadata
        def mutate(paths):
            value=original(paths)
            if len(client.fk_calls)==4:now[0]=datetime.fromisoformat(kw['expires'])
            return value
        monkeypatch.setattr(entry.Inputs,'metadata',staticmethod(mutate))
    result=await entry.execute(plan,**kw,client_factory=AsyncMock(return_value=client))
    assert not result['complete'] and result['pvat_entries']==0 and result['stop_entries']==1
