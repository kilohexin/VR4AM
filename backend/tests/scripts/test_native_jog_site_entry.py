"""All execution factories are injected; no SDK init or robot access."""
import importlib.util
import json
from pathlib import Path
from datetime import UTC,datetime,timedelta
from unittest.mock import AsyncMock
from dataclasses import replace,asdict
import pytest
import yaml
from tests.config.test_real_robot_config import REAL_CONFIG_TEMPLATE
from tests.commissioning.test_native_jog_site_run import setup
from app.commissioning.readonly_jog_jacobian import collect,new_report
from tests.commissioning.test_readonly_jog_jacobian import JacobianClient

SOURCE=Path(__file__).resolve().parents[3]/'scripts/real_robot_native_jog_window.py'

@pytest.fixture
async def context(tmp_path,monkeypatch):
    spec=importlib.util.spec_from_file_location('native_site_entry',SOURCE)
    entry=importlib.util.module_from_spec(spec);spec.loader.exec_module(entry)
    monkeypatch.setattr(entry,'ROOT',tmp_path)
    code=tmp_path/'code.py';code.write_text('synthetic')
    monkeypatch.setattr(entry,'runtime_hashes',lambda:{'code.py':entry.sha(code)})
    monkeypatch.setattr(entry,'existing_rpc_connections',lambda:0)
    scope,client=setup()
    client.running_motion=0
    data=yaml.safe_load(REAL_CONFIG_TEMPLATE.format(mode='control'))
    s=scope.settings;real=data['real_robot'];real['ip']='192.168.42.10'
    real['teleop_ready_q']=list(s.teleop_ready_q)
    real['startup_tcp_min_m']=list(s.startup_tcp_min_m);real['startup_tcp_max_m']=list(s.startup_tcp_max_m)
    real['control']['max_joint_speed_radps']=.05;real['control']['max_joint_acceleration_radps2']=.2
    config=tmp_path/'control.yaml';config.write_text(yaml.safe_dump(data))
    # Generate complete thirteen-point evidence using synthetic mathematical FK.
    from app.config import Settings,REAL_ROBOT_CONFIRMATION
    monkeypatch.setenv('VR4ARM_REAL_ROBOT_CONFIRM',REAL_ROBOT_CONFIRMATION)
    from app.commissioning.native_jog_mdh import mdh_tcp_jacobian,MdhTool
    from scipy.spatial.transform import Rotation
    from app.robots.lebai_codec import pose_to_lebai
    from app.schemas.messages import Pose
    async def fk(q):
        t,_=mdh_tcp_jacobian(q,scope.reference_tool)
        return pose_to_lebai(Pose(p=tuple(t[:3,3]),q=tuple(Rotation.from_matrix(t[:3,:3]).as_quat())))
    client.kinematics_forward=fk
    async def factory(ip):return client
    report=new_report(simulation=True)
    settings=replace(Settings.load(config).lebai,mode='readonly')
    await collect(settings,factory,lambda:None,report)
    assert report['complete'],report
    report['runtime']='LIVE_READONLY' # synthetic fixture; only injected execution below
    reference=tmp_path/'reference/result.json';reference.parent.mkdir()
    reference.write_text(json.dumps(report))
    admission=dict(scope=dict(scope='one_readonly_jog_jacobian_no_robot_writes',target_ip=settings.ip,
        allowed_writes=[],max_fk_calls=13,max_ik_calls=0),approved_preview_sha256='a'*64)
    reference.with_name('admission.json').write_text(json.dumps(admission))
    planning=tmp_path/'limits.json';planning.write_text(json.dumps(asdict(scope.limits)))
    expires=(datetime.now(UTC)+timedelta(minutes=5)).isoformat()
    plan=entry.prepare(config,reference,planning,tmp_path/'.tmp/once',expires=expires)
    approved=tmp_path/'preview.json';approved.write_text(json.dumps(entry.preview(plan)))
    kw=dict(approved=approved,expected_hash=entry.sha(approved),expires=expires,confirm=entry.CONFIRM)
    return entry,plan,kw,client


async def test_default_preview_does_not_reserve_or_initialize(context,monkeypatch,capsys):
    e,p,_,_=context
    monkeypatch.setattr(e,'execute',AsyncMock(side_effect=AssertionError('preview only')))
    assert e.main(['--config',str(p['config']),'--reference-survey',str(p['reference_survey']),
        '--planning-limits',str(p['planning_limits']),'--output',str(p['output'])])==0
    r=json.loads(capsys.readouterr().out)
    assert r['max_read_entries']==256 and r['max_total_calls']==259
    assert r['allowed_writes']==['speedl','stop_move'] and not p['output'].exists()


@pytest.mark.parametrize('kind',['confirm','expiry','hash','preview','config','reference','admission','planning','runtime','rpc'])
async def test_invalid_admission_never_calls_factory(context,monkeypatch,kind):
    e,p,kw,_=context;factory=AsyncMock()
    if kind=='confirm':kw['confirm']='I_APPROVE_ONE_FINITE_WINDOW_AND_ONE_RESERVED_STOP_ON_REVOCATION'
    elif kind=='expiry':kw['expires']='2000-01-01T00:00:00Z'
    elif kind=='hash':kw['expected_hash']='b'*64
    elif kind=='preview':kw['approved'].write_text('{}')
    elif kind=='runtime':(e.ROOT/'code.py').write_text('changed')
    elif kind=='rpc':monkeypatch.setattr(e,'existing_rpc_connections',lambda:1)
    else:
        key={'reference':'reference_survey','admission':'reference_admission','planning':'planning_limits','config':'config'}[kind]
        p[key].write_bytes(p[key].read_bytes()+b'\n ')
    with pytest.raises((ValueError,FileExistsError)):await e.execute(p,**kw,client_factory=factory)
    factory.assert_not_called();assert not p['output'].exists()


async def test_injected_full_run_is_saved_simulation_and_once(context):
    e,p,kw,c=context;factory=AsyncMock(return_value=c)
    r=await e.execute(p,**kw,client_factory=factory)
    assert r['complete'],r
    assert r['runtime']=='SIMULATION' and factory.await_count==1
    assert json.loads((p['output']/'result.json').read_text())==r
    with pytest.raises(FileExistsError):await e.execute(p,**kw,client_factory=factory)
    assert factory.await_count==1


@pytest.mark.parametrize('kind',['simulation','incomplete','foreign','missing','unknown'])
async def test_reference_evidence_is_not_arbitrary(context,kind):
    e,p,_,_=context;r=json.loads(p['reference_survey'].read_text())
    if kind=='simulation':r['runtime']='SIMULATION'
    elif kind=='incomplete':r['complete']=False
    elif kind=='missing':r['fk_samples'].pop()
    elif kind=='unknown':r['calls'][0]['remote_result_unknown']=True
    else:
        a=json.loads(p['reference_admission'].read_text());a['scope']['target_ip']='192.168.99.10'
        p['reference_admission'].write_text(json.dumps(a))
    p['reference_survey'].write_text(json.dumps(r))
    with pytest.raises(ValueError):e.prepare(p['config'],p['reference_survey'],p['planning_limits'],p['output'])


async def test_sdk_failure_consumes_output_and_is_sanitized(context,monkeypatch):
    e,p,kw,_=context
    monkeypatch.setattr(e,'load_sdk',lambda:(_ for _ in ()).throw(ValueError('vendor secret')))
    # Exercise native admission gates without importing the SDK.
    monkeypatch.setattr(e,'require_execution_runtime',lambda:None)
    r=await e.execute(p,**kw)
    assert not r['complete'] and not r['connection_diagnostics']['native_sdk_init_entered']
    assert 'vendor secret' not in json.dumps(r) and p['output'].exists()


@pytest.mark.parametrize('loaded_wrong',[False,True])
async def test_pinned_sdk_identity_rejects_before_init(context,monkeypatch,loaded_wrong):
    from types import SimpleNamespace
    e,p,_,_=context
    package=e.ROOT/'.venv/Lib/site-packages/lebai_sdk';package.mkdir(parents=True)
    binary=package/'lebai_sdk.pyd';binary.write_bytes(b'synthetic_wrong_sdk')
    initialized=[]
    sdk=SimpleNamespace(__file__=str(package/'__init__.py'),init=lambda:initialized.append(True))
    native=SimpleNamespace(__file__=str(package/'wrong.pyd'))
    monkeypatch.setattr(e.readonly,'verify_sdk_source',lambda:None)
    if loaded_wrong:
        original=e.sha
        monkeypatch.setattr(e,'sha',lambda path:e.SDK_SHA if Path(path)==binary else original(path))
        monkeypatch.setattr(e.importlib,'import_module',lambda name:sdk if name=='lebai_sdk' else native)
    else:
        monkeypatch.setattr(e.importlib,'import_module',lambda name:pytest.fail('import before pinned SHA'))
    with pytest.raises(ValueError):e.load_sdk()
    assert not initialized


async def test_source_change_after_speed_entry_does_not_block_reserved_stop(context):
    e,p,kw,c=context;old=c.speedl
    async def speed(*args):
        value=await old(*args);(e.ROOT/'code.py').write_text('changed after speed entry');return value
    c.speedl=speed
    r=await e.execute(p,**kw,client_factory=AsyncMock(return_value=c))
    assert r['speedl_entries']==r['stop_entries']==1
    assert [v[0] for v in c.write_calls]==['speedl','stop_move']
    assert not r['complete']


async def test_failed_factory_is_consumed_with_zero_motion_and_stop(context):
    e,p,kw,c=context;factory=AsyncMock(side_effect=RuntimeError('sensitive vendor text'))
    r=await e.execute(p,**kw,client_factory=factory)
    assert not r['complete'] and r['speedl_entries']==r['stop_entries']==0
    assert 'sensitive vendor text' not in json.dumps(r) and p['output'].exists()
    with pytest.raises(FileExistsError):await e.execute(p,**kw,client_factory=factory)
    assert factory.await_count==1


async def test_light_read_checks_do_not_scan_runtime_but_hot_still_does(context,monkeypatch):
    e,p,kw,_=context;x=e.Inputs(p,kw['approved'],kw['expected_hash'],kw['expires'],kw['confirm']);x.cold()
    def forbid():raise AssertionError('full runtime scan')
    monkeypatch.setattr(x,'fingerprints',forbid)
    x.read()
    with pytest.raises(AssertionError,match='full runtime scan'):x.hot()


@pytest.mark.parametrize('change',['config','preview','reference','expiry'])
async def test_light_read_path_keeps_input_and_absolute_expiry_checks(context,change):
    e,p,kw,_=context;x=e.Inputs(p,kw['approved'],kw['expected_hash'],kw['expires'],kw['confirm']);x.cold()
    if change=='config':p['config'].write_bytes(p['config'].read_bytes()+b'\n# changed')
    elif change=='preview':kw['approved'].write_text('{}')
    elif change=='reference':p['reference_survey'].write_bytes(p['reference_survey'].read_bytes()+b' ')
    else:x.expires='2000-01-01T00:00:00Z'
    with pytest.raises(ValueError):x.read()


async def test_old_snapshot_policy_preview_cannot_authorize_new_read_groups(context):
    e,p,kw,c=context;old=e.preview(p)
    old.pop('snapshot_order',None);old.pop('read_authorization_policy',None)
    kw['approved'].write_text(json.dumps(old));kw['expected_hash']=e.sha(kw['approved'])
    with pytest.raises(ValueError,match='preview_changed'):
        await e.execute(p,**kw,client_factory=AsyncMock(return_value=c))
    assert not p['output'].exists()


async def test_cold_hash_check_uses_two_complete_metadata_sweeps(context,monkeypatch):
    e,p,kw,_=context;x=e.Inputs(p,kw['approved'],kw['expected_hash'],kw['expires'],kw['confirm'])
    original=x.fingerprints;calls=[]
    def counted():calls.append(True);return original()
    monkeypatch.setattr(x,'fingerprints',counted)
    x.cold();assert len(calls)==2
    calls.clear();x.cold();assert len(calls)==2


async def test_source_change_during_preflight_is_caught_at_group_boundary(context):
    e,p,kw,c=context;old=c.get_kin_data;count=0
    async def read():
        nonlocal count
        count+=1
        if count==2:(e.ROOT/'code.py').write_text('changed during preflight')
        return await old()
    c.get_kin_data=read
    r=await e.execute(p,**kw,client_factory=AsyncMock(return_value=c))
    assert r['primary_rejection']=='native_jog_runtime_metadata_changed'
    assert r['rejection_phase']=='preflight'
    assert r['snapshot_groups'][0]['outcome']=='failed'
    assert r['fk_entries']==r['speedl_entries']==r['stop_entries']==0


async def test_native_write_requires_completed_full_hash_check(context):
    e,p,kw,_=context;x=e.Inputs(p,kw['approved'],kw['expected_hash'],kw['expires'],kw['confirm'])
    with pytest.raises(ValueError,match='native_jog_runtime_not_verified'):x.write()


async def test_native_write_uses_verified_metadata_without_runtime_byte_hash(context,monkeypatch):
    e,p,kw,_=context;x=e.Inputs(p,kw['approved'],kw['expected_hash'],kw['expires'],kw['confirm']);x.cold()
    def forbidden():raise AssertionError('runtime bytes hashed inside lease')
    monkeypatch.setattr(e,'runtime_hashes',forbidden)
    x.write()


async def test_failed_full_recheck_invalidates_previous_verification(context,monkeypatch):
    e,p,kw,_=context;x=e.Inputs(p,kw['approved'],kw['expected_hash'],kw['expires'],kw['confirm']);x.cold()
    monkeypatch.setattr(e,'runtime_hashes',lambda:{'code.py':'0'*64})
    with pytest.raises(ValueError,match='runtime_changed'):x.cold()
    with pytest.raises(ValueError,match='runtime_not_verified'):x.write()


async def test_full_sha_after_sdk_loading_rejects_source_change_before_sdk_init(context,monkeypatch):
    from tests.robots.fake_lebai import FakeLebaiModule
    e,p,kw,c=context;sdk=FakeLebaiModule(c)
    def load():
        (e.ROOT/'code.py').write_text('changed while loading')
        return sdk
    monkeypatch.setattr(e,'require_execution_runtime',lambda:None)
    monkeypatch.setattr(e,'load_sdk',load)  # Explicit fake module; no native import.
    r=await e.execute(p,**kw)
    assert not r['complete'] and r['speedl_entries']==r['stop_entries']==0
    assert sdk.init_count==0 and not sdk.connect_calls
    assert not r['connection_diagnostics']['native_sdk_init_entered']


@pytest.mark.parametrize('change',['runtime','config','preview','reference','expiry','identity'])
async def test_metadata_write_still_rejects_changed_authority(context,monkeypatch,change):
    e,p,kw,_=context;x=e.Inputs(p,kw['approved'],kw['expected_hash'],kw['expires'],kw['confirm']);x.cold()
    if change=='runtime':(e.ROOT/'code.py').write_text('changed source')
    elif change=='config':p['config'].write_bytes(p['config'].read_bytes()+b'\n# changed')
    elif change=='preview':kw['approved'].write_text('{}')
    elif change=='reference':p['reference_survey'].write_bytes(p['reference_survey'].read_bytes()+b' ')
    elif change=='expiry':x.expires='2000-01-01T00:00:00Z'
    else:
        original=Path.stat;code=e.ROOT/'code.py'
        from types import SimpleNamespace
        def replaced(path,*args,**kwargs):
            s=original(path,*args,**kwargs)
            if path==code:
                return SimpleNamespace(st_dev=s.st_dev+1,st_ino=s.st_ino,st_size=s.st_size,
                    st_ctime_ns=s.st_ctime_ns,st_mtime_ns=s.st_mtime_ns)
            return s
        monkeypatch.setattr(Path,'stat',replaced)
    with pytest.raises(ValueError):x.write()


async def test_old_full_sha_at_write_preview_cannot_authorize_metadata_policy(context):
    e,p,kw,c=context;old=e.preview(p)
    old['runtime_check']='full_sha_before_lease_and_actual_write_then_original_freshness'
    kw['approved'].write_text(json.dumps(old));kw['expected_hash']=e.sha(kw['approved'])
    with pytest.raises(ValueError,match='preview_changed'):
        await e.execute(p,**kw,client_factory=AsyncMock(return_value=c))
    assert not p['output'].exists()


async def test_full_runtime_hash_finishes_after_loading_and_before_fk_lease(context,monkeypatch):
    e,p,kw,c=context;events=[];original=e.runtime_hashes;fk=c.kinematics_forward;speed=c.speedl
    def hashes():events.append('hash');return original()
    async def factory(ip):events.append('loaded');return c
    async def measured_fk(q):events.append('fk');return await fk(q)
    async def measured_speed(*args):events.append('speed');return await speed(*args)
    monkeypatch.setattr(e,'runtime_hashes',hashes);c.kinematics_forward=measured_fk;c.speedl=measured_speed
    r=await e.execute(p,**kw,client_factory=factory)
    assert r['complete'],r
    assert events.index('loaded')<max(i for i,v in enumerate(events) if v=='hash')<events.index('fk')
    assert events[-1]=='speed'


async def test_runtime_metadata_scan_preserves_order_and_joins_all_workers(context,monkeypatch):
    import threading
    e,p,kw,_=context
    names=[f'module{i}.py' for i in range(16)]
    for name in names:(e.ROOT/name).write_text(name)
    p['runtime_hashes']={name:e.sha(e.ROOT/name) for name in names}
    x=e.Inputs(p,kw['approved'],kw['expected_hash'],kw['expires'],kw['confirm'])
    original=Path.stat;workers=set()
    def observed(path,*args,**kwargs):
        if path.name in names:workers.add(threading.get_ident())
        return original(path,*args,**kwargs)
    # Resolve/validate paths before measuring the metadata scan itself.
    x.paths();monkeypatch.setattr(Path,'stat',observed)
    actual=x.fingerprints()
    expected=[(s.st_dev,s.st_ino,s.st_size,s.st_mtime_ns,s.st_ctime_ns)
        for name in names for s in (original(e.ROOT/name),)]
    assert actual==expected
    assert threading.get_ident() not in workers and workers
    assert not any(t.name.startswith('jog-metadata') for t in threading.enumerate())
