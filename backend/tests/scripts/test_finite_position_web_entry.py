import importlib.util
from datetime import datetime,UTC,timedelta
from pathlib import Path
import pytest

SOURCE=Path(__file__).resolve().parents[3]/'scripts/run_finite_position_web.py'


def api():
    assert SOURCE.exists(),'explicit default-preview real web launcher missing'
    spec=importlib.util.spec_from_file_location('web_position_entry',SOURCE)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def test_no_arguments_cannot_start_real_server():
    m=api()
    with pytest.raises(SystemExit):m.parser().parse_args([])
    assert m.parser().parse_args(['--config','a','--reference-survey','b',
        '--ready-pose','c','--output','d']).execute is False

def test_vr_opt_in_remains_default_off_in_preview_cli():
    m=api();args=['--config','a','--reference-survey','b','--ready-pose','c','--output','d']
    assert m.parser().parse_args(args).allow_vr is False
    assert m.parser().parse_args(args+['--allow-vr']).allow_vr is True


def test_budget_scope_rejects_late_motion_but_reserves_stop_and_readonly_cleanup():
    m=api();now=[0];checked=[]
    guard=m.WebWindow(lambda:checked.append(True),clock=lambda:now[0])
    guard.check();guard.entry('connect');guard.entry('movej');guard.entry('set_claw')
    now[0]=m.DURATION_NS
    with pytest.raises(ValueError,match='expired'):guard.check()
    with pytest.raises(ValueError,match='expired'):guard.entry('movej')
    guard.entry('stop_move');guard.entry('get_kin_data')
    now[0]+=m.CLEANUP_NS
    with pytest.raises(ValueError):guard.entry('get_kin_data')


def test_no_reconnect_no_unlisted_write_and_no_auto_resume():
    m=api();guard=m.WebWindow(lambda:None,clock=lambda:0)
    guard.entry('connect')
    with pytest.raises(ValueError):guard.entry('connect')
    for method in ('stop_sys','resume_move','pause_move','move_pvat'):
        with pytest.raises(ValueError):guard.entry(method)
    for _ in range(m.MOVE_LIMIT):guard.entry('movej')
    with pytest.raises(ValueError):guard.entry('movej')
    guard.entry('stop_move')


def test_scope_expiry_during_identity_work_is_rejected():
    m=api();now=[m.DURATION_NS-1]
    def identity():now[0]=m.DURATION_NS
    guard=m.WebWindow(identity,clock=lambda:now[0]);guard.started=0
    with pytest.raises(ValueError,match='expired'):guard.entry('movej')
    assert guard.entries['movej']==0


async def test_stuck_server_cleanup_returns_unresolved_without_waiting_forever(monkeypatch):
    import asyncio,time
    m=api();monkeypatch.setattr(m,'DURATION_NS',20_000_000);monkeypatch.setattr(m,'CLEANUP_NS',40_000_000)
    release=asyncio.Event()
    class Stuck:
        should_exit=False
        async def serve(self):
            while not release.is_set():
                try:await release.wait()
                except asyncio.CancelledError:continue
    server=Stuck();start=time.perf_counter()
    report=await m.run_server_window(server,m.WebWindow(lambda:None))
    assert report['cleanup_unresolved'] and server.should_exit and time.perf_counter()-start<.5
    release.set();await asyncio.sleep(0)


async def test_native_counts_exclude_rejected_final_entry():
    m=api();calls=[]
    class Client:
        async def movej(self,*args):calls.append('movej')
    window=m.WebWindow(lambda:None,clock=lambda:0);client=m.WindowClient(Client(),window)
    def rejected():raise ValueError('position_snapshot_expired')
    with pytest.raises(ValueError):await client._position_guarded_movej(([],0,0,0,0),rejected)
    assert not calls and window.native_entries['movej']==0 and window.entries['movej']==1


async def test_proxy_latches_failed_native_write_and_never_retries():
    m=api();calls=[]
    class Client:
        async def movej(self,*args):calls.append('movej');raise TimeoutError()
        async def set_claw(self,*args):calls.append('set_claw')
        async def stop_move(self):calls.append('stop_move')
    guard=m.WebWindow(lambda:None,clock=lambda:0);client=m.WindowClient(Client(),guard)
    with pytest.raises(TimeoutError):await client.movej([],0,0,0,0)
    with pytest.raises(ValueError):await client.set_claw(10,20)
    await client.stop_move()
    assert calls==['movej','stop_move']


def test_preview_is_pure_and_binds_assets(tmp_path,monkeypatch,capsys):
    m=api();web=tmp_path/'web';web.mkdir();(web/'index.html').write_text('synthetic')
    output=tmp_path/'unused'
    from dataclasses import replace
    from app.config import Settings
    from tests.robots.real_settings import control_settings
    from app.commissioning.finite_position_site import MdhPositionModel
    s=control_settings(ip='192.168.42.10')
    p=dict(settings=s,model=MdhPositionModel.from_settings(s),input_hashes={},runtime_hashes={},
        authorization_expires_utc=None,output=output,arrival_policy='coarse_grasp')
    monkeypatch.setattr(m.single,'prepare',lambda *a,**kw:p)
    monkeypatch.setattr(m,'serve',lambda *a,**kw:pytest.fail('server forbidden in preview'))
    assert m.main(['--config','a','--reference-survey','b','--ready-pose','c','--output',str(output),
        '--web-directory',str(web)])==0
    import json
    row=json.loads(capsys.readouterr().out)
    assert row['scope']=='bounded_web_finite_position_and_gripper'
    assert row['max_movej_entries']==m.MOVE_LIMIT and row['normal_release']=='finish_current_step_no_next_target'
    assert row['arrival_policy']['name']=='coarse_grasp' and row['home_enabled'] is False
    assert row['web_hashes'] and row['motion_authorized'] is False
    assert str(m.COMMON_CONFIG.resolve()) in row['input_hashes']
    assert row['application_settings_binding']['lebai']==row['settings_binding']
    assert not output.exists()


def test_common_config_change_after_preview_cannot_be_approved(tmp_path,monkeypatch):
    m=api()
    from tests.robots.real_settings import control_settings
    from app.commissioning.finite_position_site import MdhPositionModel
    import json
    common=tmp_path/'default.yaml';common.write_bytes(m.COMMON_CONFIG.read_bytes())
    monkeypatch.setattr(m,'COMMON_CONFIG',common)
    web=tmp_path/'web';web.mkdir();(web/'index.html').write_text('synthetic')
    s=control_settings(ip='192.168.42.10');expiry=(datetime.now(UTC)+timedelta(minutes=5)).isoformat()
    p=dict(settings=s,model=MdhPositionModel.from_settings(s),input_hashes={},runtime_hashes={},
        authorization_expires_utc=expiry,output=tmp_path/'unused',arrival_policy='coarse_grasp')
    row=m.preview(p,web);approved=tmp_path/'approved.json';approved.write_text(json.dumps(row))
    digest=m.single.sha(approved)
    m.approve(p,row,approved,digest,expiry,m.CONFIRM)
    common.write_text(common.read_text()+'\n# changed after approval\n')
    with pytest.raises(ValueError,match='sources_changed'):m.approve(p,row,approved,digest,expiry,m.CONFIRM)
