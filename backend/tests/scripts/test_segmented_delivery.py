import importlib.util
import json
from pathlib import Path
import sys

import pytest

ROOT=Path(__file__).resolve().parents[3]

def module():
    spec=importlib.util.spec_from_file_location('segmented_delivery_test',ROOT/'scripts/run_segmented_demo.py')
    value=importlib.util.module_from_spec(spec)
    sys.modules[spec.name]=value
    spec.loader.exec_module(value)
    return value

def artifact(tmp_path):
    value=module()
    root=tmp_path/'repository'
    (root/'web/src').mkdir(parents=True)
    (root/'web/src/main.ts').write_text('export const a=1;')
    (root/'backend/app').mkdir(parents=True)
    (root/'backend/app/example.py').write_text('x=1')
    (root/'config').mkdir()
    (root/'config/fake-lebai.yaml').write_text('fixture')
    built=tmp_path/'package'
    built.mkdir()
    (built/'index.html').write_text('<html>fixture</html>')
    value.write_manifest(root,built)
    return value,root,built

def test_manifest_accepts_current_offline_artifact(tmp_path):
    value,root,built=artifact(tmp_path)
    result=value.verify_manifest(root,built)
    assert result['runtime']=='LEBAI_FAKE'
    assert result['hardware_verified'] is False
    assert result['target_strategy']=='segmented_experimental'

@pytest.mark.parametrize('change',['source','asset','missing','escape','claim','extra_source'])
def test_stale_tampered_or_unsafe_package_is_refused(tmp_path,change):
    value,root,built=artifact(tmp_path)
    if change=='source':(root/'web/src/main.ts').write_text('changed')
    elif change=='asset':(built/'index.html').write_text('changed')
    elif change=='missing':(built/'index.html').unlink()
    else:
        path=built/'delivery-manifest.json'
        payload=json.loads(path.read_text())
        if change=='escape':payload['assets']['../outside']='0'*64
        elif change=='claim':payload['hardware_verified']=True
        else:payload['sources']['config/real-robot.local.yaml']='0'*64
        path.write_text(json.dumps(payload))
    with pytest.raises(ValueError,match='delivery_'):
        value.verify_manifest(root,built)

@pytest.mark.parametrize('flag',['--ip','--host','--config','--execute-once','--live'])
def test_cli_has_no_live_entry(flag):
    value=module()
    with pytest.raises(SystemExit) as error:
        value.parser().parse_args([flag,'172.16.0.1'])
    assert error.value.code==2


def test_check_does_not_create_an_app_or_start_a_service(tmp_path,monkeypatch):
    value,root,built=artifact(tmp_path)
    monkeypatch.setattr(value,'ROOT',root)
    monkeypatch.setattr(value,'latest_package',lambda:built)
    import app.digital_twin.segmented_demo as demo
    def forbidden(*args):raise AssertionError('check started app')
    monkeypatch.setattr(demo,'create_segmented_demo_app',forbidden)
    assert value.main(['--check'])==0


def test_failed_service_startup_is_not_reported_as_success(tmp_path,monkeypatch):
    import types
    value,root,built=artifact(tmp_path)
    monkeypatch.setattr(value,'ROOT',root)
    monkeypatch.setattr(value,'latest_package',lambda:built)
    import app.digital_twin.segmented_demo as demo
    monkeypatch.setattr(demo,'create_segmented_demo_app',lambda directory:object())
    class Server:
        started=False
        def __init__(self,*args):pass
        async def serve(self):pass
    stub=types.SimpleNamespace(run=lambda *a,**k:None,Config=lambda *a,**k:object(),Server=Server)
    monkeypatch.setitem(sys.modules,'uvicorn',stub)
    assert value.main([])==2


def test_single_source_link_cannot_bypass_containment_check(tmp_path,monkeypatch):
    value,root,built=artifact(tmp_path)
    candidate=root/'config/fake-lebai.yaml'
    original=Path.is_symlink
    # Windows hosts may disallow creating symlinks without extra privileges.
    # Simulate only the filesystem's symlink flag at this specific boundary.
    monkeypatch.setattr(Path,'is_symlink',lambda p:p==candidate or original(p))
    with pytest.raises(ValueError,match='delivery_source_escape'):
        value.verify_manifest(root,built)


def test_nested_manifest_name_is_an_asset_not_a_hash_bypass(tmp_path):
    value,root,built=artifact(tmp_path)
    (built/'nested').mkdir()
    (built/'nested/delivery-manifest.json').write_text('untracked asset')
    with pytest.raises(ValueError,match='delivery_assets_changed'):
        value.verify_manifest(root,built)


@pytest.mark.parametrize('name',['web/vite.config.ts','web/tsconfig.json'])
def test_build_configuration_change_invalidates_package(tmp_path,name):
    value,root,built=artifact(tmp_path)
    (root/name).write_text('changed build configuration')
    with pytest.raises(ValueError,match='delivery_sources_changed'):
        value.verify_manifest(root,built)


@pytest.mark.parametrize('failed,forced,completed,expected',[
    (False,False,True,0),(True,False,True,2),(False,True,True,2),(False,False,False,2)])
def test_interrupt_succeeds_only_after_verified_local_app_shutdown(tmp_path,monkeypatch,
        failed,forced,completed,expected):
    import types
    value,root,built=artifact(tmp_path)
    monkeypatch.setattr(value,'ROOT',root)
    monkeypatch.setattr(value,'latest_package',lambda:built)
    import app.digital_twin.segmented_demo as demo
    monkeypatch.setattr(demo,'create_segmented_demo_app',lambda directory:object())
    class Server:
        started=True
        force_exit=forced
        lifespan=types.SimpleNamespace(shutdown_failed=failed,error_occurred=False,
            shutdown_event=types.SimpleNamespace(is_set=lambda:completed))
        def __init__(self,*args):pass
        async def serve(self):raise KeyboardInterrupt()
    monkeypatch.setitem(sys.modules,'uvicorn',types.SimpleNamespace(
        Config=lambda *a,**k:object(),Server=Server))
    assert value.main([])==expected


def test_offline_service_uses_an_isolated_high_resolution_loop():
    import asyncio
    from types import SimpleNamespace
    value=module();captured=[]
    policy=asyncio.get_event_loop_policy()
    async def serve():
        loop=asyncio.get_running_loop();captured.append(loop)
        assert loop.vr4arm_timing_clock=='time.perf_counter_ns'
        assert loop._clock_resolution<=1e-6
    value._run_offline_server(SimpleNamespace(serve=serve))
    assert len(captured)==1 and captured[0].is_closed()
    assert asyncio.get_event_loop_policy() is policy
