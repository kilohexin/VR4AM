import json
from types import SimpleNamespace
import pytest


def container():
    return dict(Id='container-id',Image='image-id',State=dict(Running=True),Mounts=[],
        Config=dict(Image='registry.cn-shanghai.aliyuncs.com/lebai/l-master:latest',Cmd=['/data/startup.sh']),
        HostConfig=dict(Privileged=False,Devices=[],NetworkMode='bridge'),
        NetworkSettings=dict(Ports={'3030/tcp':[{'HostIp':'0.0.0.0','HostPort':'3030'}],'3020/tcp':[{'HostIp':'0.0.0.0','HostPort':'3020'}]}))


@pytest.mark.parametrize('damage',[None,'image','port','device','mount','privileged','stopped','cmd'])
def test_admission_checks_official_local_simulation_mapping(monkeypatch,damage):
    from app.digital_twin import docker_runtime as d
    row=container()
    if damage=='image':row['Config']['Image']='other:latest'
    if damage=='port':row['NetworkSettings']['Ports']['3030/tcp'][0]['HostPort']='3031'
    if damage=='device':row['HostConfig']['Devices']=['hardware']
    if damage=='mount':row['Mounts']=['host-data']
    if damage=='privileged':row['HostConfig']['Privileged']=True
    if damage=='stopped':row['State']['Running']=False
    if damage=='cmd':row['Config']['Cmd']=['other']
    calls=[]
    def inspect(args,**kwargs):
        calls.append(args)
        return SimpleNamespace(stdout=json.dumps('npipe:////./pipe/dockerDesktopLinuxEngine' if args[1]=='context' else [row]))
    monkeypatch.setattr(d.subprocess,'run',inspect)
    if damage:
        with pytest.raises(ValueError,match='not_admitted'):d.verify_local_docker('local-sim')
    else:assert d.verify_local_docker('local-sim')['port']==3030
    assert calls[-1]==['docker','inspect','local-sim']


@pytest.mark.parametrize('damage',['remote_context','host_override','other_bind'])
def test_admission_requires_the_inspected_daemon_and_port_to_be_local(monkeypatch,damage):
    from app.digital_twin import docker_runtime as d
    row=container()
    monkeypatch.delenv('DOCKER_HOST',raising=False)
    if damage=='host_override':monkeypatch.setenv('DOCKER_HOST','tcp://remote:2375')
    if damage=='other_bind':row['NetworkSettings']['Ports']['3030/tcp'][0]['HostIp']='192.168.1.1'
    endpoint='ssh://remote' if damage=='remote_context' else 'npipe:////./pipe/dockerDesktopLinuxEngine'
    monkeypatch.setattr(d.subprocess,'run',lambda args,**kw:SimpleNamespace(stdout=json.dumps(endpoint if args[1]=='context' else [row])))
    with pytest.raises(ValueError,match='not_admitted'):d.verify_local_docker('local-sim')


@pytest.mark.parametrize('damage',[None,'shared_port','public_bind','no_name','real_port','extra_bind'])
def test_explicit_test_instance_requires_its_own_fixed_loopback_mappings(monkeypatch,damage):
    from app.digital_twin import docker_runtime as d
    row=container()
    for key,port in (('3030/tcp','13030'),('3020/tcp','13020')):
        row['NetworkSettings']['Ports'][key]=[dict(HostIp='127.0.0.1',HostPort=port)]
    if damage=='shared_port':row['NetworkSettings']['Ports']['3030/tcp'][0]['HostPort']='3030'
    if damage=='public_bind':row['NetworkSettings']['Ports']['3030/tcp'][0]['HostIp']='0.0.0.0'
    if damage=='real_port':row['NetworkSettings']['Ports']['3031/tcp']=[dict(HostIp='127.0.0.1',HostPort='13031')]
    if damage=='extra_bind':row['NetworkSettings']['Ports']['3030/tcp'].append(dict(HostIp='0.0.0.0',HostPort='13030'))
    monkeypatch.delenv('DOCKER_HOST',raising=False)
    monkeypatch.setattr(d.subprocess,'run',lambda args,**kw:SimpleNamespace(stdout=json.dumps(
        'npipe:////./pipe/dockerDesktopLinuxEngine' if args[1]=='context' else [row])))
    if damage:
        with pytest.raises(ValueError,match='not_admitted'):
            d.verify_local_docker(None if damage=='no_name' else 'test-instance',isolated_test=True)
    else:
        assert d.verify_local_docker('test-instance',isolated_test=True)['port']==13030


def test_test_container_is_rejected_outside_explicit_docker_backend(tmp_path):
    from app.main import create_app
    with pytest.raises(ValueError,match='explicit_selection_required'):
        create_app(docker_test_container='test-only')


def test_isolated_launcher_preview_pins_verified_id_and_cannot_use_main_ui_port(monkeypatch,capsys):
    from app.digital_twin import docker_runtime as d
    from scripts.run_lmaster_docker import main
    calls=[]
    def verify(container,*,isolated_test):
        calls.append((container,isolated_test))
        return dict(container_id='verified-id',image_id='image-id',target='127.0.0.1',port=13030)
    monkeypatch.setattr(d,'verify_local_docker',verify)
    assert main(['--isolated-test-container','test-only','--port','8774','--pose-follow-vr'])==0
    report=json.loads(capsys.readouterr().out)
    assert calls==[('test-only',True)] and not report['execute']
    assert report['sdk_target']=='127.0.0.1:13030' and report['test_seconds']==120
    with pytest.raises(SystemExit):main(['--isolated-test-container','test-only'])
    with pytest.raises(SystemExit):main(['--isolated-test-container','','--port','8774'])
    with pytest.raises(SystemExit):main(['--test-seconds','60'])
    assert len(calls)==1


@pytest.mark.parametrize('initial',['STOP','ROBOT_ON','IDLE','MOVING','PAUSED','ERROR','BOOTING','STARTING'])
async def test_virtual_start_only_occurs_from_unstarted_states_and_never_resumes_or_resets(initial):
    from app.digital_twin.docker_runtime import ensure_docker_idle
    states=iter([initial,'STARTING','IDLE']);writes=[]
    async def get_state():return next(states)
    async def start():writes.append('start_sys')
    client=SimpleNamespace(get_robot_state=get_state,start_sys=start)
    if initial in ('STOP','ROBOT_ON','IDLE'):await ensure_docker_idle(client)
    else:
        with pytest.raises(ValueError,match='requires_idle'):await ensure_docker_idle(client)
    assert writes==(['start_sys'] if initial in ('STOP','ROBOT_ON') else [])


@pytest.mark.parametrize('pose',[False,True])
def test_launcher_preview_never_loads_or_connects_sdk(monkeypatch,capsys,pose):
    import sys
    from app.digital_twin import docker_runtime as d
    from scripts.run_lmaster_docker import main
    before='lebai_sdk' in sys.modules
    monkeypatch.setattr(d,'verify_local_docker',lambda:dict(container_id='local-id',image_id='image-id',target='127.0.0.1',port=3030))
    assert main(['--pose-follow-vr'] if pose else [])==0
    report=json.loads(capsys.readouterr().out)
    assert report['execute'] is False and report['sdk_target']=='127.0.0.1:3030'
    assert report['pose_follow_vr'] is pose
    assert report['pose_follow_quarantined'] is pose
    assert report['pose_follow_window_s']==(300 if pose else None)
    assert report['pose_follow_grip_span_mm']==(300 if pose else None)
    assert ('lebai_sdk' in sys.modules)==before


async def test_docker_served_build_cannot_connect_to_an_embedded_foreign_ws_url(tmp_path):
    import httpx
    from app.digital_twin.docker_runtime import create_docker_app
    (tmp_path/'index.html').write_text('<html>Docker</html>',encoding='utf-8')
    # ASGI transport intentionally does not start lifespan or load an SDK.
    app=create_docker_app(tmp_path)
    async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app),base_url='http://localhost:8773') as c:
        r=await c.get('/')
    assert r.status_code==200
    assert r.headers['content-security-policy']=="connect-src 'self' ws://localhost:8773 wss://localhost:8773"
