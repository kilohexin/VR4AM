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


@pytest.mark.parametrize('initial',['STOP','IDLE','MOVING','PAUSED','ERROR'])
async def test_virtual_start_only_occurs_from_stop_and_never_resumes_or_resets(initial):
    from app.digital_twin.docker_runtime import ensure_docker_idle
    states=iter([initial,'STARTING','IDLE']);writes=[]
    async def get_state():return next(states)
    async def start():writes.append('start_sys')
    client=SimpleNamespace(get_robot_state=get_state,start_sys=start)
    if initial in ('STOP','IDLE'):await ensure_docker_idle(client)
    else:
        with pytest.raises(ValueError,match='requires_idle'):await ensure_docker_idle(client)
    assert writes==(['start_sys'] if initial=='STOP' else [])


def test_launcher_preview_never_loads_or_connects_sdk(monkeypatch,capsys):
    import sys
    from app.digital_twin import docker_runtime as d
    from scripts.run_lmaster_docker import main
    before='lebai_sdk' in sys.modules
    monkeypatch.setattr(d,'verify_local_docker',lambda:dict(container_id='local-id',image_id='image-id',target='127.0.0.1',port=3030))
    assert main([])==0
    report=json.loads(capsys.readouterr().out)
    assert report['execute'] is False and report['sdk_target']=='127.0.0.1:3030'
    assert ('lebai_sdk' in sys.modules)==before
