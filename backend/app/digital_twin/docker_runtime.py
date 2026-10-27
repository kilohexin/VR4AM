"""Official controller simulation: isolated configuration and local admission."""
import asyncio
import json
import math
import os
import subprocess
import time
from dataclasses import replace
from app.config import TcpExpectation
from app.digital_twin.runtime import load_digital_twin_settings

IMAGE='registry.cn-shanghai.aliyuncs.com/lebai/l-master'


class DockerClock:
    def now_ns(self):return time.perf_counter_ns()


def docker_settings():
    s=load_digital_twin_settings();ready=(0.,-math.pi/4,math.pi/2,-math.pi/4,math.pi/2,0.)
    control=replace(s.lebai.control,max_joint_speed_radps=.05,max_joint_acceleration_radps2=.2,
        max_tcp_speed_mps=.005,max_tcp_acceleration_mps2=.02,
        max_tcp_rotation_radps=.1,max_tcp_angular_acceleration_radps2=.2,translation_scale=1.)
    return replace(s,lebai=replace(s.lebai,ip='127.0.0.1',expected_tcp=TcpExpectation(0.,0.,0.,0.,0.,0.),
        home_q=ready,teleop_ready_q=ready,control=control),segmented_target=None,transactional_target=None)


def verify_local_docker(container=None):
    """Read metadata only; never pull/restart/reconfigure a container."""
    try:
        if os.environ.get('DOCKER_HOST'):
            raise ValueError('docker_simulator_not_admitted')
        endpoint=json.loads(subprocess.run(['docker','context','inspect','--format','{{json .Endpoints.docker.Host}}'],
            capture_output=True,text=True,check=True,timeout=5).stdout)
        if not isinstance(endpoint,str) or not endpoint.startswith(('npipe:////./pipe/','unix:///')):
            raise ValueError('docker_simulator_not_admitted')
        if container is None:
            candidates=subprocess.run(['docker','ps','--filter','ancestor='+IMAGE+':latest','--format','{{.ID}}'],
                capture_output=True,text=True,check=True,timeout=5).stdout.split()
            if len(candidates)!=1:raise ValueError('docker_simulator_not_admitted')
            container=candidates[0]
        rows=json.loads(subprocess.run(['docker','inspect',container],capture_output=True,text=True,
            check=True,timeout=5).stdout)
        item=rows[0];config=item['Config'];host=item['HostConfig'];ports=item['NetworkSettings']['Ports']
        if (item['State']['Running'] is not True or not config['Image'].startswith(IMAGE+':')
                or config['Cmd']!=['/data/startup.sh'] or host['Privileged'] or host.get('Devices')
                or item.get('Mounts') or host['NetworkMode']=='host'
                or not any(p['HostPort']=='3030' and p.get('HostIp') in ('127.0.0.1','0.0.0.0')
                    for p in ports.get('3030/tcp',[]) or [])
                or not any(p['HostPort']=='3020' and p.get('HostIp') in ('127.0.0.1','0.0.0.0')
                    for p in ports.get('3020/tcp',[]) or [])
                or ports.get('3031/tcp')):
            raise ValueError('docker_simulator_not_admitted')
        return dict(container_id=item['Id'],image_id=item['Image'],target='127.0.0.1',port=3030)
    except (KeyError,IndexError,TypeError,AttributeError,json.JSONDecodeError,subprocess.SubprocessError,OSError):
        raise ValueError('docker_simulator_not_admitted') from None


async def ensure_docker_idle(client):
    state=await client.get_robot_state()
    if state=='STOP':
        await client.start_sys()
        for _ in range(100):
            state=await client.get_robot_state()
            if state=='IDLE':return
            if state not in ('STARTING','BOOTING','ROBOT_ON','STOP'):break
            await asyncio.sleep(.1)
    if state!='IDLE':raise ValueError('docker_simulator_requires_idle')


def create_docker_app(web_directory):
    import os
    import uuid
    from pathlib import Path
    from fastapi.staticfiles import StaticFiles
    from app.main import create_app
    directory=Path(web_directory).resolve()
    if not (directory/'index.html').is_file():raise ValueError('docker_simulator_web_build_missing')
    app=create_app(settings=docker_settings(),backend_label='LEBAI_DOCKER',docker_simulation=True)
    run_id=str(uuid.uuid4())
    @app.get('/simulation/status')
    async def status():
        backend=app.state.backend;client=backend._client
        return dict(runtime='LEBAI_DOCKER',provider='official_lmaster',sdk_target='127.0.0.1:3030',
            simulator=True,hardware_verified=False,run_id=run_id,pid=os.getpid(),
            gripper_provider='local_virtual',gripper_physics=False,
            timing_profile='virtual_controller',stop_rpc_timeout_ms=1000,stop_verification_timeout_ms=2000,
            moves=getattr(client,'move_entries',0),claws=getattr(client,'claw_entries',0),
            control_fault=app.state.control._fault,
            unknown_reads=sorted(backend._snapshot_calls.unknown_methods),
            unknown_writes=sorted(backend._motion_calls.unknown_methods),
            admission=backend._docker_admission,model_sha256=backend.position_model.model_sha256)
    app.mount('/',StaticFiles(directory=directory,html=True),name='official-docker')
    return app
