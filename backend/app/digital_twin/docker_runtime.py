"""Official controller simulation: isolated configuration and local admission."""
import asyncio
import json
import math
import os
import re
import subprocess
import time
from dataclasses import replace
from app.config import TcpExpectation
from app.digital_twin.runtime import load_digital_twin_settings
from app.control.docker_pose_profile import POSE_PROFILE

IMAGE='registry.cn-shanghai.aliyuncs.com/lebai/l-master'


class DockerClock:
    def now_ns(self):return time.perf_counter_ns()


def docker_settings():
    s=load_digital_twin_settings();ready=(0.,-math.pi/4,math.pi/2,-math.pi/4,math.pi/2,0.)
    control=replace(s.lebai.control,max_joint_speed_radps=.6,max_joint_acceleration_radps2=.2,
        max_tcp_speed_mps=.15,max_tcp_acceleration_mps2=.15,
        max_tcp_rotation_radps=1.,max_tcp_angular_acceleration_radps2=1.,translation_scale=1.)
    return replace(s,lebai=replace(s.lebai,ip='127.0.0.1',expected_tcp=TcpExpectation(0.,0.,0.,0.,0.,0.),
        home_q=ready,teleop_ready_q=ready,control=control),segmented_target=None,transactional_target=None)


def verify_local_docker(container=None,*,isolated_test=False):
    """Read metadata only; never pull/restart/reconfigure a container."""
    try:
        if type(isolated_test) is not bool or isolated_test and not container:
            raise ValueError('docker_simulator_not_admitted')
        sdk_port,admin_port=(13030,13020) if isolated_test else (3030,3020)
        allowed_bind=('127.0.0.1',) if isolated_test else ('127.0.0.1','0.0.0.0')
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
        if isolated_test and (set(k for k,v in ports.items() if v)!={'3030/tcp','3020/tcp'}
                or any(len(ports.get(k) or [])!=1 for k in ('3030/tcp','3020/tcp'))):
            raise ValueError('docker_simulator_not_admitted')
        if (item['State']['Running'] is not True or not config['Image'].startswith(IMAGE+':')
                or config['Cmd']!=['/data/startup.sh'] or host['Privileged'] or host.get('Devices')
                or item.get('Mounts') or host['NetworkMode']=='host'
                or not any(p['HostPort']==str(sdk_port) and p.get('HostIp') in allowed_bind
                    for p in ports.get('3030/tcp',[]) or [])
                or not any(p['HostPort']==str(admin_port) and p.get('HostIp') in allowed_bind
                    for p in ports.get('3020/tcp',[]) or [])
                or ports.get('3031/tcp')):
            raise ValueError('docker_simulator_not_admitted')
        return dict(container_id=item['Id'],image_id=item['Image'],target='127.0.0.1',port=sdk_port)
    except (KeyError,IndexError,TypeError,AttributeError,json.JSONDecodeError,subprocess.SubprocessError,OSError):
        raise ValueError('docker_simulator_not_admitted') from None


async def ensure_docker_idle(client,*,check=lambda:None):
    check()
    state=await client.get_robot_state()
    check()
    if state in ('STOP','ROBOT_ON'):
        check()
        await client.start_sys()
        check()
        for _ in range(100):
            check()
            state=await client.get_robot_state()
            check()
            if state=='IDLE':return
            if state not in ('STARTING','BOOTING','ROBOT_ON','STOP'):break
            await asyncio.sleep(.1)
    if state!='IDLE':raise ValueError('docker_simulator_requires_idle')


async def ensure_docker_speed(client,*,check=lambda:None):
    """Startup-only on the already verified local simulator, before any target."""
    check();factor=await client.get_velocity_factor();check()
    if type(factor) is not int or not 0<=factor<=100:
        raise ValueError('docker_speed_factor_invalid')
    if factor!=100:
        check();await client.set_velocity_factor(100);check()
        factor=await client.get_velocity_factor();check()
    if factor!=100:raise ValueError('docker_speed_factor_unconfirmed')
    return factor


def create_docker_app(web_directory,*,blended_translation=False,blended_rotation=False,pvat_vr=False,pose_follow_vr=False,docker_test_container=None):
    if pose_follow_vr:pvat_vr=True
    if pvat_vr:blended_rotation=True
    if blended_rotation is True:blended_translation=True
    import os
    import uuid
    from pathlib import Path
    from fastapi.staticfiles import StaticFiles
    from app.main import create_app
    directory=Path(web_directory).resolve()
    if not (directory/'index.html').is_file():raise ValueError('docker_simulator_web_build_missing')
    app=create_app(settings=docker_settings(),backend_label='LEBAI_DOCKER',docker_simulation=True,
        docker_test_container=docker_test_container)
    from contextlib import asynccontextmanager
    original_lifespan=app.router.lifespan_context
    @asynccontextmanager
    async def lifespan(application):
        async with original_lifespan(application) as state:
            application.state.backend.position_blended_translation=blended_translation is True
            application.state.backend.position_blended_rotation=blended_rotation is True
            application.state.backend.position_pvat_vr=pvat_vr is True
            application.state.backend.position_pose_follow_vr=pose_follow_vr is True
            yield state
    app.router.lifespan_context=lifespan
    @app.middleware('http')
    async def same_origin_control(request,call_next):
        response=await call_next(request)
        # Also constrain a dist built with an old VITE_TELEOP_WS_URL override.
        # Explicit WS schemes support browsers that do not match them to 'self'.
        host=request.url.netloc
        if isinstance(host,bytes):host=host.decode('ascii')
        sources=f" ws://{host} wss://{host}" if re.fullmatch(r'[A-Za-z0-9.\[\]:-]+',host) else ''
        response.headers['Content-Security-Policy']="connect-src 'self'"+sources
        return response
    run_id=str(uuid.uuid4())
    @app.get('/simulation/status')
    async def status():
        backend=app.state.backend;client=backend._client
        return dict(runtime='LEBAI_DOCKER',provider='official_lmaster',
            sdk_target='127.0.0.1:'+str(13030 if docker_test_container is not None else 3030),
            isolated_test=docker_test_container is not None,
            simulator=True,hardware_verified=False,run_id=run_id,pid=os.getpid(),pvat_vr=pvat_vr,
            pose_follow_vr=pose_follow_vr,
            pose_follow_window_s=POSE_PROFILE.duration_ms/1000 if pose_follow_vr else None,
            pose_follow_grip_span_mm=POSE_PROFILE.target_tcp_span_m*1000 if pose_follow_vr else None,
            pose_mailbox_ttl_ms=POSE_PROFILE.mailbox_ttl_ms if pose_follow_vr else None,
            pose_reference_speed_radps=POSE_PROFILE.reference_speed_radps if pose_follow_vr else None,
            pose_translation_scale=POSE_PROFILE.translation_scale if pose_follow_vr else None,
            pvat_controller_points=getattr(backend,'position_pvat_points',0),
            pvat_reference_rollovers=getattr(backend,'position_pvat_reference_rollovers',0),
            pvat_nominal_speed_mmps=10 if pvat_vr and not pose_follow_vr else None,
            gripper_provider='local_virtual',gripper_physics=False,
            kinematics_provider='controller_ik_verified_local_fk',native_fk_admission_queries=3,
            timing_profile='virtual_controller',stop_rpc_timeout_ms=1000,stop_verification_timeout_ms=2000,
            motion_profile='docker_blended_candidate' if backend.position_blended_translation else 'docker_functional',startup_velocity_factor_percent=getattr(backend,'_docker_velocity_factor',None),
            translation_step_mm=4. if backend.position_blended_translation else backend.position_step_m*1000,angle_step_deg=math.degrees(backend.position_angle_step_rad),
            blended_translation=backend.position_blended_translation,
            blended_rotation=backend.position_blended_rotation,
            rotation_blend_step_deg=1. if backend.position_blended_rotation else None,
            rotation_blend_lead_deg=2. if backend.position_blended_rotation else None,
            joint_speed_cap_radps=backend.settings.control.max_joint_speed_radps,
            tcp_speed_cap_mps=backend.settings.control.max_tcp_speed_mps,
            moves=getattr(client,'move_entries',0),claws=getattr(client,'claw_entries',0),
            control_fault=app.state.control._fault,
            control_mode=app.state.control.mode.value,session_owned=app.state.teleop_owner is not None,
            unknown_reads=sorted(backend._snapshot_calls.unknown_methods),
            unknown_writes=sorted(backend._motion_calls.unknown_methods),
            admission=backend._docker_admission,model_sha256=backend.position_model.model_sha256)
    app.mount('/',StaticFiles(directory=directory,html=True),name='official-docker')
    return app
