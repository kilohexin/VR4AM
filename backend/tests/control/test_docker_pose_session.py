import asyncio
from types import SimpleNamespace as NS
from unittest.mock import AsyncMock

import pytest

from app.control.docker_pose_session import DockerPoseSession, build_pose_program
from app.control.finite_position_types import PositionIntent
from app.robots.finite_position_docker import DockerFinitePositionAdapter
from app.schemas.messages import Pose


def rig():
    now=[1_000_000_000]
    a=object.__new__(DockerFinitePositionAdapter)
    a._docker_native=True;a.position_pose_follow_vr=True;a._latched_fault=None
    a._docker_admission={'container_id':'test-only'};a.position_busy=False;a._docker_entry_owner=None
    a._position_health=lambda b:None
    a.settings=NS(startup_tcp_min_m=(-1.,)*3,startup_tcp_max_m=(1.,)*3,
        soft_joint_min_rad=(-3.,)*6,soft_joint_max_rad=(3.,)*6,joint_limit_margin_rad=.01)
    a._position_pitch_reference=(0.,0.,0.,1.)
    a._snapshot=NS(robot_state=NS(value='IDLE'),actual_tcp=Pose(p=(.3,0.,.4),q=(0.,0.,0.,1.)),captured_ns=now[0])
    a._read_snapshot=AsyncMock(side_effect=lambda **kw:a._snapshot)
    recorder=NS(fatal_error=None,dropped_normal_events=0,write_event=AsyncMock())
    binding=NS(generation=2);a.position_binding=binding
    s=DockerPoseSession(a,binding,clock=lambda:now[0],recorder=recorder,authorize=lambda:None)
    return now,a,s


def hand(ns,seq,x=0.):
    return PositionIntent('xr-finite-pose',2,seq,ns,'held',(0,0,0),hand_pose=Pose(p=(x,0.,0.),q=(0.,0.,0.,1.)))


def test_native_docker_flag_and_full_pose_are_required():
    now,a,s=rig()
    with pytest.raises(ValueError,match='requires_pose'):
        s.offer(PositionIntent('xr-finite-pose',2,1,now[0],'held',(1,0,0)))
    a.position_pose_follow_vr=False
    with pytest.raises(ValueError,match='not_enabled'):
        DockerPoseSession(a,NS(generation=2),clock=lambda:now[0],recorder=s.recorder,authorize=lambda:None)


def test_native_program_has_no_implicit_prepare_and_fixed_absolute_bounds():
    _,a,_=rig();source=build_pose_program(a)
    assert '__POSE_MAILBOX' not in source and '-- PRODUCT_POSE_BOUNDS' not in source
    assert 'p.x<-1.0' in source and 'q[1]<-2.99' in source
    assert 'lebai:move_pvat' in source
    for forbidden in ('start_sys(', 'movej(', 'set_claw(', 'stop_sys('):
        assert forbidden not in source


def test_extended_profile_keeps_fixed_grip_and_absolute_joint_limits():
    from app.control.docker_pose_profile import POSE_PROFILE
    _,a,_=rig();source=build_pose_program(a)
    assert POSE_PROFILE.duration_ms==300000 and POSE_PROFILE.maximum_points==3750
    assert POSE_PROFILE.target_tcp_span_m==.1 and POSE_PROFILE.feedback_tcp_span_m==.12
    assert POSE_PROFILE.mailbox_ttl_ms==250 and 'mailbox_ttl_ms=250' in source
    assert '-- PRODUCT_POSE_PROFILE' not in source
    assert 'joint_min=pose_joint_min,joint_max=pose_joint_max' in source
    assert 'joints[i]-origin[i]' not in source
    # Fixed base is assigned once from the original observation; it never
    # follows the hand or reference to silently enlarge an active gesture.
    assert source.count('base=k.actual_tcp_pose')==1
    assert 'base=sample' not in source


@pytest.mark.parametrize('expired',[False,True])
@pytest.mark.parametrize('record_delay',[0,20_000_000])
async def test_delayed_start_keeps_grip_anchor_and_known_release_cleans_up(monkeypatch,expired,record_delay):
    from app.digital_twin import docker_runtime
    import app.control.docker_pose_session as module
    now,a,s=rig()
    from tests.control.test_vr_pose_input import Signals
    client=Signals();a._docker_ready_client=client
    s.offer(hand(now[0],1))
    now[0]+=20_000_000
    s.offer(hand(now[0],2,.001))
    a._snapshot.captured_ns=now[0]
    captured=[]
    stamps=[]
    async def record(event,stamp):
        if event.get('kind')=='docker_pose_controller' and event['fields'][0]=='LIVE_READY' and record_delay:
            now[0]+=record_delay
            a._snapshot.captured_ns=now[0]
            s.offer(hand(now[0],3,.001))
    s.recorder.write_event=AsyncMock(side_effect=record)
    class Input:
        def write(self,data):assert b'product_bounds' in data
        async def drain(self):pass
        def close(self):pass
    class Output:
        n=0
        async def readline(self):
            self.n+=1
            if self.n==1:return b'LIVE_READY\t1000\n'
            if self.n==2:
                async with asyncio.timeout(1):
                    while client.values[240]==0:await asyncio.sleep(.001)
                captured.append(client.values[241])
                stamps.append(client.values[240]//1024)
                return b'LIVE_POINT\t20\t2\t.3\t0\t.4\t0\n'
            if self.n==3:
                if expired:return b'LIVE_EXPIRED\t300\t1\n'
                s.offer(PositionIntent('xr-finite-pose',2,4,now[0],'release',(0,0,0)))
                return b'LIVE_QUIET\t500\n'
            if expired and self.n==4:return b'LIVE_QUIET\t500\n'
            return b'LIVE_DONE\ttrue\tnil\t1\n'
    proc=NS(stdin=Input(),stdout=Output(),wait=AsyncMock(return_value=0))
    monkeypatch.setattr(docker_runtime,'verify_local_docker',lambda:a._docker_admission)
    monkeypatch.setattr(module.asyncio,'create_subprocess_exec',AsyncMock(return_value=proc))
    await s._run()
    assert captured==[302000]  # +2mm from the ORIGINAL Grip zero, not startup hand.
    assert stamps==[1000+record_delay//1_000_000]  # Logging does not backdate future input.
    assert s.phase==('hold' if expired else 'ready') and not a.position_busy and a._latched_fault is None
    if expired:assert s.reason=='position_input_expired'
    assert s.pump.completed and s.pump.lease.owner is None


async def test_release_during_open_does_not_launch_native_program(monkeypatch):
    import app.control.docker_pose_session as module
    from app.digital_twin import docker_runtime
    now,a,s=rig();s.offer(hand(now[0],1))
    entered,finish=asyncio.Event(),asyncio.Event()
    async def signal(i):entered.set();await finish.wait();return 0
    a._docker_ready_client=NS(get_signal=signal)
    monkeypatch.setattr(docker_runtime,'verify_local_docker',lambda:a._docker_admission)
    spawn=AsyncMock();monkeypatch.setattr(module.asyncio,'create_subprocess_exec',spawn)
    task=asyncio.create_task(s._run());await entered.wait()
    s.offer(PositionIntent('xr-finite-pose',2,2,now[0],'release',(0,0,0)))
    finish.set();await task
    assert s.phase=='ready' and not a.position_busy
    spawn.assert_not_called()


@pytest.mark.parametrize('completed,busy',[(True,False),(False,False),(True,True)])
async def test_regrip_while_previous_worker_records_its_completed_tail(completed,busy):
    now,a,s=rig();s.offer(hand(now[0],1));old=s._reference
    old.release()
    s.gate.offer(PositionIntent('xr-finite-pose',2,2,now[0],'release',(0,0,0)))
    # The native program/cleanup finished, but final logging still owns the
    # worker slot. A ready status must not reuse its retired Grip anchor.
    s.worker=asyncio.get_running_loop().create_future()
    s.pump=NS(closed=True,completed=completed)
    s.phase='ready';a.position_busy=busy
    s.offer(hand(now[0],3,.1))
    if completed and not busy:
        assert s._reference is not old and s._reference.generation==2
        goal=s._reference.target(Pose(p=(.101,0,0),q=(0,0,0,1)),generation=2,seq=4,captured_ns=now[0])
        assert goal.p[0]==pytest.approx(.302)
    else:assert s._reference is old and old.generation is None
    s.worker.cancel()
