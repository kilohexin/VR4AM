"""Synthetic loopback injection only. No field poses, SDK or sockets."""
import asyncio
from dataclasses import FrozenInstanceError, replace
import json

import pytest
from scipy.spatial.transform import Rotation

from app.commissioning.native_jog_candidate import NativeJogCandidateAdapter
from app.commissioning.native_jog_model_guard import ModelJogRegionGuard
from app.commissioning.native_jog_mdh import MdhTool
from app.commissioning.native_jog_packet import NativeJogLimits, plan_jog
from app.robots.base import BackendCommandError, StopReason
from app.robots.lebai_codec import pose_to_lebai
from app.schemas.messages import Pose
from tests.commissioning.test_native_jog_mdh import independent_transform
from tests.robots.fake_lebai import FakeLebaiClient, IDLE_Q
from tests.robots.real_settings import control_settings
from tests.robots.test_lebai_adapter_control import FakeClock

SHA='a'*64
TOOL=MdhTool((0.,0.,.175),(0.,0.,0.,1.))


def test_model_binding_does_not_reserialize_settings_on_each_permission(monkeypatch):
    from app.commissioning import native_jog_model_guard as module
    args,_=fixture();guard=ModelJogRegionGuard(**args)
    def forbidden(*args,**kwargs):raise AssertionError('settings serialized inside motion lease')
    monkeypatch.setattr(module,'_identity',forbidden)
    guard.require_binding(args['settings'],args['limits'],SHA)
    object.__setattr__(args['settings'].control,'max_joint_speed_radps',.04)
    with pytest.raises(ValueError,match='binding_changed'):
        guard.require_binding(args['settings'],args['limits'],SHA)


@pytest.mark.parametrize('damage',['root','nested','tuple','type','nonfinite'])
def test_fast_model_binding_detects_in_place_changes(damage):
    args,_=fixture();guard=ModelJogRegionGuard(**args);s=args['settings']
    if damage=='root':object.__setattr__(s,'ip','127.0.0.2')
    elif damage=='nested':object.__setattr__(s.expected_tcp,'z',.18)
    elif damage=='tuple':object.__setattr__(s,'teleop_ready_q',(9.,*s.teleop_ready_q[1:]))
    elif damage=='type':object.__setattr__(s.control,'loop_hz',str(s.control.loop_hz))
    else:object.__setattr__(s.control,'max_joint_speed_radps',float('nan'))
    with pytest.raises(ValueError,match='binding_changed'):guard.require_binding(s,args['limits'],SHA)


async def test_identical_model_inputs_reuse_only_immutable_fk_math(monkeypatch):
    from app.commissioning import native_jog_model_guard as module
    a,c,_,p,guard=await connected();calls=[];original=module.mdh_tcp_jacobian
    def measured(q,tool):calls.append((tuple(q),tool));return original(q,tool)
    monkeypatch.setattr(module,'mdh_tcp_jacobian',measured)
    try:
        for _ in range(2):guard.validate(p,a._snapshot,settings=a.settings,limits=a._jog_limits,config_sha256=SHA)
        assert len(calls)==1  # Actual/target joint vectors are identical here.
        # Every actual pose is still compared against the cached pure result.
        pose=a._snapshot.actual_tcp;pose.p=(pose.p[0]+.002,*pose.p[1:])
        with pytest.raises(ValueError,match='feedback_mismatch'):
            guard.validate(p,a._snapshot,settings=a.settings,limits=a._jog_limits,config_sha256=SHA)
        assert len(calls)==1
        model_p,model_q=guard._model_pose(tuple(IDLE_Q),TOOL.translation_m,TOOL.rotation_xyzw)
        assert type(model_p) is type(model_q) is tuple
        assert guard._model_pose.cache_info().maxsize==4
    finally:await cleanup(a)


def fixture(**changes):
    t=independent_transform(IDLE_Q,TOOL)
    pose=Pose(p=tuple(t[:3,3]),q=tuple(Rotation.from_matrix(t[:3,:3]).as_quat()))
    lo=tuple(v-.1 for v in pose.p);hi=tuple(v+.1 for v in pose.p)
    base=control_settings(ip='127.0.0.1')
    settings=replace(base,teleop_ready_q=tuple(IDLE_Q),startup_tcp_min_m=lo,startup_tcp_max_m=hi,
        control=replace(base.control,max_joint_speed_radps=.05,max_joint_acceleration_radps2=.2))
    limits=NativeJogLimits(.003,.01,lo,hi,(.001,)*3)
    args=dict(settings=settings,limits=limits,config_sha256=SHA,reference_q=tuple(IDLE_Q),
        tool=TOOL,joint_radius_rad=(.003,)*6,initial_offset_rad=(.0002,)*6,duration_s=.04)
    args.update(changes)
    return args,pose


async def connected():
    args,pose=fixture();guard=ModelJogRegionGuard(**args)
    client=FakeLebaiClient.idle();clock=FakeClock()
    client.kin_data['actual_tcp_pose']=pose_to_lebai(pose)
    client.kin_data['target_tcp_pose']=pose_to_lebai(pose)
    async def speedl(*values):client.write_calls.append(('speedl',*values));return 7
    client.speedl=speedl
    async def factory(host):assert host=='127.0.0.1';return client
    adapter=NativeJogCandidateAdapter(args['settings'],limits=args['limits'],config_sha256=SHA,
        client_factory=factory,clock=clock.now_ns,sleep=clock.sleep,event_callback=None,
        model_region_guard=guard)
    await adapter.connect();assert (await adapter.preflight()).ready
    packet=plan_jog((0,0,1),1,limits=args['limits'],actual_p=pose.p,snapshot_ns=clock.now_ns(),
        now_ns=clock.now_ns(),generation=adapter._pump.generation,sequence=1,
        input_ns=clock.now_ns(),input_expires_ns=clock.now_ns()+100_000_000,
        duration_s=.04,config_sha256=SHA)
    return adapter,client,clock,packet,guard


async def cleanup(adapter):
    try:await adapter.disconnect()
    except BackendCommandError:pass


async def test_guarded_single_submission_keeps_original_stop_and_no_physical_claim():
    a,c,_,p,guard=await connected()
    try:
        assert await a.submit_jog(p,authorize=lambda:None)==7
        # Model eligibility is an entry gate, never a prerequisite for stop.
        a._model_region_guard=None
        a._snapshot.tcp_setting.p=(0.,0.,.2)
        await a.stop(StopReason.GRIP_RELEASED)
        assert a._stop_transaction.confirmed
        assert [w[0] for w in c.write_calls]==['speedl','stop_move']
        assert guard.assessment['model_reference_eligible']
        assert not guard.assessment['continuous_control_admitted']
        assert not guard.assessment['joint_bounds_verified']
        with pytest.raises(BackendCommandError,match='consumed'):await a.submit_jog(p,authorize=lambda:None)
    finally:await cleanup(a)


@pytest.mark.parametrize('damage',['actual_q','target_q','actual_speed','target_acc','tool',
    'feedback','target_tcp','guard_removed','limits','settings'])
async def test_invalid_bound_state_has_no_native_entry_or_cleanup_write(damage):
    a,c,_,p,_=await connected()
    try:
        s=a._snapshot
        if damage=='actual_q':a._snapshot=replace(s,actual_q=(s.actual_q[0]+.001,*s.actual_q[1:]))
        elif damage=='target_q':a._snapshot=replace(s,target_q=(s.target_q[0]+.001,*s.target_q[1:]))
        elif damage=='actual_speed':a._snapshot=replace(s,actual_qd=(1e-6,0.,0.,0.,0.,0.))
        elif damage=='target_acc':a._snapshot=replace(s,target_qdd=(1e-6,0.,0.,0.,0.,0.))
        elif damage=='tool':s.tcp_setting.p=(0.,0.,.17501)
        elif damage=='feedback':s.actual_tcp.p=(s.actual_tcp.p[0]+.002,*s.actual_tcp.p[1:])
        elif damage=='target_tcp':s.target_tcp.p=(s.target_tcp.p[0]+.002,*s.target_tcp.p[1:])
        elif damage=='guard_removed':a._model_region_guard=None
        elif damage=='limits':a._jog_limits=replace(a._jog_limits,max_speed_mps=.004)
        else:a.settings=replace(a.settings,control=replace(a.settings.control,max_joint_speed_radps=.15))
        with pytest.raises((ValueError,BackendCommandError)):await a.submit_jog(p,authorize=lambda:None)
        assert a.native_jog_entries==0 and not c.write_calls
        assert not a._motion_calls.unknown_methods
    finally:await cleanup(a)
    assert not c.write_calls  # Disconnect must not add a stop after local rejection.


@pytest.mark.parametrize('change',['guard','pose','authority'])
async def test_waiting_for_lock_cannot_bypass_model_binding(change):
    a,c,_,p,_=await connected();allowed=True
    def authorize():
        if not allowed:raise BackendCommandError('revoked')
    await a._sdk_lock.acquire();task=asyncio.create_task(a.submit_jog(p,authorize=authorize))
    try:
        await asyncio.sleep(0)
        if change=='guard':a._model_region_guard=None
        elif change=='pose':a._snapshot.actual_tcp.p=(9.,9.,9.)
        else:allowed=False
        a._sdk_lock.release()
        result=(await asyncio.gather(task,return_exceptions=True))[0]
        assert isinstance(result,(ValueError,BackendCommandError))
        assert a.native_jog_entries==0 and not c.write_calls and not a._motion_calls.unknown_methods
    finally:
        if a._sdk_lock.locked():a._sdk_lock.release()
        await cleanup(a)


async def test_slow_model_check_cannot_renew_packet_deadline(monkeypatch):
    a,c,clock,p,_=await connected();old=ModelJogRegionGuard.validate
    def slow(self,*args,**kwargs):
        result=old(self,*args,**kwargs);clock.advance_ms(100);return result
    monkeypatch.setattr(ModelJogRegionGuard,'validate',slow)
    try:
        with pytest.raises((ValueError,BackendCommandError)):await a.submit_jog(p,authorize=lambda:None)
        assert not c.write_calls and a.native_jog_entries==0
    finally:await cleanup(a)


@pytest.mark.parametrize('damage',['sha','soft_limit','tool','reference','model_rates','duration'])
def test_invalid_model_region_refused_before_any_client_factory(damage):
    args,_=fixture()
    if damage=='sha':args['config_sha256']='bad'
    elif damage=='soft_limit':args['settings']=replace(args['settings'],soft_joint_max_rad=(.001,*args['settings'].soft_joint_max_rad[1:]))
    elif damage=='tool':args['tool']=MdhTool((0,0,.18),(0,0,0,1))
    elif damage=='reference':args['reference_q']=(.01,*IDLE_Q[1:])
    elif damage=='model_rates':args['settings']=replace(args['settings'],control=replace(args['settings'].control,max_joint_speed_radps=1e-8))
    else:args['duration_s']=0
    with pytest.raises(ValueError):ModelJogRegionGuard(**args)


@pytest.mark.parametrize('host',['172.16.2.162','localhost','::1'])
def test_guard_does_not_open_nonloopback_configuration(host):
    args,_=fixture();args['settings']=replace(args['settings'],ip=host)
    with pytest.raises(ValueError):ModelJogRegionGuard(**args)


def test_model_guard_freezes_vectors_and_copies_its_assessment():
    args,_=fixture();q=list(IDLE_Q);radius=[.003]*6;args.update(reference_q=q,joint_radius_rad=radius)
    guard=ModelJogRegionGuard(**args);frozen=json.dumps(guard.assessment,sort_keys=True)
    q[0]=9;radius[0]=9;guard.assessment['model_reference_eligible']=False
    assert guard.reference_q==tuple(IDLE_Q) and guard.joint_radius_rad==(.003,)*6
    assert json.dumps(guard.assessment,sort_keys=True)==frozen
    with pytest.raises(FrozenInstanceError):guard.duration_s=.08


async def test_packet_duration_must_match_bound_model_reference():
    a,c,_,p,_=await connected()
    try:
        with pytest.raises((ValueError,BackendCommandError)):
            await a.submit_jog(replace(p,duration_s=.08),authorize=lambda:None)
        assert not c.write_calls and a.native_jog_entries==0
    finally:await cleanup(a)


async def test_model_is_checked_inside_owned_native_task(monkeypatch):
    a,c,_,p,_=await connected();old=a._motion_calls.call
    async def change_before_entry(*args,**kwargs):
        a._snapshot.tcp_setting.p=(0.,0.,.176)
        return await old(*args,**kwargs)
    monkeypatch.setattr(a._motion_calls,'call',change_before_entry)
    try:
        with pytest.raises((ValueError,BackendCommandError)):await a.submit_jog(p,authorize=lambda:None)
        assert a.native_jog_entries==0 and not c.write_calls
        assert not a._motion_calls.unknown_methods
    finally:await cleanup(a)


@pytest.mark.parametrize('kind',['speedl','stop'])
async def test_guarded_failures_keep_original_unknown_and_stop_latch(kind):
    a,c,_,p,guard=await connected();release=asyncio.Event()
    async def stuck(*args):
        c.write_calls.append(('speedl' if kind=='speedl' else 'stop_move',))
        try:await release.wait()
        except asyncio.CancelledError:await release.wait()
        return 7 if kind=='speedl' else None
    try:
        if kind=='speedl':
            c.speedl=stuck
            with pytest.raises(BackendCommandError):await a.submit_jog(p,authorize=lambda:None)
            assert a._motion_calls.unknown_methods=={'speedl'}
            with pytest.raises(BackendCommandError):await a.stop(StopReason.GRIP_RELEASED)
        else:
            await a.submit_jog(p,authorize=lambda:None);c.stop_move=stuck
            with pytest.raises(BackendCommandError):await a.stop(StopReason.GRIP_RELEASED)
        assert a._latched_fault=='stop_unverified'
        assert not a._stop_transaction.confirmed
        release.set();await asyncio.sleep(.02)
        assert a._latched_fault=='stop_unverified'
        assert not (await a.preflight()).ready
        assert not guard.assessment['continuous_control_admitted']
        assert sum(w[0]=='speedl' for w in c.write_calls)==1
        assert sum(w[0]=='stop_move' for w in c.write_calls)==1
    finally:release.set();await cleanup(a)


@pytest.mark.parametrize('change',['settings','limits','sha','guard_type'])
def test_adapter_rejects_mismatched_guard_before_connect(change):
    args,_=fixture();guard=ModelJogRegionGuard(**args)
    settings,limits,sha=args['settings'],args['limits'],SHA
    if change=='settings':settings=replace(settings)
    elif change=='limits':limits=replace(limits)
    elif change=='sha':sha='b'*64
    else:guard=object()
    async def forbidden(_):pytest.fail('factory must never run')
    with pytest.raises(ValueError):NativeJogCandidateAdapter(settings,limits=limits,config_sha256=sha,
        client_factory=forbidden,clock=lambda:0,sleep=asyncio.sleep,event_callback=None,
        model_region_guard=guard)


def test_mutable_nested_settings_change_invalidates_binding():
    args,_=fixture();lo=list(args['settings'].soft_joint_min_rad)
    args['settings']=replace(args['settings'],soft_joint_min_rad=lo)
    guard=ModelJogRegionGuard(**args);lo[0]+=.1
    with pytest.raises(ValueError,match='binding_changed'):
        guard.require_binding(args['settings'],args['limits'],SHA)
