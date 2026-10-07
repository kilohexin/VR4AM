"""Runtime request/feedback boundary, with injected completion handles only."""
from dataclasses import replace

import pytest

from app.control.catch_up import CatchUpSupervisor
from app.control.transactional_settings import PursuitSettings
from app.robots.command_admission import CommandAdmission,CommandCompletion,CommandRef
from app.robots.motion_view import JointObservation

ZERO=(0.,)*6
SETTINGS=PursuitSettings(500,40,40,.001,.002,3)


class Harness:
    def __init__(self):
        self.now=1_000_000_000
        self.authorized=self.current=True
        self.accepted=CommandRef(1,3,1)
        self.writes=[]
        self.probe=CatchUpSupervisor(self,self.accepted,ZERO,clock=lambda:self.now,
            authorized=lambda:self.authorized,generation_current=lambda g:self.current,settings=SETTINGS)

    async def submit_catch_up(self,accepted,command_id):
        self.writes.append(command_id)
        ref=CommandRef(command_id,3,len(self.writes)+1)
        completion=CommandCompletion(ref)
        return CommandAdmission(ref,None,completion=completion)

    def sample(self,*,close=True,captured=None,completed=None,purpose='state'):
        captured=self.now if captured is None else captured
        return JointObservation(3,captured,captured if completed is None else completed,purpose,
                                ZERO if close else (.01,*ZERO[1:]),ZERO,ZERO)

    async def returned(self,command_id=2):
        receipt=await self.probe.submit(command_id)
        receipt.completion._resolve('sdk_returned',sdk_outcome='returned',accepted_command=self.accepted,pvat_mode='catch_up')
        assert self.probe.settle()=='pursuing'
        return receipt


async def test_sdk_return_requires_new_feedback_before_another_dispatch():
    h=Harness();await h.returned()
    assert h.probe.phase=='awaiting_feedback'
    assert not h.probe.ready_to_submit
    with pytest.raises(RuntimeError,match='feedback_pending'):await h.probe.submit(3)
    assert h.writes==[2]


async def test_snapshot_started_before_completion_collection_cannot_settle():
    h=Harness();receipt=await h.probe.submit(2)
    h.now+=10_000_000;old=h.sample();h.now+=10_000_000
    receipt.completion._resolve('sdk_returned',sdk_outcome='returned',accepted_command=h.accepted,pvat_mode='catch_up')
    h.probe.settle()
    assert h.probe.observe(old)=='pursuing'
    assert h.probe.good_samples==0 and not h.probe.ready_to_submit


async def test_new_far_feedback_allows_one_dispatch_and_resets_observation_streak():
    h=Harness();await h.returned();h.now+=20_000_000
    h.probe.observe(h.sample(close=False))
    assert h.probe.ready_to_submit and h.probe.phase=='dispatchable'
    await h.probe.submit(3)
    assert h.writes==[2,3] and h.probe.phase=='inflight' and h.probe.good_samples==0
    assert not h.probe.ready_to_submit


async def test_close_feedback_waits_without_more_pvat_and_requests_exit_after_three_samples():
    h=Harness();await h.returned()
    for expected in (1,2,3):
        h.now+=20_000_000
        state=h.probe.observe(h.sample())
        assert state==('observed_settled' if expected==3 else 'pursuing')
        assert h.probe.good_samples==expected and not h.probe.ready_to_submit
    assert h.writes==[2]


async def test_no_new_feedback_expires_without_changing_total_pursuit_budget():
    h=Harness();await h.returned();started=h.probe.started_ns
    h.now+=40_000_001
    assert h.probe._check()=='feedback_expired'
    assert h.probe.started_ns==started
    with pytest.raises(RuntimeError,match='feedback_expired'):await h.probe.submit(3)
    assert h.writes==[2]


async def test_feedback_at_declared_gap_boundary_can_be_processed_before_timeout():
    h=Harness();await h.returned();h.now+=40_000_000
    assert h.probe.observe(h.sample(close=False))=='pursuing'
    assert h.probe.ready_to_submit


@pytest.mark.parametrize('change',[
    {'read_completed_ns':1_000_000_000}, {'read_completed_ns':1_030_000_000},
    {'read_purpose':'connection'}, {'read_purpose':'stop_verification'},
    {'generation':2}, {'captured_ns':True},
])
async def test_invalid_or_other_purpose_samples_do_not_release_feedback_wait(change):
    h=Harness();await h.returned();h.now+=20_000_000
    assert h.probe.observe(replace(h.sample(),**change))=='pursuing'
    assert h.probe.good_samples==0 and not h.probe.ready_to_submit


async def test_duplicate_cache_cannot_extend_feedback_deadline():
    h=Harness();await h.returned();h.now+=20_000_000;sample=h.sample()
    h.probe.observe(sample);deadline=h.probe.feedback_deadline_ns
    h.now+=20_000_000;h.probe.observe(sample)
    assert h.probe.feedback_deadline_ns==deadline and h.probe.good_samples==0
    h.now=deadline+1
    assert h.probe._check()=='feedback_expired'


@pytest.mark.parametrize('reason',['authority','generation'])
async def test_revoke_preempts_wait_and_late_feedback_does_not_reactivate(reason):
    h=Harness();await h.returned();h.now+=20_000_000
    if reason=='authority':h.authorized=False
    else:h.current=False
    assert h.probe.observe(h.sample())=='authority_revoked'
    h.authorized=h.current=True
    assert h.probe._check()=='authority_revoked'
    assert not h.probe.ready_to_submit


async def test_repeated_reads_of_same_completion_do_not_reset_feedback_fence():
    h=Harness();await h.returned();fence=h.probe.feedback_after_ns;deadline=h.probe.feedback_deadline_ns
    h.now+=10_000_000
    assert h.probe.settle()=='pursuing'
    assert h.probe.feedback_after_ns==fence and h.probe.feedback_deadline_ns==deadline


@pytest.mark.parametrize('field,value',[('actual_q',(float('nan'),)*6),('actual_qd',(0.,)*5),
                                       ('actual_q',None)])
async def test_invalid_vectors_do_not_unlock_dispatch_or_extend_wait(field,value):
    h=Harness();await h.returned();deadline=h.probe.feedback_deadline_ns;h.now+=20_000_000
    assert h.probe.observe(replace(h.sample(close=False),**{field:value}))=='pursuing'
    assert not h.probe.ready_to_submit and h.probe.feedback_deadline_ns==deadline


async def test_far_feedback_cannot_be_used_after_waiting_through_its_age_bound():
    h=Harness();await h.returned();h.now+=20_000_000
    h.probe.observe(h.sample(close=False));assert h.probe.ready_to_submit
    h.now+=40_000_001
    assert not h.probe.ready_to_submit and h.probe.state=='feedback_expired'
    with pytest.raises(RuntimeError,match='feedback_expired'):await h.probe.submit(3)
    assert h.writes==[2]


@pytest.mark.parametrize('boundary',['feedback_gap','grip_release','disarm'])
@pytest.mark.parametrize('outcome',['confirmed','incomplete','error','late','cancelled'])
async def test_runtime_wait_exit_uses_original_stop_and_preserves_latches(boundary,outcome):
    import asyncio
    from app.schemas.messages import TeleopMode
    from tests.control.test_transactional_vr_tick import rig
    async with rig() as r:
        await r.warm(pursuit=True);await r.dispatch()
        assert r.control.pursuit.phase=='awaiting_feedback'
        snapshot=r.control.coordinator.snapshot();before=list(r.run.client.write_calls)
        entered,release=asyncio.Event(),asyncio.Event();task=None
        async def stop_move():
            r.run.client.write_calls.append(('stop_move',));entered.set()
            if outcome=='error':raise RuntimeError('injected_stop_error')
            if outcome=='late':r.clock.advance_ms(610)
            if outcome=='cancelled':await release.wait()
        r.run.client.stop_move=stop_move
        if outcome=='incomplete':r.run.client.kin_data['actual_joint_speed']=[.1]*6
        try:
            if boundary=='disarm':operation=r.control.disarm()
            else:
                r.clock.advance_ms(41 if boundary=='feedback_gap' else 20)
                r.publish(grip=boundary!='grip_release')
                operation=r.control.tick()
            task=asyncio.create_task(operation)
            await asyncio.wait_for(entered.wait(),1)
            if outcome=='cancelled':
                task.cancel()
                with pytest.raises(asyncio.CancelledError):await task
                release.set()
            elif boundary=='disarm' and outcome!='confirmed':
                from app.robots.base import BackendCommandError
                reason={'incomplete':'stop_incomplete','error':'sdk_call_failed:stop_move','late':'sdk_timeout:stop_move'}[outcome]
                with pytest.raises(BackendCommandError,match=reason):await asyncio.wait_for(task,2)
            else:await asyncio.wait_for(task,2)
            assert r.run.client.write_calls==[*before,('stop_move',)]
            assert r.control.coordinator.snapshot()==snapshot
            assert not r.run.adapter._pump.has_pending
            assert r.control.pursuit.state=='authority_revoked'
            if outcome=='confirmed':
                assert (await r.control.state_message()).fault is None
                assert r.control.mode in {TeleopMode.DISARMED,TeleopMode.HOLD}
            else:
                fault='stop_incomplete' if outcome=='incomplete' else 'stop_unverified'
                assert (await r.control.state_message()).fault==fault
                r.run.client.kin_data['actual_joint_speed']=[0.]*6
                r.publish();await r.control.tick()
                assert (await r.control.state_message()).fault==fault
                assert not (await r.control.reset_fault()).accepted
                with pytest.raises(RuntimeError,match='arm_blocked_by_fault'):await r.control.arm()
                assert r.run.client.write_calls==[*before,('stop_move',)]
            if outcome=='late':
                event=next(e for e in r.run.events if e['kind']=='stop_rpc_lifecycle')
                assert event['deadline_ns']-event['started_ns']==200_000_000
                assert event['sdk_await_completed_ns']-event['sdk_await_started_ns']==610_000_000
        finally:
            release.set()
            if task is not None and not task.done():task.cancel();await asyncio.gather(task,return_exceptions=True)


async def test_feedback_timeout_is_checked_even_without_a_new_vr_frame():
    from app.schemas.messages import TeleopMode
    from tests.control.test_transactional_vr_tick import rig
    async with rig() as r:
        await r.warm(pursuit=True);await r.dispatch()
        count=len(r.control.requests);r.clock.advance_ms(41)
        await r.control.tick()
        assert r.control.exits==['feedback_expired'] and r.control.mode is TeleopMode.DISARMED
        assert len(r.control.requests)==count


async def test_gripper_await_can_expire_prepared_catchup_in_same_tick():
    import asyncio
    from app.schemas.messages import TeleopMode
    from tests.control.test_transactional_vr_tick import rig
    async with rig() as r:
        await r.warm(pursuit=True);await r.dispatch()
        count=len(r.control.requests);entered,release=asyncio.Event(),asyncio.Event()
        original=r.run.client.set_claw
        async def blocked(*args):
            await original(*args);entered.set();await release.wait()
        r.run.client.set_claw=blocked
        task=asyncio.create_task(r.tick(elapsed=20,trigger=.8))
        try:
            await asyncio.wait_for(entered.wait(),1)
            assert r.control.pursuit.phase=='dispatchable'
            r.clock.advance_ms(41);release.set();await asyncio.wait_for(task,2)
            assert r.control.exits==['feedback_expired'] and r.control.mode is TeleopMode.DISARMED
            assert len(r.control.requests)==count
        finally:
            release.set();await asyncio.gather(task,return_exceptions=True)


@pytest.mark.parametrize('gate',['accepted_missing','accepted_changed','disconnected','preflight',
                               'stopping','pump','latched'])
async def test_runtime_wait_checks_acceptance_and_backend_gates_before_counting(gate,monkeypatch):
    from tests.control.test_transactional_vr_tick import rig
    async with rig() as r:
        await r.warm(pursuit=True);await r.dispatch()
        r.clock.advance_ms(20);r.publish()
        view=r.run.adapter.read_motion_view()
        changes={'accepted_missing':{'accepted':None},
            'accepted_changed':{'accepted':replace(view.accepted,command=replace(view.accepted.command,submission_id=999))},
            'disconnected':{'client_attached':False},'preflight':{'preflight_ready':False},
            'stopping':{'stop_in_progress':True},'pump':{'pump_running':False},
            'latched':{'latched_fault':'stop_unverified'}}
        monkeypatch.setattr(r.run.adapter,'read_motion_view',lambda:replace(view,**changes[gate]))
        update=r.control.collect(r.control._target_context())
        assert update.fault==('stop_unverified' if gate=='latched' else 'candidate_dispatch_failed')
        assert not update.stop and r.control.pursuit.good_samples==0
