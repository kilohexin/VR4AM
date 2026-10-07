"""Offline guard plus original fault/stop transactions; injected clients only."""
import asyncio

import pytest

from app.control.robot_control import LatestVRFrame
from app.schemas.messages import TeleopMode
from tests.control.elapsed_step_probe import publish
from tests.control.pvat_window_supervision_probe import WindowGuard,WindowSupervisedControl
from tests.control.test_robot_control import FakeBackend,FakeClock,RecordingRecorder
from tests.robots.pvat_cruise_candidate import build_cruise_plan,fit_reference_window
from tests.robots.pvat_ramp_candidate import build_ramp_candidate
from tests.robots.pvat_interpolation_probe import quintic_segment
from tests.robots.pvat_phase_encoding_probe import encode_phases,retarget_window
from tests.robots.test_lebai_adapter_control import _connected_control_adapter,_wait_until,_target

ZERO=(0.,)*6
axis=lambda x:(x,*ZERO[1:])


def window():
    return build_ramp_candidate(ZERO,ZERO,ZERO,axis(.1)).segment


@pytest.mark.parametrize('elapsed',[0,20,79])
def test_guard_accepts_declared_reference_and_observation(elapsed):
    guard=WindowGuard();guard.commit(window(),1_000_000_000)
    guard.observe(*window().state(elapsed/1000))
    assert guard.check(1_000_000_000+elapsed*1_000_000) is None


@pytest.mark.parametrize('elapsed',[80,110])
def test_guard_expires_at_horizon_without_extrapolation(elapsed):
    guard=WindowGuard();guard.commit(window(),0);guard.observe(ZERO,ZERO,ZERO)
    assert guard.check(elapsed*1_000_000)=='pvat_window_expired'
    assert guard.fault=='pvat_window_expired'
    with pytest.raises(ValueError,match='guard_faulted'):guard.commit(window(),elapsed*1_000_000)


@pytest.mark.parametrize('field,reason',[
    (0,'pvat_reference_position_mismatch'),(1,'pvat_reference_velocity_mismatch'),(2,'pvat_reference_acceleration_mismatch')])
def test_guard_rejects_reference_mismatch_without_changing_observed_values(field,reason):
    guard=WindowGuard();guard.commit(window(),0)
    state=list(window().state(.02));state[field]=axis(1.)
    guard.observe(*state)
    assert guard.check(20_000_000)==reason


def test_missing_observation_is_not_invented_as_matching_reference():
    guard=WindowGuard();guard.commit(window(),0)
    assert guard.check(0)=='pvat_reference_missing'


async def active(backend=None,clock=None):
    backend=backend or FakeBackend();clock=clock or FakeClock()
    guard=WindowGuard();latest=LatestVRFrame();recorder=RecordingRecorder()
    control=WindowSupervisedControl(window_guard=guard,backend=backend,latest=latest,clock=clock,recorder=recorder)
    await control.connect();publish(latest,clock,1,False,0,0);await control.tick();await control.arm()
    publish(latest,clock,2,True,0,0);await control.tick()
    assert control.mode is TeleopMode.ACTIVE
    return control,guard,latest,backend,clock,recorder


async def test_expiry_faults_even_without_a_new_vr_frame_and_never_dispatches_a_target():
    control,guard,latest,backend,clock,recorder=await active()
    guard.commit(window(),clock.now_ns());guard.observe(ZERO,ZERO,ZERO)
    targets=list(backend.targets);clock.advance_ms(80)
    await control.tick()
    assert control._fault=='pvat_window_expired'
    assert backend.targets==targets and len(backend.stops)==1
    assert any(e['kind']=='stop_confirmed' for e in recorder.events)
    with pytest.raises(RuntimeError,match='arm_blocked_by_fault'):await control.arm()
    assert not (await control.reset_fault()).accepted


@pytest.mark.parametrize('trigger',['expiry','position','velocity','acceleration','missing','interpolation',
                                    'short_phase','retarget_expired','retarget_infeasible'])
@pytest.mark.parametrize('outcome',['confirmed','incomplete','error','late','cancelled'])
async def test_guard_fault_uses_original_adapter_stop_and_preserves_failure_latches(trigger,outcome):
    adapter,client,clock=await _connected_control_adapter()
    task=None;release=asyncio.Event();entered=asyncio.Event();events=[]
    async def record(event,timestamp):events.append(event)
    async def stop_move():
        client.write_calls.append(('stop_move',));entered.set()
        if outcome=='error':raise RuntimeError('injected_stop_error')
        if outcome=='late':clock.advance_ms(610)
        if outcome=='cancelled':await release.wait()
    client.stop_move=stop_move;adapter._event_callback=record
    try:
        control,guard,latest,_,_,recorder=await active(adapter,clock)
        guard.commit(window(),clock.now_ns())
        rejection_reasons={'interpolation':'pvat_window_interpolation_limit',
                           'short_phase':'pvat_phase_too_short',
                           'retarget_expired':'pvat_retarget_window_expired',
                           'retarget_infeasible':'no_feasible_settling_plan'}
        if trigger in rejection_reasons:
            with pytest.raises(ValueError,match=rejection_reasons[trigger]) as rejected:
                if trigger=='interpolation':
                    fit_reference_window(build_cruise_plan(ZERO,axis(.5)),.35,.08)
                elif trigger=='short_phase':
                    encode_phases(build_cruise_plan(ZERO,axis(.00001)).segments)
                else:
                    segment=quintic_segment(ZERO,axis(.01),axis(.1),axis(.00112),axis(.018),axis(.1),.08)
                    packet=encode_phases((segment,))[0]
                    retarget_window(packet,.08 if trigger=='retarget_expired' else 0.,axis(.1))
            guard.reject(str(rejected.value))
        elif trigger!='missing':
            state=[ZERO,ZERO,ZERO]
            if trigger!='expiry':state[{'position':0,'velocity':1,'acceleration':2}[trigger]]=axis(1.)
            guard.observe(*state)
        if outcome=='incomplete':client.kin_data['actual_joint_speed']=[.1]*6
        if trigger=='expiry':clock.advance_ms(80)
        before=list(client.write_calls)
        task=asyncio.create_task(control.tick())
        await asyncio.wait_for(entered.wait(),1)
        if outcome=='cancelled':
            task.cancel()
            with pytest.raises(asyncio.CancelledError):await task
            release.set()
        else:await asyncio.wait_for(task,2)
        expected_fault=rejection_reasons.get(trigger, 'pvat_window_expired' if trigger=='expiry' else
                                            ('pvat_reference_missing' if trigger=='missing' else f'pvat_reference_{trigger}_mismatch'))
        expected=expected_fault if outcome=='confirmed' else ('stop_incomplete' if outcome=='incomplete' else 'stop_unverified')
        assert (await control.state_message()).fault==expected
        client.kin_data['actual_joint_speed']=[0.]*6
        for seq in (3,4):
            publish(latest,clock,seq,True,.1,0);await control.tick()
            assert (await control.state_message()).fault==expected
        assert not (await control.reset_fault()).accepted
        with pytest.raises(RuntimeError,match='arm_blocked_by_fault'):await control.arm()
        assert client.write_calls==[*before,('stop_move',)]
        assert not adapter._pump.has_pending
        kinds=[e['kind'] for e in recorder.events]
        assert ('stop_confirmed' in kinds)==(outcome=='confirmed')
        if outcome=='late':
            lifecycle=next(e for e in events if e['kind']=='stop_rpc_lifecycle')
            assert lifecycle['deadline_ns']-lifecycle['started_ns']==200_000_000
            assert lifecycle['sdk_await_completed_ns']-lifecycle['sdk_await_started_ns']==610_000_000
    finally:
        release.set();client.release_ik.set()
        if task is not None and not task.done():task.cancel();await asyncio.gather(task,return_exceptions=True)
        await adapter.disconnect()
    assert client.network_calls==client.discovery_calls==0


def test_renewal_cannot_erase_a_gap_even_if_no_tick_checked_it_yet():
    guard=WindowGuard();guard.commit(window(),0)
    with pytest.raises(ValueError,match='guard_faulted'):guard.commit(window(),110_000_000)
    assert guard.fault=='pvat_window_expired'


def test_renewal_must_preserve_declared_pva_and_clear_old_observations():
    plan=build_cruise_plan(ZERO,axis(.5));guard=WindowGuard()
    guard.commit(plan.window(0.,.08),0);guard.observe(ZERO,ZERO,ZERO)
    guard.commit(plan.window(.02,.08),20_000_000)
    assert guard.check(20_000_000)=='pvat_reference_missing'


def test_discontinuous_renewal_is_rejected():
    guard=WindowGuard();guard.commit(window(),0)
    with pytest.raises(ValueError,match='guard_faulted'):guard.commit(window(),20_000_000)
    assert guard.fault=='pvat_reference_discontinuous'


def test_invalid_observation_latches_instead_of_leaving_the_old_valid_observation():
    guard=WindowGuard();guard.commit(window(),0);guard.observe(ZERO,ZERO,ZERO)
    with pytest.raises(ValueError,match='invalid_observation'):guard.observe(ZERO,ZERO,axis(float('nan')))
    assert guard.check(0)=='pvat_reference_invalid'


async def test_expired_window_invalidates_blocked_ik_and_pending_requests_before_send():
    adapter,client,clock=await _connected_control_adapter();task=None
    try:
        control,guard,_,_,_,_=await active(adapter,clock)
        guard.commit(window(),clock.now_ns());guard.observe(ZERO,ZERO,ZERO)
        client.block_ik=True
        await adapter.command_tcp(_target(),17)
        await asyncio.wait_for(client.ik_started.wait(),1)
        await adapter.command_tcp(_target(),18)
        assert adapter._pump.has_pending
        before=list(client.write_calls);clock.advance_ms(80)
        task=asyncio.create_task(control.tick())
        await _wait_until(lambda:adapter._stop_transaction is not None)
        client.release_ik.set();await asyncio.wait_for(task,2)
        assert client.write_calls==[*before,('stop_move',)]
        assert not adapter._pump.has_pending
        assert (await control.state_message()).fault=='pvat_window_expired'
    finally:
        client.release_ik.set()
        if task is not None and not task.done():task.cancel();await asyncio.gather(task,return_exceptions=True)
        await adapter.disconnect()
    assert client.network_calls==client.discovery_calls==0


async def test_normal_grip_release_revokes_reference_without_fault_or_a_second_stop():
    control,guard,latest,backend,clock,_=await active()
    guard.commit(window(),clock.now_ns());clock.advance_ms(20)
    guard.observe(*window().state(.02));publish(latest,clock,3,False,0,0)
    await control.tick()
    assert control.mode is TeleopMode.HOLD
    assert guard.window is None and guard.fault is None
    # Keep input fresh: original STALE handling legitimately stops old frames.
    clock.advance_ms(100);publish(latest,clock,4,False,0,0);await control.tick()
    assert len(backend.stops)==1 and control._fault is None


def test_phase_partition_at_endpoint_cannot_bypass_window_expiry():
    packets=encode_phases(build_cruise_plan(ZERO,axis(.5)).segments)
    guard=WindowGuard();guard.commit(packets[0].segment,0)
    guard.observe(*packets[0].state(.074))
    assert guard.check(74_000_000) is None
    with pytest.raises(ValueError,match='guard_faulted'):
        guard.commit(packets[1].segment,75_000_000)
    assert guard.fault=='pvat_window_expired'


def test_phase_partition_sent_early_cannot_pretend_its_future_pva_is_current():
    packets=encode_phases(build_cruise_plan(ZERO,axis(.5)).segments)
    guard=WindowGuard();guard.commit(packets[0].segment,0)
    with pytest.raises(ValueError,match='guard_faulted'):
        guard.commit(packets[1].segment,20_000_000)
    assert guard.fault=='pvat_reference_discontinuous'


def test_moving_retarget_can_renew_before_expiry_with_matching_pva_and_new_observation():
    original=encode_phases((quintic_segment(ZERO,axis(.02),ZERO,axis(.0016),axis(.02),ZERO,.08),))[0]
    guard=WindowGuard();guard.commit(original.segment,0);guard.observe(*original.state(0.))
    next_window=retarget_window(original,.04,axis(-.001))[0]
    guard.commit(next_window.segment,40_000_000)
    assert guard.observation is None
    guard.observe(*next_window.state(0.))
    assert guard.check(40_000_000) is None
    assert guard.fault is None
