"""Segmented delivery candidate: explicit fake SDK, never robot/network access."""
import asyncio
from dataclasses import asdict, replace
from unittest.mock import AsyncMock

import pytest

from app.control.target_strategy import TargetContext
from app.control.transactional_settings import PursuitSettings
from app.robots.base import BackendCommandError
from app.schemas.messages import Pose
from tests.commissioning.test_finite_diagnostic import AuditClient
from tests.robots.real_settings import control_settings
from tests.robots.test_lebai_adapter_control import FakeClock, _target
from tests.robots.test_vr_profile_experiments import limiter


OPTIONS = PursuitSettings(500, 40, 40, .00001, .00001, 3)


async def rig(settling=OPTIONS, horizon=.08, allow_unsent_expiry_hold=False, parallel=False):
    from app.robots.segmented_adapter import SegmentedLebaiAdapter
    from app.control.segmented_strategy import SegmentedTargetStrategy
    clock, client = FakeClock(), AuditClient()
    settings = control_settings()
    settings = replace(settings, control=replace(settings.control,
        pvat_horizon_s=horizon, max_joint_speed_radps=.05, max_joint_acceleration_radps2=.2))
    adapter = SegmentedLebaiAdapter(settings, settling=settling,
        client_factory=AsyncMock(return_value=client), clock=clock.now_ns,
        pvat_clock=clock.now_ns, sleep=clock.sleep,
        allow_unsent_expiry_hold=allow_unsent_expiry_hold,
        parallel_fk=parallel,
        backend_label='LEBAI_FAKE' if allow_unsent_expiry_hold else 'LEBAI')
    await adapter.connect()
    assert (await adapter.preflight()).ready
    box = [TargetContext(1, clock.now_ns(), True, True)]
    strategy = SegmentedTargetStrategy(adapter, limiter(), context=lambda: box[0])
    strategy.on_anchor(_target(.3), box[0])
    return adapter, client, clock, box, strategy


async def send(strategy, adapter, target, command_id):
    plan = strategy.prepare(_target(.3), target, .02, limiter=limiter())
    assert plan.kind == 'candidate'
    assert await strategy.dispatch(plan, command_id, backend=adapter) is None
    return await asyncio.wait_for(strategy.receipt.completion.wait(), 2)


async def test_fake_returned_fk_revocation_requires_original_stop_then_a_new_audit():
    from app.robots.base import StopReason
    from tests.robots.test_lebai_adapter_control import _wait_until
    adapter,client,clock,box,strategy=await rig(allow_unsent_expiry_hold=True,parallel=True)
    allowed=True
    adapter.bind_motion_authority(lambda:allowed)
    async def release_after_reply(count,pose):
        nonlocal allowed
        if count==4:allowed=False
        return pose
    client.fk_hook=release_after_reply
    try:
        result=await send(strategy,adapter,_target(.31),1)
        await _wait_until(lambda:adapter._finite_context is None)
        assert result.sdk_outcome=='not_started' and result.invalidated
        assert not client.write_calls and not adapter._finite_fk_failed
        strategy.on_revoke(box[0].generation)
        await adapter.stop(StopReason.GRIP_RELEASED)
        assert adapter._stop_transaction.confirmed
        client.fk_hook=None;allowed=True
        box[0]=replace(box[0],generation=box[0].generation+1,now_ns=clock.now_ns())
        assert (await adapter.preflight()).ready
        strategy.on_anchor(_target(.3),box[0])
        result=await send(strategy,adapter,_target(.31),2)
        assert result.sdk_outcome=='returned'
        assert len(client.fk_calls)==8  # Fresh four-FK audit; no revoked-result reuse.
        assert sum(call[0]=='move_pvat' for call in client.write_calls)==1
    finally:await adapter.disconnect()


async def observe(adapter, client, clock, box, strategy, *, close=True):
    clock.value += 20_000_000
    box[0] = replace(box[0], now_ns=clock.now_ns())
    if close:
        accepted = adapter.read_motion_view().accepted
        client.kin_data['actual_joint_pose'] = list(accepted.solution_q)
        client.kin_data['actual_tcp_pose']['x'] = accepted.position[0]
    await adapter.get_state()
    return strategy.collect(box[0])


async def test_one_segment_zero_terminal_waits_then_uses_latest_input():
    adapter, client, clock, box, strategy = await rig()
    try:
        done = await send(strategy, adapter, _target(.31), 1)
        assert done.pvat_mode == 'advance'
        first = strategy.collect(box[0])
        assert first.committed is not None and first.fault is None
        assert len(client.write_calls) == 1
        assert client.write_calls[0][2:5] == ([0.]*6, [0.]*6, .08)
        for x in [.32, .29, .28]:
            plan = strategy.prepare(first.committed, _target(x), .02, limiter=limiter())
            assert plan.kind == 'wait'
            await strategy.dispatch(plan, 2, backend=adapter)
        assert len(client.write_calls) == 1
        # SDK return alone, elapsed horizon, or repeated cache is insufficient.
        clock.value += 80_000_000
        box[0] = replace(box[0], now_ns=clock.now_ns())
        for _ in range(3):
            assert strategy.collect(box[0]).committed is None
        assert not adapter.read_segment_status().ready
        for _ in range(3):
            await observe(adapter, client, clock, box, strategy)
        assert adapter.read_segment_status().ready
        second = strategy.prepare(first.committed, _target(.28), .02, limiter=limiter())
        assert second.kind == 'candidate' and second.pose.p[0] < first.committed.p[0]
        await strategy.dispatch(second, 3, backend=adapter)
        await asyncio.wait_for(strategy.receipt.completion.wait(), 2)
        assert len(client.write_calls) == 2 and len(client.ik_calls) == 2
        assert all(c[2:5] == ([0.]*6, [0.]*6, .08) for c in client.write_calls)
        assert len(client.fk_calls) == 8
    finally:
        await adapter.disconnect()


async def test_busy_adapter_rejects_direct_second_admission_without_overwrite():
    adapter, client, _clock, box, strategy = await rig()
    try:
        await send(strategy, adapter, _target(.31), 1)
        strategy.collect(box[0])
        with pytest.raises(BackendCommandError, match='segment_feedback_pending'):
            await adapter.submit_segment(_target(.3001), 2)
        assert len(client.write_calls) == 1
        with pytest.raises(BackendCommandError, match='segmented_interface_required'):
            await adapter.submit_tcp(_target(.3001), 3)
    finally:
        await adapter.disconnect()


@pytest.mark.parametrize('change', ['release', 'generation', 'latched', 'accepted', 'feedback_timeout'])
async def test_withdrawal_or_bad_feedback_cannot_emit_another_segment(change):
    adapter, client, clock, box, strategy = await rig()
    try:
        await send(strategy, adapter, _target(.31), 1)
        strategy.collect(box[0])
        if change == 'release':
            box[0] = replace(box[0], authorized=False)
            strategy.on_revoke(2)
        elif change == 'generation':
            adapter._pump.invalidate()
        elif change == 'latched':
            adapter._latched_fault = 'stop_unverified'
        elif change == 'accepted':
            adapter._accepted_target_ref = replace(adapter._accepted_target_ref, submission_id=999)
        else:
            clock.value += 500_000_000
            box[0] = replace(box[0], now_ns=clock.now_ns())
        update = strategy.collect(box[0])
        if change != 'release':
            assert update.fault is not None
        plan = strategy.prepare(_target(.3), _target(.32), .02, limiter=limiter())
        await strategy.dispatch(plan, 2, backend=adapter)
        assert len(client.write_calls) == 1
    finally:
        await adapter.disconnect()


async def test_rotation_input_does_not_rotate_first_delivery_candidate():
    adapter, client, _clock, box, strategy = await rig()
    try:
        request = Pose(p=(.31, 0, .4), q=(0, 0, .5, .8660254037844386))
        await send(strategy, adapter, request, 1)
        assert strategy.collect(box[0]).committed.q == _target(.3).q
        assert client.ik_calls[0][0]['rz'] == 0
    finally:
        await adapter.disconnect()


async def test_old_prepared_segment_cannot_be_dispatched_after_new_input():
    adapter, client, _clock, _box, strategy = await rig()
    try:
        old = strategy.prepare(_target(.3), _target(.31), .02, limiter=limiter())
        strategy.prepare(_target(.3), _target(.29), .02, limiter=limiter())
        with pytest.raises(BackendCommandError, match='segment_plan_obsolete'):
            await strategy.dispatch(old, 1, backend=adapter)
        assert client.write_calls == []
    finally:
        await adapter.disconnect()


async def test_real_app_selects_segment_mode_only_with_explicit_fake_factory(tmp_path, monkeypatch):
    from app.main import create_app
    from app.control.segmented_strategy import SegmentedTargetStrategy
    from app.robots.segmented_adapter import SegmentedLebaiAdapter
    from app.recording.noop import NoopRecorder
    from tests.control.test_transactional_runtime import config
    settings = config(tmp_path, monkeypatch, policy={
        'kind': 'segmented_experimental', 'settling': asdict(OPTIONS)})
    factory = AsyncMock(return_value=AuditClient())
    monkeypatch.setattr('app.main.build_recorder', lambda *a: NoopRecorder())
    app = create_app(settings=settings, client_factory=factory, backend_label='LEBAI_FAKE')
    async with app.router.lifespan_context(app):
        assert type(app.state.backend) is SegmentedLebaiAdapter
        assert type(app.state.control.target_strategy) is SegmentedTargetStrategy
        assert app.state.control.target_lifecycle is app.state.control.target_strategy
        assert not app.state.hardware_verified
        assert app.state.backend._client.write_calls == []
    factory.assert_awaited_once()


async def test_slow_fk_sends_no_segment_and_preserves_original_stop():
    adapter, client, _clock, box, strategy = await rig()
    async def slow(index, result):
        await asyncio.sleep(.2)
        return result
    client.fk_hook = slow
    try:
        done = await send(strategy, adapter, _target(.31), 1)
        assert done.sdk_outcome == 'not_started'
        assert strategy.collect(box[0]).fault is not None
        assert [c[0] for c in client.write_calls] == ['stop_move']
    finally:
        await adapter.disconnect()


async def test_prepare_readiness_fault_is_reported_by_collect_to_owner():
    adapter, client, _clock, box, strategy = await rig()
    try:
        adapter._latched_fault = 'stop_unverified'
        plan = strategy.prepare(_target(.3), _target(.31), .02, limiter=limiter())
        assert plan.kind == 'wait'
        assert strategy.collect(box[0]).fault == 'stop_unverified'
        assert client.write_calls == []
    finally:
        await adapter.disconnect()


async def test_previous_acceptance_must_be_collected_before_next_plan():
    adapter, _client, clock, box, strategy = await rig()
    try:
        await send(strategy, adapter, _target(.31), 1)
        adapter.read_segment_status()
        clock.value += 80_000_000
        # Accumulate feedback without delivering acceptance to the policy.
        for _ in range(3):
            clock.value += 20_000_000
            accepted = adapter.read_motion_view().accepted
            adapter._client.kin_data['actual_joint_pose'] = list(accepted.solution_q)
            adapter._client.kin_data['actual_tcp_pose']['x'] = accepted.position[0]
            await adapter.get_state()
            adapter.read_segment_status()
        box[0] = replace(box[0], now_ns=clock.now_ns())
        assert adapter.read_segment_status().ready
        assert strategy.prepare(_target(.3), _target(.29), .02, limiter=limiter()).kind == 'wait'
        assert strategy.collect(box[0]).committed == accepted.tcp
    finally:
        await adapter.disconnect()


async def test_ready_status_requires_current_valid_acceleration_feedback():
    adapter, client, clock, box, strategy = await rig()
    try:
        await send(strategy, adapter, _target(.31), 1)
        strategy.collect(box[0])
        clock.value += 80_000_000
        for _ in range(3):
            await observe(adapter, client, clock, box, strategy)
        assert adapter.read_segment_status().ready
        # Simulate an immutable cache whose acceleration evidence is absent.
        # A real read error retains the original adapter's harder fault gate.
        adapter._motion_observation = replace(adapter._motion_observation, actual_qdd=None)
        assert not adapter.read_segment_status().ready
        with pytest.raises(BackendCommandError, match='segment_feedback_pending'):
            await adapter.submit_segment(_target(.3001), 2)
    finally:
        await adapter.disconnect()


async def test_partial_finite_endpoint_cannot_commit_full_requested_tcp():
    adapter, client, _clock, box, strategy = await rig()
    adapter._pvat_limits = replace(adapter._pvat_limits, max_joint_acceleration_radps2=.00001)
    try:
        done = await send(strategy, adapter, _target(.31), 1)
        assert done.sdk_outcome == 'not_started'
        assert done.error == 'segment_full_endpoint_infeasible'
        assert strategy.collect(box[0]).fault is not None
        assert [c[0] for c in client.write_calls] == ['stop_move']
        assert adapter.read_motion_view().accepted is None
    finally:
        await adapter.disconnect()


@pytest.mark.parametrize('params', [dict(settle_samples=2), dict(position_tolerance_rad=.1),
    dict(velocity_tolerance_radps=.1), dict(budget_ms=5000), dict(max_observation_age_ms=200)])
def test_segment_configuration_rejects_weak_observation_gates(params):
    from app.control.transactional_settings import SegmentedSettings
    with pytest.raises(ValueError, match='invalid_segment_settling'):
        SegmentedSettings(replace(OPTIONS, **params))


async def test_regressing_sample_cannot_be_reused_to_complete_streak():
    adapter, client, clock, box, strategy = await rig()
    try:
        await send(strategy, adapter, _target(.31), 1)
        strategy.collect(box[0])
        clock.value += 80_000_000
        await observe(adapter, client, clock, box, strategy)
        old = adapter._motion_observation
        await observe(adapter, client, clock, box, strategy)
        adapter._motion_observation = old
        assert not adapter.read_segment_status().ready
        # Two forward frames after a regression must still be insufficient.
        for _ in range(2):
            await observe(adapter, client, clock, box, strategy)
        assert not adapter.read_segment_status().ready
    finally:
        await adapter.disconnect()


async def test_stationary_at_old_joint_pose_is_not_segment_arrival():
    adapter, client, clock, box, strategy = await rig(replace(OPTIONS, position_tolerance_rad=.001))
    try:
        await send(strategy, adapter, _target(.31), 1)
        strategy.collect(box[0])
        clock.value += 80_000_000
        for _ in range(3):
            await observe(adapter, client, clock, box, strategy, close=False)
        assert not adapter.read_segment_status().ready
        assert len(client.write_calls) == 1
    finally:
        await adapter.disconnect()


async def test_nonzero_start_acceleration_cannot_dispatch_segment():
    adapter, client, _clock, box, strategy = await rig()
    client.kin_data['actual_joint_acc'] = [.01,0,0,0,0,0]
    try:
        done = await send(strategy, adapter, _target(.31), 1)
        assert done.sdk_outcome == 'not_started'
        assert done.error == 'segment_initial_not_settled'
        assert strategy.collect(box[0]).fault is not None
        assert [c[0] for c in client.write_calls] == ['stop_move']
    finally:
        await adapter.disconnect()


@pytest.mark.parametrize('stop_late', [False, True])
async def test_full_robot_control_grip_release_owns_stop_and_never_resumes(stop_late):
    from app.control.robot_control import LatestVRFrame, RobotControl
    from app.control.transactional_settings import SegmentedSettings
    from app.control.coordinate_mapper import CoordinateMapper
    from app.recording.noop import NoopRecorder
    from app.schemas.messages import ControllerState, TeleopMode, VRFrame
    adapter, client, clock, _box, _strategy = await rig()
    latest = LatestVRFrame()
    control = RobotControl(backend=adapter, latest=latest, clock=clock,
        recorder=NoopRecorder(), mapper=CoordinateMapper(), limiter=limiter(),
        segmented_settings=SegmentedSettings(OPTIONS))
    seq = 0
    def publish(grip, x=0.):
        nonlocal seq
        seq += 1
        latest.publish(VRFrame(v=1, type='vr_frame', session_id='segment-offline', seq=seq,
            client_mono_ms=clock.now_ns()/1e6, tracking_valid=True, visibility='visible',
            right=ControllerState(p=(x,0,0), q=(0,0,0,1), grip=grip, trigger=0)), clock.now_ns())
    try:
        await control.connect()
        publish(False)
        await control.tick()
        await control.arm()
        publish(True)
        await control.tick()
        assert control.mode is TeleopMode.ACTIVE
        clock.value += 20_000_000
        publish(True, .01)
        await control.tick()
        await asyncio.wait_for(control.target_strategy.receipt.completion.wait(), 2)
        await control.tick()
        assert control.last_target == adapter.read_motion_view().accepted.tcp
        assert [c[0] for c in client.write_calls] == ['move_pvat']
        if stop_late:
            async def late_stop():
                client.write_calls.append(('stop_move',))
                await asyncio.sleep(.621)
            client.stop_move = late_stop
        publish(False)
        await control.tick()
        assert [c[0] for c in client.write_calls] == ['move_pvat', 'stop_move']
        if stop_late:
            assert control._fault == 'stop_unverified'
            assert not adapter._stop_transaction.confirmed
        for _ in range(3):
            publish(True, -.01)
            await control.tick()
        assert [c[0] for c in client.write_calls] == ['move_pvat', 'stop_move']
    finally:
        if stop_late:
            with pytest.raises(BackendCommandError, match='sdk_timeout:stop_move'):
                await adapter.disconnect()
            assert adapter._client is None
            assert [c[0] for c in client.write_calls] == ['move_pvat', 'stop_move']
        else:
            await adapter.disconnect()


async def test_full_vr_chain_reads_new_input_then_advances_and_handles_trigger():
    from app.control.robot_control import LatestVRFrame, RobotControl
    from app.control.transactional_settings import SegmentedSettings
    from app.control.coordinate_mapper import CoordinateMapper
    from app.recording.noop import NoopRecorder
    from app.schemas.messages import ControllerState, VRFrame
    adapter, client, clock, _box, _strategy = await rig()
    latest = LatestVRFrame()
    control = RobotControl(backend=adapter, latest=latest, clock=clock,
        recorder=NoopRecorder(), mapper=CoordinateMapper(), limiter=limiter(),
        segmented_settings=SegmentedSettings(OPTIONS))
    seq = 0
    def publish(grip=True, x=0., trigger=0.):
        nonlocal seq
        seq += 1
        latest.publish(VRFrame(v=1, type='vr_frame', session_id='segment-latest', seq=seq,
            client_mono_ms=clock.now_ns()/1e6, tracking_valid=True, visibility='visible',
            right=ControllerState(p=(x,0,0), q=(0,0,0,1), grip=grip, trigger=trigger)), clock.now_ns())
    try:
        await control.connect()
        publish(grip=False)
        await control.tick()
        await control.arm()
        publish()
        await control.tick()
        clock.value += 20_000_000
        publish(x=.01)
        await control.tick()
        await asyncio.wait_for(control.target_strategy.receipt.completion.wait(), 2)
        await control.tick()
        first = control.last_target.model_copy(deep=True)
        for _ in range(4):
            clock.value += 20_000_000
            publish(x=-.02)
            await control.tick()
            assert len(client.ik_calls) == 1
        # Fresh joint arrival is simulated explicitly; real control never
        # manufactures feedback from a returned command.
        accepted = adapter.read_motion_view().accepted
        client.kin_data['actual_joint_pose'] = list(accepted.solution_q)
        client.kin_data['actual_tcp_pose']['x'] = accepted.position[0]
        for index in range(3):
            clock.value += 20_000_000
            await adapter.get_state()
            publish(x=-.02)
            await control.tick()
            if index < 2:
                assert len(client.ik_calls) == 1
        await asyncio.wait_for(control.target_strategy.receipt.completion.wait(), 2)
        await control.tick()
        assert len(client.ik_calls) == 2
        assert control.last_target.p[0] < first.p[0]
        clock.value += 20_000_000
        publish(x=-.02, trigger=1.)
        await control.tick()
        assert [c[0] for c in client.write_calls] == ['move_pvat','move_pvat','set_claw']
        assert control._fault is None
    finally:
        await adapter.disconnect()


@pytest.mark.parametrize('mode,policy', [
    ('readonly', {'kind':'segmented_experimental', 'settling':asdict(OPTIONS)}),
    ('control', {'kind':'segmented_experimental'}),
    ('control', {'kind':'segmented_experimental', 'settling':{**asdict(OPTIONS),'typo':1}}),
    ('control', {'kind':'segmented_experimental', 'settling':{**asdict(OPTIONS),'settle_samples':2}}),
])
def test_invalid_segment_selection_fails_closed(tmp_path, monkeypatch, mode, policy):
    from tests.control.test_transactional_runtime import config
    with pytest.raises(RuntimeError, match='segmented_requires|invalid_config:target_strategy'):
        config(tmp_path, monkeypatch, mode=mode, policy=policy)


async def test_planner_uses_actual_shorter_reference_horizon():
    adapter, client, _clock, box, strategy = await rig(horizon=.04)
    try:
        plan = strategy.prepare(_target(.3), _target(.31), .02, limiter=limiter())
        # From zero velocity, a=.02 m/s² and dt=.04 s: .02*.04².
        assert plan.pose.p[0] - .3 == pytest.approx(.000032)
        await strategy.dispatch(plan, 1, backend=adapter)
        done = await asyncio.wait_for(strategy.receipt.completion.wait(), 2)
        assert done.sdk_outcome == 'returned' and done.error is None
        assert strategy.collect(box[0]).committed == plan.pose
        assert client.write_calls[0][4] == .04
    finally:
        await adapter.disconnect()


async def test_segment_certifies_finite_joint_window_once_without_skipping_fk(monkeypatch):
    import app.robots.segmented_adapter as segmented
    import app.commissioning.finite_diagnostic as diagnostic
    original=segmented.build_finite_pvat_candidate
    calls=[]
    def counted(*args,**kwargs):
        result=original(*args,**kwargs)
        calls.append(result)
        return result
    monkeypatch.setattr(segmented,'build_finite_pvat_candidate',counted)
    monkeypatch.setattr(diagnostic,'build_finite_pvat_candidate',counted)
    adapter,client,_clock,_box,strategy=await rig()
    try:
        done=await send(strategy,adapter,_target(.31),1)
        assert done.sdk_outcome=='returned' and done.error is None
        assert len(calls)==1
        assert len(client.fk_calls)==4 and len(client.ik_calls)==1
        assert client.write_calls==[('move_pvat',list(calls[0].point.q),[0.]*6,[0.]*6,.08)]
    finally:
        await adapter.disconnect()
