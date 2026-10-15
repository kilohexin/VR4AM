import asyncio
import copy
from dataclasses import replace
import json
from unittest.mock import AsyncMock

import pytest

from app.commissioning import readonly_kinematics as survey
from app.config import Settings
from tests.config.test_real_robot_config import REAL_CONFIG_TEMPLATE
from tests.robots.fake_lebai import FakeLebaiClient, ACTUAL_TCP, IDLE_Q


class SurveyClient(FakeLebaiClient):
    def __init__(self):
        super().__init__()
        self.fk_count = 0

    async def kinematics_inverse(self, pose, joints):
        self.read_calls.append('kinematics_inverse')
        result = list(joints)
        result[0] += pose['z'] - .4
        return result

    async def kinematics_forward(self, joints):
        self.read_calls.append('kinematics_forward')
        self.fk_count += 1
        return {**ACTUAL_TCP, 'z': .4 + joints[0]}


@pytest.fixture
def settings(tmp_path):
    path = tmp_path / 'readonly.yaml'
    path.write_text(REAL_CONFIG_TEMPLATE.format(mode='readonly'))
    robot = Settings.load(path).lebai
    return replace(robot, control=replace(robot.control, max_joint_speed_radps=.05,
                                         max_joint_acceleration_radps2=.2))


async def collect(settings, client, *, admit=lambda: None):
    report = survey.new_report(simulation=True)
    await survey.collect(settings, AsyncMock(return_value=client), admit, report)
    return report


@pytest.mark.parametrize('state', ['IDLE', 'PAUSED', 5, 6])
async def test_fixed_readonly_survey_has_no_writes_or_motion_verdict(settings, state):
    client = SurveyClient()
    client.robot_state = state
    client.running_motion = 12
    report = await collect(settings, client)
    assert report['complete'] and report['kinematics_consistent']
    assert not report['motion_authorized'] and not report['hardware_stop_verified']
    assert report['runtime'] == 'SIMULATION'
    assert client.write_calls == [] and client.fk_count == 4
    assert len([x for x in client.read_calls if x == 'get_kin_data']) == 2
    assert report['candidate']['qd'] == [0.] * 6
    assert report['candidate']['qdd'] == [0.] * 6
    assert report['candidate_full_endpoint_feasible'] == (report['progress_fraction'] == 1.)
    assert report['target']['p'][2] == pytest.approx(.4002)
    assert [x['role'] for x in report['fk_samples']] == ['feedback', 'goal', 'midpoint', 'endpoint']


def test_timing_breakdown_separates_sdk_await_from_diagnostic_checks():
    report=dict(chain_started_ns=100_000_000,chain_elapsed_ms=400.,calls=[
        dict(method='kinematics_forward',started_ns=110_000_000,sdk_await_started_ns=150_000_000,
             sdk_await_completed_ns=160_000_000,completed_ns=180_000_000),
        dict(method='kinematics_inverse',started_ns=200_000_000,sdk_await_started_ns=240_000_000,
             sdk_await_completed_ns=250_000_000,completed_ns=280_000_000)])
    result=survey.chain_timing_breakdown(report)
    assert result['diagnostic_wall_ms']==400.
    assert result['sdk_await_sum_ms']==20.
    assert result['pre_sdk_ms']==80.
    assert result['post_sdk_ms']==50.
    assert result['outside_queries_ms']==250.
    assert result['sdk_call_samples']==2
    assert result['production_lease_verified'] is False


@pytest.mark.parametrize('damage',['no_chain','missing_return','reversed'])
def test_incomplete_timing_is_not_reported_as_zero_sdk_latency(damage):
    report=dict(chain_started_ns=100,chain_elapsed_ms=1.,calls=[dict(method='kinematics_forward',
        started_ns=100,sdk_await_started_ns=200,sdk_await_completed_ns=300,completed_ns=400)])
    if damage=='no_chain':report.pop('chain_elapsed_ms')
    if damage=='missing_return':report['calls'][0].pop('sdk_await_completed_ns')
    if damage=='reversed':report['calls'][0]['sdk_await_completed_ns']=150
    assert survey.chain_timing_breakdown(report) is None


async def test_partial_reference_is_not_full_segment_admission(settings):
    settings=replace(settings,control=replace(settings.control,max_joint_acceleration_radps2=.001))
    report=await collect(settings,SurveyClient())
    assert report['complete'] and report['kinematics_consistent']
    assert report['progress_fraction']<1.
    assert report['candidate_full_endpoint_feasible'] is False
    assert report['motion_authorized'] is False


@pytest.mark.parametrize('state', ['MOVING', 'STOP', 'HOLD', True, None])
async def test_other_states_end_queries_without_stop(settings, state):
    client = SurveyClient()
    client.robot_state = state
    report = await collect(settings, client)
    assert not report['complete'] and not report['kinematics_consistent']
    assert client.fk_count == 0 and client.write_calls == []


@pytest.mark.parametrize('change', ['estop', 'moving_speed', 'small_nonzero_speed', 'tcp_setting', 'missing_acc', 'bad_acc', 'running'])
async def test_feedback_admission_failures_do_not_call_ik_or_fk(settings, change):
    client = SurveyClient()
    if change == 'estop':
        client.estop_reason = 4
    elif change == 'moving_speed':
        client.kin_data['actual_joint_speed'][0] = .03
    elif change == 'small_nonzero_speed':
        client.kin_data['actual_joint_speed'][0] = .001
    elif change == 'tcp_setting':
        client.tcp['z'] += .002
    elif change == 'missing_acc':
        del client.kin_data['actual_joint_acc']
    elif change == 'bad_acc':
        client.kin_data['actual_joint_acc'][0] = float('nan')
    else:
        client.running_motion = 4
        client.motion_state = 'RUNNING'
    report = await collect(settings, client)
    assert not report['complete'] and client.fk_count == 0
    assert 'kinematics_inverse' not in client.read_calls and client.write_calls == []


@pytest.mark.parametrize('role', [1, 2, 3, 4])
async def test_wrong_fk_samples_fail_without_motion(settings, role):
    client = SurveyClient()
    original = client.kinematics_forward

    async def wrong(q):
        result = await original(q)
        if client.fk_count == role:
            result['x'] += .001
        return result

    client.kinematics_forward = wrong
    report = await collect(settings, client)
    assert not report['complete'] and not report['kinematics_consistent']
    assert client.fk_count == role and client.write_calls == []


@pytest.mark.parametrize('field', ['actual_joint_pose', 'actual_tcp_pose', 'tcp_setting', 'robot_state'])
async def test_final_observation_change_invalidates_result(settings, field):
    client = SurveyClient()
    original = client.kinematics_forward

    async def changed(q):
        result = await original(q)
        if client.fk_count == 4:
            if field == 'actual_joint_pose':
                client.kin_data[field][1] += .002
            elif field == 'actual_tcp_pose':
                client.kin_data[field]['x'] += .0002
            elif field == 'tcp_setting':
                client.tcp['z'] += .0001
            else:
                client.robot_state = 'IDLE'  # State transitions are not silently promoted.
        return result

    client.robot_state = 'PAUSED'
    client.kinematics_forward = changed
    report = await collect(settings, client)
    assert not report['complete'] and not report['kinematics_consistent']
    assert client.write_calls == []


async def test_permissions_rechecked_inside_query_task(settings):
    client = SurveyClient()
    factory = AsyncMock(return_value=client)
    checks = 0

    def admit():
        nonlocal checks
        checks += 1
        if checks == 2:
            raise ValueError('expired')

    report = survey.new_report(simulation=True)
    await survey.collect(settings, factory, admit, report)
    factory.assert_not_called()
    assert not report['complete'] and report['calls'][0]['outcome'] == 'not_started'


async def test_future_returning_fk_method_is_supported(settings):
    client = SurveyClient()

    def future(q):
        client.fk_count += 1
        result = asyncio.get_running_loop().create_future()
        result.set_result({**ACTUAL_TCP, 'z': .4 + q[0]})
        return result

    client.kinematics_forward = future
    report = await collect(settings, client)
    assert report['complete'] and client.fk_count == 4 and client.write_calls == []


async def test_non_cancellable_read_has_bounded_exit_and_no_late_rewrite(settings, monkeypatch):
    client = SurveyClient()
    release = asyncio.Event()
    started = asyncio.Event()
    finished = asyncio.Event()
    monkeypatch.setattr(survey, 'READ_TIMEOUT_S', .02)

    async def stubborn(q):
        started.set()
        try:
            await release.wait()
        except asyncio.CancelledError:
            await release.wait()
        finished.set()
        return dict(ACTUAL_TCP)

    client.kinematics_forward = stubborn
    report = await asyncio.wait_for(collect(settings, client), .3)
    assert started.is_set() and not report['complete'] and report['pending_remote_result_unknown']
    frozen = copy.deepcopy(report)
    release.set()
    await finished.wait()
    await asyncio.sleep(0)
    assert report == frozen and client.write_calls == []


async def test_vendor_exception_message_is_not_copied(settings):
    client = SurveyClient()
    client.kinematics_forward = AsyncMock(side_effect=RuntimeError('secret-password'))
    report = await collect(settings, client)
    assert not report['complete'] and 'secret-password' not in json.dumps(report)


async def test_measured_chain_over_vr_ttl_is_evidence_only(settings, monkeypatch):
    client = SurveyClient()
    clock = [1_000_000_000]
    monkeypatch.setattr(survey, 'monotonic_ns', lambda: clock[0])
    original = client.kinematics_forward

    async def slow(q):
        result = await original(q)
        clock[0] += 30_000_000
        return result

    client.kinematics_forward = slow
    report = await collect(settings, client)
    assert report['complete'] and report['kinematics_consistent']
    assert report['chain_elapsed_ms'] >= 120
    assert not report['chain_within_vr_lease'] and not report['motion_authorized']


@pytest.mark.parametrize('change', ['missing_fk', 'no_ik_solution', 'acceleration', 'joint_step', 'singularity', 'joint_limit'])
async def test_missing_capability_or_infeasible_candidate_ends_queries(settings, change):
    client = SurveyClient()
    if change == 'missing_fk':
        client.kinematics_forward = None
    elif change == 'no_ik_solution':
        client.kinematics_inverse = AsyncMock(return_value=None)
    elif change == 'acceleration':
        client.kin_data['actual_joint_acc'][0] = .287
    else:
        q = list(IDLE_Q)
        if change == 'joint_step':
            q[0] += .1
        elif change == 'singularity':
            q[2] = 0
        else:
            settings = replace(settings, soft_joint_max_rad=(.0001, *settings.soft_joint_max_rad[1:]))
            q[0] = .0002
        client.kinematics_inverse = AsyncMock(return_value=q)
    report = await collect(settings, client)
    assert not report['complete'] and client.fk_count == (0 if change in ('missing_fk', 'acceleration') else 1)
    assert client.write_calls == []


@pytest.mark.parametrize('raw', [None, [], {}, {**ACTUAL_TCP, 'rz': float('nan')}, {**ACTUAL_TCP, 'x': True}])
async def test_invalid_fk_data_end_without_following_query(settings, raw):
    client = SurveyClient()
    client.kinematics_forward = AsyncMock(return_value=raw)
    report = await collect(settings, client)
    assert not report['complete'] and client.kinematics_forward.await_count == 1
    assert client.write_calls == []


async def test_cancellation_freezes_unknown_result_and_ends_queries(settings):
    client = SurveyClient()
    started = asyncio.Event()

    async def blocked(q):
        started.set()
        await asyncio.Event().wait()

    client.kinematics_forward = blocked
    report = survey.new_report(simulation=True)
    task = asyncio.create_task(survey.collect(settings, AsyncMock(return_value=client), lambda: None, report))
    await started.wait()
    task.cancel()
    with pytest.raises(asyncio.CancelledError):
        await task
    assert report['error'] == 'interrupted' and report['pending_remote_result_unknown']
    assert not report['complete'] and not report['motion_authorized'] and client.write_calls == []


async def test_total_budget_expires_without_any_sdk_entry(settings, monkeypatch):
    monkeypatch.setattr(survey, 'TOTAL_TIMEOUT_S', 0.)
    factory = AsyncMock()
    report = survey.new_report(simulation=True)
    await survey.collect(settings, factory, lambda: None, report)
    factory.assert_not_called()
    assert not report['complete'] and report['calls'] == []


async def test_strict_vr_lease_boundary_is_not_renewed(settings, monkeypatch):
    client = SurveyClient()
    clock = [1_000_000_000]
    monkeypatch.setattr(survey, 'monotonic_ns', lambda: clock[0])
    original = client.kinematics_forward

    async def boundary(q):
        result = await original(q)
        if client.fk_count == 4:
            clock[0] += 100_000_000
        return result

    client.kinematics_forward = boundary
    report = await collect(settings, client)
    assert report['complete'] and report['chain_elapsed_ms'] == 100
    assert not report['chain_within_vr_lease'] and client.write_calls == []


@pytest.mark.parametrize('raw', ['NONE', 'none', 'None', ' NONE ', '0', 0])
async def test_explicit_vendor_no_estop_representations_are_accepted(settings, raw):
    client = SurveyClient()
    client.robot_state = 'PAUSED'
    client.estop_reason = raw
    report = await collect(settings, client)
    assert report['complete'] and report['kinematics_consistent']
    assert next(c['raw'] for c in report['calls'] if c['method'] == 'get_estop_reason') == raw
    assert client.write_calls == []


@pytest.mark.parametrize('raw', [None, '', False, True, 0., 'UNKNOWN', 'SYSTEM', 4, 'hard_estop', 'NONE extra', [], {}])
async def test_unknown_ambiguous_or_fault_estop_is_still_rejected(settings, raw):
    client = SurveyClient()
    client.estop_reason = raw
    report = await collect(settings, client)
    assert not report['complete'] and report['error'] == 'survey_estop_or_unknown'
    assert client.fk_count == 0 and 'kinematics_inverse' not in client.read_calls
    assert client.write_calls == []


async def test_feedback_fk_precedes_ik_and_survives_ik_timeout_as_evidence(settings, monkeypatch):
    client = SurveyClient()
    monkeypatch.setattr(survey, 'READ_TIMEOUT_S', .02)
    async def blocked(pose, q):
        client.read_calls.append('kinematics_inverse')
        await asyncio.Event().wait()
    client.kinematics_inverse = blocked
    report = await collect(settings, client)
    assert not report['complete'] and report['error'] == 'survey_query_timeout'
    assert report['feedback_fk_consistent'] and not report['kinematics_consistent']
    assert client.fk_count == 1 and client.write_calls == []
    assert client.read_calls.index('kinematics_forward') < client.read_calls.index('kinematics_inverse')
    assert [r['role'] for r in report['fk_samples']] == ['feedback']


async def test_feedback_fk_mismatch_avoids_ik_query(settings):
    client = SurveyClient()
    client.kinematics_forward = AsyncMock(return_value={**ACTUAL_TCP, 'z': .41})
    report = await collect(settings, client)
    assert not report['complete'] and not report['feedback_fk_consistent']
    assert 'kinematics_inverse' not in client.read_calls and client.write_calls == []
