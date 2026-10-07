"""Runtime policy selection and bounded state; injected backends only."""

from dataclasses import asdict, replace
import ast
import inspect

import pytest
import yaml

from app.config import Settings, REAL_ROBOT_CONFIRMATION
from app.control.robot_control import RobotControl, LatestVRFrame
from app.control.target_strategy import DefaultTargetStrategy
from app.control.target_strategy import TargetContext
from app.control.transactional_settings import PursuitSettings
from app.control.transactional_strategy import TransactionalTargetStrategy
from app.recording.noop import NoopRecorder
from tests.control.test_robot_control import FakeClock
from tests.robots.test_lebai_backpressure_experiments import Experiment
from tests.robots.test_vr_profile_experiments import limiter
from tests.config.test_real_robot_config import REAL_CONFIG_TEMPLATE


OPTIONS = PursuitSettings(budget_ms=500, max_observation_age_ms=40,
    max_observation_gap_ms=40, position_tolerance_rad=.001,
    velocity_tolerance_radps=.002, settle_samples=3)


def control(**kwargs):
    return RobotControl(backend=Experiment().adapter, latest=LatestVRFrame(),
        clock=FakeClock(), recorder=NoopRecorder(), limiter=limiter(), **kwargs)


def test_runtime_is_opt_in_and_binds_both_interfaces():
    default = control()
    assert isinstance(default.target_strategy, DefaultTargetStrategy)
    assert default.target_lifecycle is None
    opted = control(transactional_settings=OPTIONS)
    assert isinstance(opted.target_strategy, TransactionalTargetStrategy)
    assert opted.target_lifecycle is opted.target_strategy
    assert opted.target_strategy.coordinator is None
    assert opted.target_strategy.last_receipt is None
    assert not any(isinstance(v, list) for v in vars(opted.target_strategy).values())


@pytest.mark.parametrize('injection', ['strategy', 'lifecycle'])
def test_conflicting_strategy_injection_rejected(injection):
    with pytest.raises(ValueError, match='conflicting_target_strategy'):
        control(transactional_settings=OPTIONS, **{'target_' + injection: object()})


@pytest.mark.parametrize('field', list(asdict(OPTIONS)))
@pytest.mark.parametrize('invalid', [0, -1, True, float('nan'), float('inf'), '1'])
def test_pursuit_parameters_reject_invalid_values(field, invalid):
    with pytest.raises(ValueError, match='invalid_pursuit'):
        replace(OPTIONS, **{field: invalid})


@pytest.mark.parametrize('change', [dict(settle_samples=1), dict(settle_samples=2.5),
    dict(budget_ms=1.5), dict(max_observation_age_ms=600), dict(max_observation_gap_ms=600)])
def test_pursuit_parameters_reject_inconsistent_bounds(change):
    with pytest.raises(ValueError, match='invalid_pursuit'):
        replace(OPTIONS, **change)


def config(tmp_path, monkeypatch, *, mode='control', policy=None):
    monkeypatch.setenv('VR4ARM_REAL_ROBOT_CONFIRM', REAL_ROBOT_CONFIRMATION)
    payload = yaml.safe_load(REAL_CONFIG_TEMPLATE.format(mode=mode))
    if policy is not None:
        payload['target_strategy'] = policy
    path = tmp_path / 'synthetic.yaml'
    path.write_text(yaml.safe_dump(payload), encoding='utf-8')
    return Settings.load(path)


def test_explicit_config_loads_without_connecting(tmp_path, monkeypatch):
    assert config(tmp_path, monkeypatch).transactional_target is None
    selected = config(tmp_path, monkeypatch, policy={
        'kind': 'transactional_experimental', 'pursuit': asdict(OPTIONS)})
    assert selected.transactional_target == OPTIONS


@pytest.mark.parametrize('policy', [{}, {'kind': 'unknown'}, {'kind': 'transactional_experimental'},
    {'kind': 'legacy', 'pursuit': asdict(OPTIONS)},
    {'kind': 'transactional_experimental', 'pursuit': {**asdict(OPTIONS), 'typo': 1}},
    {'kind': 'transactional_experimental', 'pursuit': {**asdict(OPTIONS), 'budget_ms': None}},
    {'kind': 'transactional_experimental', 'pursuit': asdict(OPTIONS), 'typo': 1}])
def test_config_fails_closed_on_invalid_selection(tmp_path, monkeypatch, policy):
    with pytest.raises(RuntimeError, match='invalid_config:target_strategy'):
        config(tmp_path, monkeypatch, policy=policy)


def test_readonly_cannot_select_motion_policy(tmp_path, monkeypatch):
    with pytest.raises(RuntimeError, match='transactional_requires_lebai_control'):
        config(tmp_path, monkeypatch, mode='readonly', policy={
            'kind': 'transactional_experimental', 'pursuit': asdict(OPTIONS)})


def test_production_policy_imports_no_test_modules():
    from app.control import transactional_strategy, candidate_commit, brake_retarget, braking_approach, catch_up
    for module in (transactional_strategy, candidate_commit, brake_retarget, braking_approach, catch_up):
        tree = ast.parse(inspect.getsource(module))
        assert not any(isinstance(n, ast.ImportFrom) and (n.module or '').startswith('tests')
            for n in ast.walk(tree))


def test_anchor_allows_clock_advancing_between_context_reads():
    from tests.control.test_braking_approach import ORIGIN
    tick = 0
    def context():
        nonlocal tick
        tick += 1
        return TargetContext(1, tick, True, True)
    policy = TransactionalTargetStrategy(Experiment().adapter, limiter(), OPTIONS, context=context)
    policy.on_anchor(ORIGIN, context())
    assert policy.coordinator.pose == ORIGIN


async def test_thousand_commits_retain_only_current_receipt():
    import weakref
    import gc
    from app.robots.command_admission import CommandAdmission, CommandCompletion, CommandRef
    from app.robots.motion_view import MotionView
    from tests.control.test_braking_approach import ORIGIN, pose_delta

    class Backend:
        async def submit_tcp(self, pose, command_id):
            ref = CommandRef(command_id, 0, command_id)
            done = CommandCompletion(ref)
            done._resolve('sdk_returned', sdk_outcome='returned', accepted_command=ref, pvat_mode='advance')
            return CommandAdmission(ref, None, completion=done)

        async def submit_catch_up(self, *args):
            raise AssertionError('unexpected_pursuit')

        def read_motion_view(self):
            return MotionView(0, 0, True, True, None, None, True, False, None, None)

    backend, limits = Backend(), limiter()
    context = TargetContext(1, 0, True, True)
    policy = TransactionalTargetStrategy(backend, limits, OPTIONS, context=lambda: context)
    policy.on_anchor(ORIGIN, context)
    refs = []
    for i in range(1, 1001):
        context = replace(context, now_ns=i * 20_000_000)
        plan = policy.prepare(policy.coordinator.pose, pose_delta(ORIGIN, .02, .3), .02, limiter=limits)
        await policy.dispatch(plan, i, backend=backend)
        refs.append(weakref.ref(policy.last_receipt))
        assert policy.collect(context).committed is not None
    gc.collect()
    assert sum(ref() is not None for ref in refs) == 1
    assert policy.coordinator.snapshot().version == 1000
    assert not policy.coordinator.pending
    assert not any(isinstance(v, list) for v in vars(policy).values())


@pytest.mark.parametrize('setting', [dict(budget_ms=120), dict(budget_ms=800)])
def test_custom_budget_is_used_instead_of_old_experiment_constant(setting):
    from app.control.catch_up import CatchUpSupervisor
    from app.robots.command_admission import CommandRef
    now = 0
    params = replace(OPTIONS, **setting)
    probe = CatchUpSupervisor(None, CommandRef(1, 0, 1), (0.,) * 6,
        clock=lambda: now, authorized=lambda: True, generation_current=lambda g: True,
        settings=params)
    now = params.budget_ms * 1_000_000 - 1
    assert probe._check() == 'pursuing'
    now += 1
    assert probe._check() == 'budget_expired'


async def test_actual_app_startup_selects_runtime_policy_with_injected_sdk(tmp_path, monkeypatch):
    from unittest.mock import AsyncMock
    from app.main import create_app
    from tests.robots.fake_lebai import FakeLebaiClient
    settings = config(tmp_path, monkeypatch, policy={
        'kind': 'transactional_experimental', 'pursuit': asdict(OPTIONS)})
    client = FakeLebaiClient.idle()
    factory = AsyncMock(return_value=client)
    monkeypatch.setattr('app.main.build_recorder', lambda *a: NoopRecorder())
    app = create_app(settings=settings, client_factory=factory, backend_label='LEBAI_FAKE')
    async with app.router.lifespan_context(app):
        control = app.state.control
        assert type(control) is RobotControl
        assert type(control.target_strategy) is TransactionalTargetStrategy
        assert control.target_lifecycle is control.target_strategy
        assert control.target_strategy.settings == OPTIONS
        assert not app.state.hardware_verified
        assert client.write_calls == []
    factory.assert_awaited_once()
    assert client.network_calls == client.discovery_calls == 0


def test_session_metadata_identifies_selected_policy(tmp_path, monkeypatch):
    from app.main import build_recorder
    settings = config(tmp_path, monkeypatch, policy={
        'kind': 'transactional_experimental', 'pursuit': asdict(OPTIONS)})
    metadata = build_recorder(settings, 'LEBAI_FAKE').metadata
    assert metadata['target_strategy'] == 'transactional_experimental'
    assert metadata['pursuit_settings'] == asdict(OPTIONS)


def test_selected_policy_preserves_real_motion_confirmation_gate(tmp_path, monkeypatch):
    payload = yaml.safe_load(REAL_CONFIG_TEMPLATE.format(mode='control'))
    payload['target_strategy'] = {'kind': 'transactional_experimental', 'pursuit': asdict(OPTIONS)}
    path = tmp_path / 'synthetic.yaml'
    path.write_text(yaml.safe_dump(payload), encoding='utf-8')
    monkeypatch.delenv('VR4ARM_REAL_ROBOT_CONFIRM', raising=False)
    with pytest.raises(RuntimeError, match='real_robot_confirmation_required'):
        Settings.load(path)


def test_simulator_and_incompatible_backend_rejected():
    from app.control.transactional_settings import parse_target_strategy
    from tests.control.test_robot_control import FakeBackend
    with pytest.raises(RuntimeError, match='transactional_requires_lebai_control'):
        parse_target_strategy({'target_strategy': {'kind': 'transactional_experimental',
            'pursuit': asdict(OPTIONS)}}, backend='simulator', mode=None)
    with pytest.raises(ValueError, match='transactional_backend_required'):
        TransactionalTargetStrategy(FakeBackend(), limiter(), OPTIONS, context=lambda: None)


@pytest.mark.parametrize('pursuit', [False, True])
async def test_old_prepared_plan_cannot_cross_reanchor(pursuit):
    from app.robots.base import BackendCommandError
    from tests.control.test_transactional_vr_tick import rig
    async with rig() as r:
        await r.warm(pursuit=pursuit)
        policy = r.control.target_strategy
        plan = policy.prepare(r.control.last_target, r.control.latest_requested, .02, limiter=r.control.limiter)
        await r.control.disarm()
        await r.tick(grip=False)
        await r.control.arm()
        await r.tick()
        count = len(r.run.client.write_calls)
        with pytest.raises(BackendCommandError, match='candidate_dispatch_failed'):
            await policy.dispatch(plan, 999, backend=r.run.adapter)
        assert len(r.run.client.write_calls) == count


async def test_dispatch_rejects_different_backend_before_submission():
    from app.robots.base import BackendCommandError
    from tests.control.test_transactional_vr_tick import rig
    async with rig() as r:
        policy = r.control.target_strategy
        plan = policy.prepare(r.control.last_target, r.control.last_target, .02, limiter=r.control.limiter)
        with pytest.raises(BackendCommandError, match='candidate_backend_changed'):
            await policy.dispatch(plan, 999, backend=Experiment().adapter)
        assert r.run.client.write_calls == []


@pytest.mark.parametrize('mismatch', ['age', 'gap', 'position', 'velocity', 'none'])
def test_custom_observation_parameters_are_applied(mismatch):
    from app.control.catch_up import CatchUpSupervisor
    from app.robots.command_admission import CommandRef
    from app.robots.motion_view import JointObservation
    params = PursuitSettings(200, 10, 20, .0001, .0002, 2)
    now = 0
    probe = CatchUpSupervisor(None, CommandRef(1, 0, 1), (0.,) * 6,
        clock=lambda: now, authorized=lambda: True, generation_current=lambda g: True,
        settings=params)
    probe.dispatched = True
    now = 10_000_000
    sample = JointObservation(0, now, now, 'state', (0.,) * 6, (0.,) * 6)
    assert probe.observe(sample) == 'pursuing'
    assert probe.good_samples == 1
    now = 20_000_000
    sample = replace(sample, captured_ns=now, read_completed_ns=now)
    if mismatch == 'age':
        now += 11_000_000
    elif mismatch == 'gap':
        now = 31_000_000
        sample = replace(sample, captured_ns=now, read_completed_ns=now)
    elif mismatch == 'position':
        sample = replace(sample, actual_q=(.00011,) * 6)
    elif mismatch == 'velocity':
        sample = replace(sample, actual_qd=(.00021,) * 6)
    expected = 'observed_settled' if mismatch == 'none' else (
        'feedback_expired' if mismatch in {'age','gap'} else 'pursuing')
    assert probe.observe(sample) == expected


def test_optional_step_caps_and_invalid_motion_limits():
    from app.control.safety import SafetyLimiter
    from app.control.brake_retarget import BrakeBeforeRetarget
    from tests.control.test_braking_approach import ORIGIN, pose_delta
    candidate = BrakeBeforeRetarget(SafetyLimiter())
    candidate.limits.set_pose_anchor(ORIGIN)
    assert candidate.limit_motion(ORIGIN, pose_delta(ORIGIN, .01, .01), .02) != ORIGIN
    with pytest.raises(ValueError, match='invalid_transactional_limits'):
        BrakeBeforeRetarget(SafetyLimiter(max_linear_accel=0))
