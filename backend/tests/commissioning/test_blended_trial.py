from types import SimpleNamespace as NS
import pytest
from app.commissioning.blended_trial import TrialBudget,RealBlendTrialAdapter,RealBlendTrialPort
from app.robots.finite_position_web import RealFinitePositionAdapter
from tests.robots.test_blended_position_docker import rig


def test_budget_is_single_use_expiring_and_preserves_unknown():
    now=[1];b=TrialBudget(lambda:None,lambda:now[0]);b.enter();b.enter()
    with pytest.raises(ValueError,match='consumed'):b.enter()
    now[0]+=20_000_000_000
    with pytest.raises(ValueError,match='expired'):b.check()
    b.unknown=True
    with pytest.raises(ValueError,match='unknown'):b.check()


def test_real_trial_adds_unknown_movel_without_changing_existing_real_selector():
    assert 'movel' in RealBlendTrialAdapter._stop_blocking_motion_methods
    assert 'movel' in RealBlendTrialAdapter._immediate_stop_latch_methods
    assert 'movel' not in RealFinitePositionAdapter._stop_blocking_motion_methods
    assert RealBlendTrialAdapter._stop_rpc_timeout_s==.2
    assert RealBlendTrialAdapter._stop_verification_ns==500_000_000
    with pytest.raises(ValueError,match='adapter_required'):RealBlendTrialPort(NS(),object(),authorize=lambda:None)


def trial():
    old,plan,writes=rig();p=object.__new__(RealBlendTrialPort);p.__dict__.update(old.__dict__)
    p.origin=(0.,0.,0.);p.budget=TrialBudget(lambda:None,lambda:1)
    plan.target.direction=(0.,0.,1.)
    return p,plan,writes


@pytest.mark.parametrize('damage',['reverse','too_far','transverse','expired','revoked','consumed'])
async def test_native_trial_boundary_rejects_without_write(damage):
    p,plan,writes=trial()
    if damage=='reverse':plan.target.direction=(0.,0.,-1.)
    if damage=='too_far':plan.target.position=(0.,0.,.012)
    if damage=='transverse':plan.target.position=(.001,0.,.004)
    if damage=='expired':p.budget.clock=lambda:20_000_000_001
    if damage=='consumed':p.budget.entries=2
    def entering():
        if damage=='revoked':raise ValueError('revoked')
    with pytest.raises(ValueError):await p.submit(plan,entering=entering)
    assert not writes and not p.adapter._motion_calls.unknown_methods


async def test_trial_native_result_unknown_consumes_slot_and_is_not_retried():
    p,plan,writes=trial()
    async def fail(*args):writes.append(args);raise TimeoutError()
    p.client.native.movel=fail
    with pytest.raises(TimeoutError):await p.submit(plan,entering=lambda:None)
    assert p.budget.entries==1 and p.budget.unknown and len(writes)==1
    assert p.adapter._motion_calls.unknown_methods=={'movel'}


async def test_successful_trial_counts_actual_native_entry():
    p,plan,writes=trial()
    assert await p.submit(plan,entering=lambda:None)==42
    assert p.budget.entries==1 and len(writes)==1


@pytest.mark.parametrize('mismatch',[False,True])
async def test_native_fk_checks_each_sample_before_plan_can_enter(monkeypatch,mismatch):
    from app.robots.blended_position_docker import DockerBlendPort
    p,old,writes=trial();calls=[]
    plan=NS(joints=tuple((float(i),)*6 for i in range(5)))
    async def audit(self,target):self.plan=plan;return plan
    monkeypatch.setattr(DockerBlendPort,'audit',audit)
    async def fk(q):
        calls.append(q)
        return dict(x=.01 if mismatch else 0.,y=0.,z=0.,rx=0.,ry=0.,rz=0.)
    async def math_call(name,operation,authorize):authorize();return await operation()
    p.client.kinematics_forward=fk;p.adapter._position_math=math_call
    p.adapter.position_model=NS(forward=lambda q:((0.,0.,0.),(0.,0.,0.,1.)))
    if mismatch:
        with pytest.raises(ValueError,match='native_fk_mismatch'):await p.audit(old.target)
        assert p.plan is None
    else:
        assert await p.audit(old.target) is plan
        assert len(calls)==5
    assert not writes


def test_preview_has_no_execution_authority_or_sdk_import():
    from scripts.preview_real_blended_trial import preview
    value=preview()
    assert value['execute'] is False and value['deployment_ready'] is False
    assert value['sdk_connections']==0 and value['max_movel']==2
    assert value['pending'] and all(len(v)==64 for v in value['source_hashes'].values())
