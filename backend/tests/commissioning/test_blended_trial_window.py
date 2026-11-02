from types import SimpleNamespace as NS
import pytest
from app.commissioning.blended_trial_window import TrialWindow,TrialClient


@pytest.mark.parametrize('method',['movej','set_claw','start_sys','stop_sys','move_pvat','resume'])
def test_unscoped_writes_cannot_enter(method):
    w=TrialWindow(lambda:None,lambda:1)
    with pytest.raises(ValueError):w.entry(method)
    with pytest.raises(AttributeError):getattr(TrialClient(NS(),w),method)


def test_count_and_expiry_never_remove_single_reserved_stop():
    now=[1];w=TrialWindow(lambda:None,lambda:now[0])
    for method,cap in [('connect',1),('movel',2),('kinematics_inverse',8),('kinematics_forward',10)]:
        for _ in range(cap):w.entry(method)
        with pytest.raises(ValueError,match='consumed'):w.entry(method)
    now[0]+=21_000_000_000;w.unknown=True
    with pytest.raises(ValueError):w.entry('movel')
    w.entry('stop_move')
    with pytest.raises(ValueError,match='consumed'):w.entry('stop_move')
    now[0]+=5_000_000_000
    with pytest.raises(ValueError,match='cleanup_expired'):w.entry('get_tcp')


async def test_unknown_native_write_is_not_replayed():
    calls=[]
    async def fail(*args):calls.append(args);raise TimeoutError()
    w=TrialWindow(lambda:None,lambda:1);client=TrialClient(NS(movel=fail),w)
    with pytest.raises(TimeoutError):await client.movel(1)
    with pytest.raises(ValueError,match='write_revoked'):await client.movel(1)
    assert len(calls)==1 and w.unknown


async def test_unvalidated_native_activation_rejects_before_any_inputs_or_sdk():
    from scripts.real_robot_blended_trial import run
    with pytest.raises(ValueError,match='activation_pending'):await run({},None)
