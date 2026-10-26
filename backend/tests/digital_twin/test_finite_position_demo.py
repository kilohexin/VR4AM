import importlib
import pytest


def api():
    assert importlib.util.find_spec('app.digital_twin.finite_position_demo'),'finite position web demo missing'
    return importlib.import_module('app.digital_twin.finite_position_demo')


def test_demo_has_explicit_candidate_identity_and_real_target_rejected():
    m=api();s=m.demo_settings()
    assert s.lebai.ip=='digital-twin' and s.segmented_target is None and s.transactional_target is None
    assert m.PositionDemoModel().factory_calibrated is False


async def test_demo_movej_advances_actual_position_before_finished():
    m=api();s=m.demo_settings();now=[0]
    client=m.FinitePositionDemoClient(s.lebai,clock=lambda:now[0],initial_q=s.lebai.teleop_ready_q)
    q=list(s.lebai.teleop_ready_q);q[0]+=.001
    motion_id=await client.movej(q,.02,.01,0.,0.)
    assert await client.get_motion_state(motion_id)=='RUNNING'
    now[0]=150_000_000;kin=await client.get_kin_data()
    assert s.lebai.teleop_ready_q[0]<kin['actual_joint_pose'][0]<q[0]
    now[0]=2_000_000_000;kin=await client.get_kin_data()
    assert abs(kin['actual_joint_pose'][0]-q[0])<1e-9
    assert await client.get_motion_state(motion_id)=='FINISHED'


def test_demo_model_has_geometry_preparation_before_native_entry():
    m=api();s=m.demo_settings();model=m.PositionDemoModel()
    q=list(s.lebai.teleop_ready_q);q[0]+=.001
    assert callable(getattr(model,'validate_path',None))
    model.validate_path(s.lebai.teleop_ready_q,tuple(q),s.lebai)
    q[0]=100.
    with pytest.raises(ValueError):model.validate_path(s.lebai.teleop_ready_q,tuple(q),s.lebai)


async def test_demo_inverse_yields_to_release_while_solving(monkeypatch):
    import asyncio,threading
    m=api();s=m.demo_settings();client=m.FinitePositionDemoClient(s.lebai,initial_q=s.lebai.teleop_ready_q)
    entered=threading.Event();released=threading.Event();order=[];original=m.np.linalg.solve
    def slow(*args,**kwargs):
        entered.set();released.wait(.15);order.append('solve');return original(*args,**kwargs)
    monkeypatch.setattr(m.np.linalg,'solve',slow)
    p,q=m.PositionDemoModel().forward(s.lebai.teleop_ready_q)
    from app.robots.lebai_codec import pose_to_lebai
    from app.schemas.messages import Pose
    async def release():
        while not entered.is_set():await asyncio.sleep(0)
        order.append('release');released.set()
    task=asyncio.create_task(release())
    try:
        result=await client.kinematics_inverse(pose_to_lebai(Pose(p=(p[0]+.0005,p[1],p[2]),q=q)),list(s.lebai.teleop_ready_q))
        await task;assert result is not None and order[0]=='release'
    finally:released.set();task.cancel()


@pytest.mark.parametrize('direction',[-1.,1.])
@pytest.mark.parametrize('translation',[False,True])
async def test_lm3_model_pitch_plans_and_naturally_arrives_without_stop(direction,translation):
    import math
    from app.robots.finite_position_adapter import FinitePositionAdapter
    from app.control.finite_position_types import PositionIntent
    from app.robots.finite_position_plan import angle
    from tests.robots.test_lebai_adapter_control import FakeClock
    m=api();settings=m.demo_settings().lebai;clock=FakeClock()
    client=m.FinitePositionDemoClient(settings,clock=clock.now_ns,initial_q=settings.teleop_ready_q)
    async def factory(ip):
        assert ip=='digital-twin'
        return client
    adapter=FinitePositionAdapter(settings,client_factory=factory,backend_label='LEBAI_FAKE',
        position_model=m.PositionDemoModel(),clock=clock.now_ns,sleep=clock.sleep)
    await adapter.connect();adapter.bind_position_generation(1)
    try:
        plan=await adapter.plan_step(PositionIntent('web',1,1,clock.now_ns(),'held',(0.,0.,1. if translation else 0.),direction),authorize=lambda:None)
        result=await adapter.execute_step(plan,before_entry=lambda:None,monitor_authorize=lambda:None,on_entered=lambda:None)
        assert result.complete,result
        assert math.dist(result.final_sample.tcp_p,plan.target_p)<.0001
        assert angle(result.final_sample.tcp_q,plan.target_q)<math.radians(.1)
        assert [x[0] for x in client._write_calls]==['movej']
    finally:await adapter.disconnect()
