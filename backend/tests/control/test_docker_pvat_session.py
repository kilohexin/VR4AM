from types import SimpleNamespace as NS
import pytest
from app.control.docker_pvat_session import DockerPvatSession,DockerPvatOrWebSession
from app.control.finite_position_types import PositionIntent
from app.robots.finite_position_docker import DockerFinitePositionAdapter


def make():
    a=object.__new__(DockerFinitePositionAdapter);a._docker_native=True;a._latched_fault=None
    a._position_health=lambda binding:None
    s=DockerPvatSession(a,NS(generation=2),clock=lambda:1_000_000_000,recorder=NS(fatal_error=None,dropped_normal_events=0),authorize=lambda:None)
    return a,s


def intent(seq=1,**kwargs):
    return PositionIntent(kwargs.pop('session_id','xr-finite-test'),kwargs.pop('generation',2),seq,1_000_000_000,
        kwargs.pop('kind','held'),kwargs.pop('vector',(0,0,1)),**kwargs)


@pytest.mark.parametrize('i',[intent(generation=3),intent(session_id='desktop'),intent(pitch=1)])
def test_other_generation_or_unsupported_source_cannot_enter(i):
    _,s=make()
    with pytest.raises(ValueError):s.offer(i)
    assert s.worker is None


def test_late_held_after_hold_cannot_resume_until_explicit_release():
    _,s=make();s.hold=True;s.phase='hold'
    s.offer(intent());assert s.hold and s.phase=='hold'
    s.offer(intent(2,kind='release',vector=(0,0,0)));assert not s.hold and s.phase=='ready'


def test_release_retires_forwarding_until_worker_drains():
    _,s=make();forwarded=[]
    s.pump=NS(closed=False,offer=lambda **kw:forwarded.append(kw))
    s.worker=object()
    s.offer(intent());assert forwarded[-1]['action']==(0.,0.,1.)
    s.offer(intent(2,kind='release',vector=(0,0,0)))
    s.offer(intent(3));assert len(forwarded)==1 and s._withdraw.is_set()


def test_selector_locks_source_before_any_other_executor_can_run(monkeypatch):
    from app.control import docker_motion_session as motion
    class Web:
        def __init__(self,*args,**kwargs):self.inputs=[]
        def offer(self,i):self.inputs.append(i)
    monkeypatch.setattr(motion,'DockerMotionSession',Web)
    s=DockerPvatOrWebSession(None,None)
    s.offer(intent(session_id='web'))
    with pytest.raises(ValueError,match='requires_preparation'):s.offer(intent(2))
    assert len(s.active.inputs)==1


def test_fake_adapter_cannot_enable_native_controller_program():
    with pytest.raises(ValueError,match='native_docker'):DockerPvatSession(NS(),NS(),clock=lambda:0,recorder=None,authorize=lambda:None)


async def test_cancelled_page_reader_joins_owned_snapshot_before_exit():
    import asyncio
    a,_=make();a.position_pvat_vr=True;a.position_busy=True
    entered=asyncio.Event();complete=asyncio.Event();outcomes=[]
    async def read():
        entered.set()
        try:await complete.wait();outcomes.append('returned')
        except asyncio.CancelledError:outcomes.append('abandoned');raise
    a._get_docker_state=read
    task=asyncio.create_task(a.get_state());await entered.wait();task.cancel();await asyncio.sleep(0)
    assert not task.done() and not outcomes
    complete.set()
    with pytest.raises(asyncio.CancelledError):await task
    assert outcomes==['returned']
