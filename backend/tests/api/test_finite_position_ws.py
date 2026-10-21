import pytest
from pydantic import ValidationError
from app.api.teleop_ws import _parse_message


def message(**kwargs):
    return dict(v=1,type='finite_position_input',session_id='web',generation=1,seq=1,
        kind='held',vector=[1.,0.,0.],**kwargs)


def test_new_position_input_parses_without_reinterpreting_stop():
    parsed=_parse_message(message())
    assert parsed.type=='finite_position_input' and parsed.vector==(1.,0.,0.)
    assert _parse_message(dict(v=1,type='disarm',request_id='stop')).type=='disarm'


@pytest.mark.parametrize('field,value',[('vector',[True,0,0]),('generation',True),
    ('seq',2**53),('vector',[float('nan'),0,0]),('vector',[1,2]),('kind','stop')])
def test_new_input_rejects_unsafe_fields(field,value):
    row=message();row[field]=value
    with pytest.raises((ValueError,ValidationError)):_parse_message(row)


async def test_controller_candidate_requires_prepare_and_exposes_status():
    from app.control.robot_control import RobotControl,LatestVRFrame
    from app.recording.noop import NoopRecorder
    from app.schemas.messages import TeleopMode
    from tests.robots.test_finite_position_adapter import prepared
    a,c,clock,_=await prepared()
    control=RobotControl(backend=a,latest=LatestVRFrame(),clock=clock,recorder=NoopRecorder())
    try:
        with pytest.raises(ValueError):control.accept_position_input(_parse_message(message()),clock.now_ns())
        control.machine.mode=TeleopMode.ARMED;control._install_position_session()
        status=control._position_status_message()
        assert status.policy=='finite_position_offline' and status.phase=='ready'
        row=message();row['generation']=control.control_generation
        control.accept_position_input(_parse_message(row),clock.now_ns())
        assert control.finite_position_session.gate.latest.seq==1
        assert [x[0] for x in c.write_calls]==[]
    finally:await control.stop();await a.disconnect()


async def test_disconnect_retires_position_session_without_clearing_fault():
    from app.control.robot_control import RobotControl,LatestVRFrame
    from app.recording.noop import NoopRecorder
    from app.schemas.messages import TeleopMode
    from tests.robots.test_finite_position_adapter import prepared
    a,_,clock,_=await prepared();control=RobotControl(backend=a,latest=LatestVRFrame(),clock=clock,recorder=NoopRecorder())
    try:
        control.machine.mode=TeleopMode.ARMED;control._install_position_session()
        await control.on_disconnect()
        assert control.finite_position_session is None
        assert control._position_status_message().phase=='inactive'
    finally:await control.stop();await a.disconnect()


async def test_only_zero_motion_candidate_startup_can_rebase_idle_deadline():
    from app.control.robot_control import RobotControl,LatestVRFrame
    from app.recording.noop import NoopRecorder
    from app.schemas.messages import TeleopMode
    from tests.robots.test_finite_position_adapter import prepared
    a,_,clock,_=await prepared();control=RobotControl(backend=a,latest=LatestVRFrame(),clock=clock,recorder=NoopRecorder())
    try:
        control.machine.mode=TeleopMode.READY
        assert control._position_idle_deadline_rebase()
        control.machine.mode=TeleopMode.ACTIVE
        assert not control._position_idle_deadline_rebase()
        control.machine.mode=TeleopMode.READY;a._motion_write_stop_required=True
        assert not control._position_idle_deadline_rebase()
        a._motion_write_stop_required=False;control._stop_unverified=True
        assert not control._position_idle_deadline_rebase()
    finally:await control.stop();await a.disconnect()


async def test_finite_session_uses_its_input_watchdog_not_stream_catchup_deadline(monkeypatch):
    from app.control.robot_control import RobotControl,LatestVRFrame
    from app.recording.noop import NoopRecorder
    from app.schemas.messages import TeleopMode
    from tests.robots.test_finite_position_adapter import prepared
    a,_,clock,_=await prepared();control=RobotControl(backend=a,latest=LatestVRFrame(),clock=clock,recorder=NoopRecorder())
    control.machine.mode=TeleopMode.ARMED;control._install_position_session();control.machine.mode=TeleopMode.ACTIVE
    count=0
    async def tick():
        nonlocal count
        count+=1
        if count==1:await clock.sleep(.1)
        if count==3:control._running=False
    monkeypatch.setattr(control,'tick',tick);control._running=True
    try:
        await control.run()
        assert control._fault is None and count==3
    finally:await control.stop();await a.disconnect()


@pytest.mark.parametrize('visible,tracked',[(False,True),(True,False)])
async def test_hidden_or_untracked_frame_revokes_finite_authority(visible,tracked):
    from app.control.robot_control import RobotControl,LatestVRFrame
    from app.recording.noop import NoopRecorder
    from app.schemas.messages import TeleopMode,VRFrame
    from tests.robots.test_finite_position_adapter import prepared
    a,c,clock,_=await prepared();latest=LatestVRFrame()
    control=RobotControl(backend=a,latest=latest,clock=clock,recorder=NoopRecorder())
    try:
        control.machine.mode=TeleopMode.ARMED;control._install_position_session()
        latest.publish(VRFrame(v=1,type='vr_frame',session_id='desktop',seq=1,client_mono_ms=1,
            tracking_valid=tracked,visibility='visible' if visible else 'hidden',
            right=dict(p=[0,0,0],q=[0,0,0,1],grip=False,trigger=0)),clock.now_ns())
        with pytest.raises(ValueError,match='position_input_unavailable'):
            control.finite_position_session.authorize()
        assert c.write_calls==[]
    finally:await control.stop();await a.disconnect()
