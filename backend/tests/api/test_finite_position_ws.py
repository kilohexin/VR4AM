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
