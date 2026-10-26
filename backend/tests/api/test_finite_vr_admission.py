"""VR software capability admission with injected clients, never real access."""
import pytest
from app.control.robot_control import RobotControl,LatestVRFrame
from app.recording.noop import NoopRecorder
from app.schemas.messages import TeleopMode
from app.api.teleop_ws import _parse_message
from tests.robots.test_finite_position_web import make,profile

async def test_real_web_default_rejects_xr_finite_intents():
    a,c,clock=await make();await a.connect()
    control=RobotControl(backend=a,latest=LatestVRFrame(),clock=clock,recorder=NoopRecorder())
    try:
        control.machine.mode=TeleopMode.ARMED;control._install_position_session()
        assert control._position_status_message().vr_enabled is False
        with pytest.raises(ValueError,match='vr_not_authorized'):
            control.accept_position_input(_parse_message(dict(v=1,type='finite_position_input',
                session_id='xr-finite-test',generation=control.control_generation,seq=1,
                kind='held',vector=[0,0,1])),clock.now_ns())
        assert c.write_calls==[]
    finally:await a.disconnect()


async def test_real_web_does_not_inherit_offline_pitch_permission():
    a,c,clock=await make();await a.connect()
    control=RobotControl(backend=a,latest=LatestVRFrame(),clock=clock,recorder=NoopRecorder())
    try:
        control.machine.mode=TeleopMode.ARMED;control._install_position_session()
        assert control._position_status_message().pitch_enabled is False
        with pytest.raises(ValueError,match='pitch_not_authorized'):
            control.accept_position_input(_parse_message(dict(v=1,type='finite_position_input',
                session_id='web',generation=control.control_generation,seq=1,
                kind='held',vector=[0,0,0],pitch=1.)),clock.now_ns())
        assert not c.write_calls
    finally:await a.disconnect()


async def test_explicit_pitch_reaches_real_selector_only_with_injected_client():
    from app.main import build_backend
    from app.config import Settings
    from dataclasses import replace
    s,c,clock=profile();calls=[]
    async def factory(ip):calls.append(ip);return c
    a=build_backend(replace(Settings.load(),backend='lebai',lebai=s,segmented_target=None,transactional_target=None),
        recorder=NoopRecorder(),client_factory=factory,
        backend_label='LEBAI',real_finite_position=True,real_position_authorize=lambda:None,
        real_position_model=c.model,real_position_pitch=True)
    assert a.position_pitch_enabled is True and a.position_vr_enabled is False
    assert not calls  # Construction and capability selection never connect.

async def test_explicit_vr_capability_accepts_input_without_sending_a_target():
    from app.robots.finite_position_web import RealFinitePositionAdapter
    s,c,clock=profile()
    async def factory(ip):return c
    a=RealFinitePositionAdapter(s,client_factory=factory,position_model=c.model,
        authorize=lambda:None,allow_vr=True,clock=clock.now_ns,sleep=clock.sleep)
    await a.connect();control=RobotControl(backend=a,latest=LatestVRFrame(),clock=clock,recorder=NoopRecorder())
    try:
        control.machine.mode=TeleopMode.ARMED;control._install_position_session()
        assert control._position_status_message().vr_enabled is True
        control.accept_position_input(_parse_message(dict(v=1,type='finite_position_input',
            session_id='xr-finite-test',generation=control.control_generation,seq=1,
            kind='held',vector=[0,0,1])),clock.now_ns())
        assert control.finite_position_session.gate.latest.seq==1 and c.write_calls==[]
    finally:await a.disconnect()

async def test_neutral_xr_transport_preserves_preparation_after_release_and_next_gesture():
    from app.schemas.messages import VRFrame
    from app.robots.finite_position_web import RealFinitePositionAdapter
    s,c,clock=profile()
    async def factory(ip):return c
    a=RealFinitePositionAdapter(s,client_factory=factory,position_model=c.model,
        authorize=lambda:None,allow_vr=True,clock=clock.now_ns,sleep=clock.sleep)
    await a.connect();latest=LatestVRFrame()
    control=RobotControl(backend=a,latest=latest,clock=clock,recorder=NoopRecorder())
    try:
        control.machine.mode=TeleopMode.ARMED;control._install_position_session()
        for seq,kind in enumerate(('held','release','held','release'),1):
            # finiteXRTransportFrame carries no legacy clutch/Trigger values.
            latest.publish(VRFrame(v=1,type='vr_frame',session_id='quest-test',seq=seq,client_mono_ms=seq,
                tracking_valid=True,visibility='visible',right=dict(p=[0,0,0],q=[0,0,0,1],grip=False,trigger=0)),clock.now_ns())
            control.accept_position_input(_parse_message(dict(v=1,type='finite_position_input',session_id='xr-finite-test',
                generation=control.control_generation,seq=seq,kind=kind,vector=[0,0,1] if kind=='held' else [0,0,0])),clock.now_ns())
            assert control._control_generation_is_current(control.control_generation)
            assert (await control.state_message()).manual_input_ready
        await control.command_gripper(.6)
        assert [x[0] for x in c.write_calls]==['set_claw']
    finally:await a.disconnect()
