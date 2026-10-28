from types import SimpleNamespace
import pytest
from app.digital_twin.docker_runtime import docker_settings
from app.commissioning.finite_position_site import MdhPositionModel
from app.robots.lebai_codec import pose_to_lebai,pose_from_lebai
from app.schemas.messages import Pose
from app.digital_twin.docker_kinematics import admit_local_fk,PROBES


@pytest.mark.parametrize('mismatch',[False,True])
async def test_local_fk_is_admitted_against_controller_and_ik_stays_native(mismatch):
    model=MdhPositionModel.from_settings(docker_settings().lebai);calls=[];inverse=[]
    async def fk(q):
        calls.append(tuple(q));p,r=model.forward(q)
        if mismatch:p=(p[0]+.001,p[1],p[2])
        return pose_to_lebai(Pose(p=p,q=r))
    async def ik(p,seed):inverse.append((p,seed));return seed
    native=SimpleNamespace(kinematics_forward=fk,kinematics_inverse=ik)
    if mismatch:
        with pytest.raises(ValueError,match='model_mismatch'):await admit_local_fk(native,model)
        return
    client=await admit_local_fk(native,model)
    assert calls==list(PROBES)
    q=(.1,-.6,1.1,-.5,1.4,.2);actual=pose_from_lebai(await client.kinematics_forward(list(q)))
    p,r=model.forward(q)
    assert actual.p==pytest.approx(p)
    assert abs(sum(a*b for a,b in zip(actual.q,r)))==pytest.approx(1)
    assert calls==list(PROBES)
    assert await client.kinematics_inverse({'x':.1},list(q))==list(q)
    assert len(inverse)==1


@pytest.mark.parametrize('revoke',[False,True])
async def test_local_fk_yields_to_live_input_and_rechecks_authority(monkeypatch,revoke):
    import asyncio
    from app.robots.finite_position_docker import DockerFinitePositionAdapter
    from app.digital_twin.virtual_claw import DockerArmWithVirtualClaw
    a=DockerFinitePositionAdapter(docker_settings().lebai)
    model=a.position_model
    async def fk(q):
        p,r=model.forward(q);return pose_to_lebai(Pose(p=p,q=r))
    local=await admit_local_fk(SimpleNamespace(kinematics_forward=fk),model)
    ready=DockerArmWithVirtualClaw(local,clock=a._clock)
    a._docker_ready_client=ready;a._client=SimpleNamespace(native=ready)
    monkeypatch.setattr(a,'_position_health',lambda binding:None)
    turns=[];operations=[]
    def authorize():
        if revoke and turns:raise ValueError('position_session_revoked')
    async def operation():
        operations.append(1)
        return await ready.kinematics_forward([0,-.7,1.4,-.7,1.5,0])
    for index in range(5):
        asyncio.get_running_loop().call_soon(turns.append,index)
        if revoke:
            with pytest.raises(ValueError,match='revoked'):
                await a._position_math('kinematics_forward',operation,authorize)
            assert not operations
            return
        await a._position_math('kinematics_forward',operation,authorize)
        assert turns==list(range(index+1)), 'local FK must not starve input/revocation callbacks'
    assert len(operations)==5
