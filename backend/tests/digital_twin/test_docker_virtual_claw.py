import pytest
from app.digital_twin.virtual_claw import DockerArmWithVirtualClaw


async def test_local_claw_ramps_to_selected_opening_and_holds_without_native_claw_calls():
    class Native:
        async def get_robot_state(self):return 'IDLE'
        async def get_claw(self):raise AssertionError('native claw unavailable')
        async def set_claw(self,*args):raise AssertionError('native claw unavailable')
    now=[0];client=DockerArmWithVirtualClaw(Native(),clock=lambda:now[0])
    await client.set_claw(30,40)
    assert (await client.get_claw())['amplitude']==0
    now[0]=250_000_000
    assert (await client.get_claw())['amplitude']==pytest.approx(15)
    now[0]=1_000_000_000
    assert (await client.get_claw())['amplitude']==40
    now[0]=2_000_000_000
    assert (await client.get_claw())['amplitude']==40
    assert await client.get_robot_state()=='IDLE'


@pytest.mark.parametrize('force,amplitude',[(31,20),(30,101),(30,-1),(float('nan'),20),(True,0)])
async def test_local_claw_rejects_invalid_or_excessive_targets(force,amplitude):
    client=DockerArmWithVirtualClaw(object(),clock=lambda:0)
    with pytest.raises(ValueError):await client.set_claw(force,amplitude)
