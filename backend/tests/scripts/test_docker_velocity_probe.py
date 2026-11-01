import pytest
from scripts.docker_velocity_probe import probe

class Client:
    def __init__(self):
        self.pose=dict(x=.3,y=0.,z=.3,rx=0.,ry=0.,rz=0.)
        self.pending=[];self.calls=0;self.stops=0
    async def get_robot_state(self):return 'IDLE'
    async def get_kin_data(self):return dict(actual_joint_speed=[0.]*6,actual_tcp_pose=dict(self.pose))
    async def movel(self,p,a,v,t,r):
        assert len(self.pending)<2
        assert a==.1 and v in (.02,.08) and t==0 and r==.001
        self.calls+=1;self.pending.append(self.calls);self.pose=dict(p);return self.calls
    async def get_motion_state(self,ident):
        assert ident==self.pending.pop(0)
        return 'FINISHED'
    async def stop_move(self):self.stops+=1

@pytest.mark.parametrize('geometric',[False,True])
async def test_bounded_blend_never_submits_more_than_two_unfinished_targets(geometric):
    c=Client();r=await probe(c,bounded=True,geometric=geometric)
    assert r['passed'] and len(r['commands'])==6 and c.stops==0
    assert c.pose['z']==pytest.approx(.324)
    if geometric:assert max(x['lead_m'] for x in r['commands'])<=.008

async def test_failed_motion_is_not_retried_and_stops_once():
    c=Client()
    async def fail(*args):c.calls+=1;raise RuntimeError('test_failure')
    c.movel=fail
    r=await probe(c,bounded=True)
    assert not r['passed'] and c.calls==1 and c.stops==1
    assert r['unknown']==['movel']


async def test_geometric_reverse_drains_before_opposite_target():
    c=Client();seen=set();original=c.get_motion_state
    async def wait_then_finish(ident):
        if ident not in seen:seen.add(ident);return 'WAIT'
        return await original(ident)
    c.get_motion_state=wait_then_finish
    r=await probe(c,geometric=True,reverse=True)
    assert r['passed'] and c.stops==0 and not r['unknown']
    assert c.pose['z']==pytest.approx(.3)
    assert len(r['commands'])==6
    assert r['reverse_drained_t']-r['reverse_requested_t']>=.2
    assert r['commands'][2]['t']<r['reverse_requested_t']<r['reverse_drained_t']<r['commands'][3]['t']
    assert all(x['t']<r['release_t'] for x in r['commands'])
    assert max(x['lead_m'] for x in r['commands'])<=.008


@pytest.mark.parametrize('axis',['pitch','spin'])
@pytest.mark.parametrize('reverse',[False,True])
async def test_rotation_keeps_tcp_fixed_and_angular_queue_bounded(axis,reverse):
    import math
    from scipy.spatial.transform import Rotation
    from app.robots.lebai_codec import pose_from_lebai
    c=Client();start=pose_from_lebai(c.pose)
    r=await probe(c,rotation=axis,reverse=reverse)
    assert r['passed'] and not r['unknown'] and c.stops==0
    assert len(r['commands'])==6
    for command in r['commands']:
        assert pose_from_lebai(command['target']).p==start.p
        assert command['lead_deg']<=1.+1e-10 and 'lead_m' not in command
    end=pose_from_lebai(c.pose)
    relative=(Rotation.from_quat(start.q).inv()*Rotation.from_quat(end.q)).as_rotvec()
    assert math.degrees(relative[0 if axis=='pitch' else 2])==pytest.approx(0. if reverse else 3.,abs=1e-7)
