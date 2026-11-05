import asyncio

from scripts import check_cartesian_xr_motion as xr
from scripts import check_cartesian_live_host as live
from app.control.cartesian_pvat_reference import JointReference


def test_input_frames_keep_their_real_clock_while_read_channel_close_is_slow(monkeypatch,tmp_path):
    class Channel:
        def __init__(self,admission):self.first=True
        async def open(self):pass
        async def call(self,name,*args):
            if name=='get_robot_state':return 'IDLE'
            return dict(actual_joint_speed=[0.]*6,actual_tcp_pose=dict(x=.3,y=0.,z=.4,rx=0.,ry=0.,rz=0.))
        async def close(self):
            if self.first:self.first=False;await asyncio.sleep(.55)
    monkeypatch.setattr(xr,'CartesianSignalChannel',Channel)
    monkeypatch.setattr(xr,'CompiledReferenceModel',lambda *a:None)
    monkeypatch.setattr(xr.motion,'analyze',lambda r:r)
    monkeypatch.setattr(xr.motion,'analyze_pitch',lambda r:r)
    async def execute(*args,_input_source,_ready,**kwargs):
        sample=_input_source.sample()
        assert not _input_source.closed and sample is not None and sample.seq>=20
        assert _input_source.binding.generation==7
        _ready.set();await asyncio.sleep(.04)
        assert _input_source.sample().seq>sample.seq
        return dict(passed=True)
    monkeypatch.setattr(xr.live,'execute',execute)
    report=asyncio.run(xr.execute({},tmp_path/'report.json'))
    assert report['passed'] and report['driver_error'] is None and report['input_frames']>=20


def test_program_uses_the_input_control_generation_without_overriding_it():
    source=live.program(JointReference((0.,-.8,1.4,-.6,1.57,0.)),'01'*16,'a9'*32,generation=7)
    assert 'generation=7,words=' in source
    import pytest
    for invalid in (0,True,1.5,2**31):
        with pytest.raises(ValueError,match='generation'):
            live.program(JointReference((0.,)*6),'01'*16,'a9'*32,generation=invalid)


def test_preview_does_not_execute_and_vendor_exception_text_is_not_copied(monkeypatch,tmp_path,capsys):
    monkeypatch.setattr(xr.sys,'argv',['xr','--container','virtual','--output',str(tmp_path/'out.json')])
    monkeypatch.setattr(xr,'native_admission',lambda _:dict(container_id='virtual'))
    monkeypatch.setattr(xr,'execute',lambda *a,**k:(_ for _ in ()).throw(AssertionError('must not run')))
    assert xr.main()==0 and '"execute": false' in capsys.readouterr().out
    assert xr.local_error(ValueError('vr_pose_capture_expired'))=='vr_pose_capture_expired'
    assert xr.local_error(ValueError('arbitrary vendor exception text'))=='ValueError'
