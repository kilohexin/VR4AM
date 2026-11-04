from pathlib import Path
import json

import pytest
from scripts import check_cartesian_pvat_reference as probe
from app.control.cartesian_reference_bundle import OfflineReferenceSupervisor
from app.control.cartesian_pvat_reference import JointReference,MdhReferenceModel
from app.commissioning.native_jog_mdh import MdhTool


@pytest.mark.parametrize('stride',[0,-1,True,1.5])
def test_bad_coverage_request_rejects_before_reading_the_source(stride):
    with pytest.raises(ValueError,match='offline_reference_braking_stride'):
        probe.replay(Path('does-not-exist'),check_braking=True,braking_stride=stride)


def test_stride_cannot_silently_skip_braking_checks():
    with pytest.raises(ValueError,match='offline_reference_braking_stride'):
        probe.replay(Path('does-not-exist'),check_braking=False,braking_stride=1)


def fixture_log(tmp_path):
    model=MdhReferenceModel(MdhTool((0.,0.,0.),(0.,0.,0.,1.)),(-3.,)*6,(3.,)*6)
    start=JointReference((0.,-.8,1.4,-.6,1.57,0.))
    rows=[dict(kind='robot_kinematics',actual_q=start.q,actual_tcp=model.pose(start.q).model_dump(),server_mono_ns=999_999_999)]
    rows.extend(dict(kind='finite_position_input',event='held',server_mono_ns=1_000_000_000+i*80_000_000,
        hand_pose=dict(p=[i*.001,1.,0.],q=[0.,0.,0.,1.])) for i in range(4))
    path=tmp_path/'fixture.jsonl';path.write_text(''.join(json.dumps(r)+'\n' for r in rows),encoding='utf-8')
    return path


def test_supervised_short_replay_drains_preapproved_release_tail_without_claiming_native_stopping(tmp_path):
    result=probe.replay(fixture_log(tmp_path),supervised=True)
    assert result['offline_candidate_passed'] and result['completed_steps']==3
    assert result['final_braking_steps']>0 and result['final_reference_speed_radps']<1e-8
    assert not result['motion_safety_verified'] and not result['native_interpolation_verified']


def test_following_refusal_is_not_reported_success_even_when_input_ends_before_the_tail(tmp_path,monkeypatch):
    class Refusing(OfflineReferenceSupervisor):
        def __init__(self,*args,**kwargs):
            super().__init__(*args,**kwargs)
            # Deliberately keep the quiet approved tail longer than input.
            # Production quiet seeding is now just one validated hold.
            self._tail*=8;self._tail_checks*=8
        def advance(self,target):return self.finish('no_candidate_with_braking_tail')
    monkeypatch.setattr(probe,'OfflineReferenceSupervisor',Refusing)
    result=probe.replay(fixture_log(tmp_path),supervised=True)
    assert not result['offline_candidate_passed'] and not result['all_reference_steps_admitted']
    assert result['backup_braking_steps']>0 and result['final_braking_steps']>0
    assert result['final_reference_speed_radps']<1e-8


def test_budgeted_replay_reports_work_limit_and_preserves_no_native_admission(tmp_path):
    result=probe.replay(fixture_log(tmp_path),supervised=True,governed_continuation=True,braking_segment_limit=24)
    assert result['offline_candidate_passed']
    assert result['braking_segment_limit']==24 and 0<result['max_braking_segment_solves_per_step']<=24
    assert result['sdk_connections']==result['motion_writes']==0
    assert not result['realtime_verified'] and not result['product_admission_complete']


def test_work_limit_cannot_be_enabled_without_supervision():
    with pytest.raises(ValueError,match='supervisor_required'):
        probe.replay(Path('does-not-exist'),braking_segment_limit=24)
