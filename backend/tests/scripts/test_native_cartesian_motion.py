import re
import subprocess

import pytest
from scipy.spatial.transform import Rotation

from scripts import check_native_cartesian_motion as probe


@pytest.fixture(scope='module')
def trajectory():
    return probe.build_trajectory((0.,-.8,1.4,-.6,1.57,0.))


def trace():
    samples=[]
    for i in range(30):
        pitch=.03 if i<15 else -.01
        samples.append(f'FINITE_SAMPLE\t{i*20}\t0.0001\t0.3\t{pitch}')
    return '\n'.join(['FINITE_PVAT_ENTER\t1\t0','FINITE_PVAT_ENTER\t2\t0']+samples+[
        'FINITE_QUIET\t600\t0.0001\t30\t0.03\t-0.01\t-0.01',
        'FINITE_MOTION_DONE\ttrue\tnil\t2'])


def result(raw=None):
    return dict(raw_output=trace() if raw is None else raw,transport_returncode=0)


def test_finite_trial_pairs_every_segment_and_ends_quiet_after_pitch_zero_crossing(trajectory):
    model,base,segments=trajectory
    assert 0<len(segments)<=64
    assert all(s.duration==.08 for s in segments)
    assert all(a.end==b.start for a,b in zip(segments,segments[1:]))
    assert max(abs(v) for v in segments[-1].end.v+segments[-1].end.a)<1e-8
    angles=[(Rotation.from_quat(base.q).inv()*Rotation.from_quat(model.pose(s.end.q).q)).as_rotvec()[0]
        for s in segments]
    assert max(angles)>.02 and min(angles)<-.005


def test_native_source_only_has_explicit_bounded_docker_trial_apis(trajectory):
    _,base,segments=trajectory
    prep=probe.preparation_program()
    motion=probe.motion_program(segments[0].start.q,base,segments)
    assert set(re.findall(r'lebai:(\w+)\(',prep+motion))=={
        'get_tcp','get_robot_state','get_kin_data','start_sys','get_velocity_factor',
        'set_velocity_factor','movej','move_pvat','stop_move'}
    assert motion.count('lebai:move_pvat(')==1
    assert 'tcp_path_deviation' in motion and 'pitch_reversal_missing' in motion
    assert 'origin_changed' in motion and 'feedback_gap' in motion
    assert 'native_admission' in probe.transport.__code__.co_names


@pytest.mark.parametrize('change',[
    lambda s:s.replace('DONE\ttrue','DONE\tfalse'),
    lambda s:s.replace('FINITE_PVAT_ENTER\t2','FINITE_PVAT_ENTER\t1'),
    lambda s:s.replace('0.0001','nan'),
    lambda s:s.replace('0.0001','0.0011'),
    lambda s:s.replace('\t-0.01','\t0.01'),
    lambda s:s.replace('QUIET\t600','QUIET\tinf'),
    lambda s:s.replace('\t30\t0.03','\t29\t0.03'),
    lambda s:s.replace('FINITE_SAMPLE\t200','FINITE_SAMPLE\t50'),
    lambda s:s.replace('FINITE_SAMPLE\t200','FINITE_SAMPLE\t400'),
    lambda s:s.replace('FINITE_QUIET','MISSING'),
    lambda s:s+'\nFINITE_ABORT\ttrue\ttrue',
    lambda s:s.replace('nil\t2','nil\t1'),
    lambda s:s.replace('nil\t2','nil\tx'),
])
def test_incomplete_nonfinite_or_inconsistent_evidence_cannot_pass(change):
    assert not probe.analyze_motion(result(change(trace())),2)['passed']


def test_good_evidence_reports_sampled_motion_only():
    evidence=probe.analyze_motion(result(),2)
    assert evidence['passed'] and not evidence['stop_unverified']
    assert evidence['feedback_samples']==30 and evidence['min_pitch_rad']==-.01


def test_native_print_trailing_delimiters_are_supported():
    native='\n'.join(line+'\t' for line in trace().splitlines())
    assert probe.analyze_motion(result(native),2)['passed']


def test_cadence_probe_is_explicit_and_cannot_hide_unbounded_preloading(trajectory):
    _,base,segments=trajectory
    old=probe.motion_program(segments[0].start.q,base,segments)
    balanced=probe.motion_program(segments[0].start.q,base,segments,cadence_probe=True)
    assert '(index-2)*70' in old and 'now-previous<80' in old
    assert '(index-2)*80' in balanced and 'now-previous<100' in balanced and 'FINITE_KIN' in balanced
    raw=trace().replace('FINITE_PVAT_ENTER\t2\t0','FINITE_PVAT_ENTER\t2\t80')
    for line in trace().splitlines():
        if line.startswith('FINITE_SAMPLE\t'):
            raw+='\nFINITE_KIN\t'+line.split('\t')[1]+'\t'+'\t'.join(['0']*18)
    assert not probe.analyze_motion(result(raw),2)['passed']
    evidence=probe.analyze_motion(result(raw),2,cadence_probe=True)
    assert evidence['passed'] and evidence['max_modeled_queue_lead_ms']==80
    assert not evidence['strict_send_interval_observed']
    assert not probe.analyze_motion(result(raw.replace('FINITE_KIN','MISSING')),2,cadence_probe=True)['passed']


def test_prepared_trial_continuation_does_not_repeat_prepare(monkeypatch,trajectory):
    _,base,segments=trajectory
    first=result('FINITE_PREP_ENTER\t\nFINITE_PREP_RETURNED\t\n'
        'FINITE_ORIGIN\t0\t-.8\t1.4\t-.6\t1.57\t0\t\nFINITE_PREP_DONE\ttrue\tnil\t')
    monkeypatch.setattr(probe,'build_trajectory',lambda q:trajectory)
    calls=[]
    def transport(admitted,source,marker):
        calls.append(marker)
        assert marker=='FINITE_MOTION_DONE' and 'lebai:movej(' not in source
        return result('FINITE_MOTION_DONE\tfalse\trejected\t0\t')
    monkeypatch.setattr(probe,'transport',transport)
    evidence=probe.complete_prepared_trial({'container_id':'isolated'},first)
    assert not evidence['passed'] and calls==['FINITE_MOTION_DONE']


def test_unknown_write_stays_unknown_even_if_exception_stop_reports_quiet():
    raw=trace().replace('DONE\ttrue\tnil\t2','DONE\tfalse\tunknown\t1')+'\nFINITE_ABORT\ttrue\ttrue'
    evidence=probe.analyze_motion(result(raw),2)
    assert not evidence['passed'] and evidence['unknown_motion_write'] and evidence['stop_unverified']


@pytest.mark.parametrize('raw',[
    'FINITE_PREP_DONE\tfalse\tread_failed',
    'FINITE_PREP_ENTER\nFINITE_ABORT\ttrue\ttrue\nFINITE_PREP_DONE\tfalse\tunknown',
    'FINITE_PREP_ENTER\nFINITE_PREP_DONE\ttrue\tnil',
])
def test_failed_or_unknown_preparation_never_enters_second_stage(monkeypatch,raw):
    calls=[]
    def transport(*args):
        calls.append(args)
        assert len(calls)==1
        return result(raw)
    monkeypatch.setattr(probe,'transport',transport)
    evidence=probe.execute({'container_id':'isolated'})
    assert not evidence['passed'] and len(calls)==1
    if 'FINITE_PREP_ENTER' in raw:assert evidence['stop_unverified']


def test_transport_rechecks_identity_before_actuation(monkeypatch):
    admitted={'container_id':'isolated'}
    monkeypatch.setattr(probe,'native_admission',lambda _:dict(container_id='changed'))
    monkeypatch.setattr(probe.subprocess,'run',lambda *a,**kw:pytest.fail('no actuation'))
    with pytest.raises(ValueError,match='identity_changed'):probe.transport(admitted,'source','DONE')


def test_transport_timeout_preserves_partial_raw_and_does_not_retry(monkeypatch):
    admitted={'container_id':'isolated'};calls=[]
    monkeypatch.setattr(probe,'native_admission',lambda _:admitted)
    def fail(*args,**kwargs):
        calls.append(args)
        raise subprocess.TimeoutExpired('docker',60,output=b'FINITE_PVAT_ENTER\t1\t0\n')
    monkeypatch.setattr(probe.subprocess,'run',fail)
    evidence=probe.transport(admitted,'source','DONE')
    assert len(calls)==1 and evidence['transport_unknown'] and evidence['stop_unverified']
    assert 'FINITE_PVAT_ENTER' in evidence['raw_output']


def test_preview_does_not_execute_motion(monkeypatch,capsys,tmp_path):
    monkeypatch.setattr(probe.sys,'argv',['probe','--container','isolated','--output',str(tmp_path/'unused.json')])
    monkeypatch.setattr(probe,'native_admission',lambda _:dict(container_id='isolated'))
    monkeypatch.setattr(probe,'execute',lambda _:pytest.fail('preview must not actuate'))
    assert probe.main()==0
    assert '"execute": false' in capsys.readouterr().out
