import importlib.util
from pathlib import Path
import subprocess

import pytest

spec=importlib.util.spec_from_file_location('docker_pose_continuation_probe',
    Path(__file__).resolve().parents[3]/'scripts/docker_pose_continuation_probe.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


def trace(boundary=False):
    lines=['CONT_REFERENCE\t.3\t0\t.4']
    n=20 if boundary else 551
    for i in range(n):
        x=.3 if boundary else .325
        lines.append(f'LIVE_POINT\t{i*80}\t{(i*2)%1024}\t{x}\t0\t.4\t0')
    if boundary:lines+=['LIVE_BOUNDARY\t2000\treference_boundary']
    else:lines+=['LIVE_EXPIRED\t44100\t551']
    lines+=['CONT_SOURCE_END\t44000\t1500\t.15','LIVE_QUIET\t44700',
            f'LIVE_DONE\ttrue\tnil\t{n}',f'CONT_DONE\ttrue\tnil\t{n}']
    return '\n'.join(lines)


def test_complete_long_trace_requires_old_time_and_range_limits_crossed():
    result=m.analyze(trace(),'continuation')
    assert result['passed'] and result['points']==551 and result['sequence_wrap_observed']
    assert not result['strict_interval_cadence']  # Nominal 80ms is not strictly below 80ms.
    assert result['x_peak_progress_m']==pytest.approx(.025)
    assert result['reference_joint_span_rad']==.15 and result['source_samples']==1500


@pytest.mark.parametrize('change',[
    lambda s:s.replace('44000','9000'),
    lambda s:s.replace('\t0.325\t','\t0.315\t'),
    lambda s:s.replace('\t1500\t','\t900\t'),
    lambda s:s.replace('\t.15','\t.07'),
    lambda s:s.replace('CONT_DONE\ttrue','CONT_DONE\tfalse'),
    lambda s:s.replace('LIVE_DONE\ttrue','LIVE_DONE\tfalse'),
    lambda s:s+'\nLIVE_ABORT\ttrue\ttrue',
    lambda s:s.replace('LIVE_EXPIRED','NOT_EXPIRED'),
    lambda s:s.replace('LIVE_POINT\t80\t2', 'LIVE_POINT\t0\t2'),
    lambda s:s.replace('CONT_DONE\ttrue\tnil\t551','CONT_DONE\ttrue\tnil\t550'),
])
def test_incomplete_short_or_aborted_trace_cannot_pass(change):
    assert not m.analyze(change(trace()),'continuation')['passed']


def test_oversized_goal_must_hold_with_no_meaningful_motion():
    assert m.analyze(trace(True),'boundary')['passed']
    assert not m.analyze(trace(True).replace('\t.3\t','\t.31\t'),'boundary')['passed']


def test_branch_hold_and_fast_hand_keep_known_completion_contract():
    raw=trace(True).replace('reference_boundary','ik_branch')
    assert m.analyze(raw,'branch')['passed']
    assert not m.analyze(raw.replace('LIVE_QUIET','MISSING_QUIET'),'branch')['passed']
    assert m.analyze(trace(),'human')['passed']
    code=m.program('human')
    assert '__HUMAN__' not in code and '__BRANCH__' not in code


def test_uses_exact_product_program_with_injected_synthetic_producer_only():
    code=m.program('continuation')
    assert '__PRODUCT_BODY__' not in code and '__BOUNDARY__' not in code
    assert 'wait_all(producer,receive,plan,motion)' in code
    assert code.count('lebai:movej(')==1
    assert 'maximum_points=3750' in code and 'target_tcp_span=0.1' in code
    with pytest.raises(ValueError):m.program('arbitrary')


def test_unknown_transport_preserves_output_without_retry(monkeypatch):
    calls=[]
    def timeout(*args,**kw):
        calls.append(args);raise subprocess.TimeoutExpired('docker',90,output=b'CONT_REFERENCE\t.3\t0\t.4\n')
    monkeypatch.setattr(m.subprocess,'run',timeout)
    result=m.execute({'container_id':'test-only'},'continuation')
    assert len(calls)==1 and not result['passed'] and result['stop_unverified']
    assert 'CONT_REFERENCE' in result['raw_output']
