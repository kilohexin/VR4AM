import pytest
from scripts import check_docker_endpoint_follow as probe


def test_incomplete_or_unknown_evidence_never_passes():
    for raw in ('', 'END_DONE\ttrue\tnil\t3', 'END_DONE\tfalse\tfailed\t0'):
        assert not probe.analyze(dict(raw_output=raw,transport_returncode=0))['passed']


def trace():
    rows=[]
    for t in range(0,1401,20):
        now=t
        if t in (100,300,500):
            i=(t+100)//200;rows += [f'END_ENTER\t{i}\t{t}',f'END_RETURN\t{i}\t{t+20}\t{i}'];now=t+20
        rows.append(f'END_SAMPLE\t{t}\t{now}\t0.0001\t0\t'+('0' if t>=700 else '0.01')+'\t0\t0\t0')
        if t==700:rows.append('END_STABLE\t700\t0\t0\t0\t700')
    rows += ['END_HELD\t1400\t0.0001\t71\t1400','END_DONE\ttrue\tnil\t3']
    return '\n'.join(rows)


def test_complete_sampled_final_endpoint_and_hold_can_pass_only_simulation():
    report=probe.analyze(dict(raw_output=trace(),transport_returncode=0))
    assert report['passed'] and not report['hardware_verified']
    assert report['writes']==3 and report['samples']==71


@pytest.mark.parametrize('change',[
    lambda s:s.replace('0.0001','0.002'),lambda s:s.replace('END_HELD','MISSING'),
    lambda s:s+'\nEND_STOP_ENTER\t1',lambda s:s.replace('END_RETURN\t2','END_RETURN\t1'),
    lambda s:s.replace('END_RETURN\t1\t120','END_RETURN\t1\t500'),
    lambda s:s.replace('END_SAMPLE\t200','END_SAMPLE\t9999'),
])
def test_tampered_or_stopped_trace_is_not_accepted(change):
    assert not probe.analyze(dict(raw_output=change(trace()),transport_returncode=0))['passed']


def test_jittered_hold_window_does_not_require_an_exact_grid():
    raw=trace().replace('END_SAMPLE\t1400\t1400','END_SAMPLE\t1407\t1407').replace('END_HELD\t1400\t0.0001\t71\t1400','END_HELD\t1407\t0.0001\t71\t1407')
    assert probe.analyze(dict(raw_output=raw,transport_returncode=0))['passed']


@pytest.mark.parametrize('change',[
    lambda s:s.splitlines()[-1]+'\n'+'\n'.join(s.splitlines()[:-1]),
    lambda s:s.replace('END_ENTER\t2\t300','END_ENTER\t3\t300'),
    lambda s:s.replace('END_HELD\t1400\t0.0001\t71','END_HELD\t1400\t0.0001\t72'),
    lambda s:s.replace('1400','10000'),
    lambda s:s.replace('END_STABLE\t700','END_STABLE\t600'),
])
def test_event_order_count_and_deadlines_are_validated(change):
    assert not probe.analyze(dict(raw_output=change(trace()),transport_returncode=0))['passed']


def test_feedback_receipt_gap_is_measured_from_previous_capture():
    raw=trace().replace('END_SAMPLE\t1400\t1400','END_SAMPLE\t1400\t1590').replace('END_HELD\t1400\t0.0001\t71\t1400','END_HELD\t1400\t0.0001\t71\t1590')
    assert not probe.analyze(dict(raw_output=raw,transport_returncode=0))['passed']
