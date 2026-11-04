import importlib.util
from pathlib import Path
import pytest

spec=importlib.util.spec_from_file_location('docker_clock_probe',Path(__file__).resolve().parents[3]/'scripts/docker_clock_probe.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


def test_completed_probe_requires_full_monotonic_observation():
    result=m.analyze('CLOCK_DONE\ttrue\t3000\t19993\n',0)
    assert result['clock_valid'] and result['elapsed_ms']==19993
    for name in ('movej(', 'move_pvat(', 'stop_move(', 'set_signal(', 'start_sys('):
        assert name not in m.PROGRAM


@pytest.mark.parametrize('raw',[
    '', 'CLOCK_DONE', 'CLOCK_DONE\ttrue\t1\t10000', 'CLOCK_DONE\ttrue\t3000\t0',
    'CLOCK_DONE\ttrue\t3000\tnan', 'CLOCK_DONE\tfalse\t3000\t10000',
    'CLOCK_DONE\ttrue\t3000\t10000\nCLOCK_DONE\ttrue\t3000\t10000',
    'CLOCK_FAILURE\tafter_read\t999\t1000\t1000\t2\nCLOCK_DONE\ttrue\t3000\t10000',
])
def test_partial_corrupt_or_rolled_back_clock_cannot_pass(raw):
    assert not m.analyze(raw,0)['clock_valid']


def test_failed_transport_keeps_the_raw_record():
    raw='CLOCK_DONE\ttrue\t3000\t19993\n'
    result=m.analyze(raw,1)
    assert result['raw_output']==raw and not result['clock_valid']


def test_preview_performs_no_controller_io(monkeypatch,tmp_path,capsys):
    import json
    monkeypatch.setattr(m,'admission',lambda _:dict(container_id='test'))
    monkeypatch.setattr(m.sys,'argv',['probe','--container','test','--output',str(tmp_path/'record.json')])
    monkeypatch.setattr(m.subprocess,'run',lambda *a,**kw:pytest.fail('preview cannot execute'))
    assert m.main()==0
    assert json.loads(capsys.readouterr().out)['execute'] is False
    assert not (tmp_path/'record.json').exists()


def test_read_timeout_keeps_partial_evidence_and_does_not_retry(monkeypatch,tmp_path,capsys):
    import json
    calls=[]
    def timeout(*a,**kw):
        calls.append(a);raise m.subprocess.TimeoutExpired('docker',35,output=b'CLOCK_FAILURE\tafter_read\t999\t1000\n')
    monkeypatch.setattr(m,'admission',lambda _:dict(container_id='test'))
    monkeypatch.setattr(m.sys,'argv',['probe','--container','test','--run','--output',str(tmp_path/'record.json')])
    monkeypatch.setattr(m.subprocess,'run',timeout)
    assert m.main()==1 and len(calls)==1
    result=json.loads((tmp_path/'record.json').read_text())
    assert result['error']=='read_result_unknown' and 'CLOCK_FAILURE' in result['raw_output']
    assert result['motion_calls']==0 and result['signal_writes']==0
