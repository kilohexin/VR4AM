from datetime import datetime,timezone,timedelta
import json

import pytest
from scripts import real_pvat_capability as probe


def authorization(tmp_path,monkeypatch):
    monkeypatch.setattr(probe,'read_rpc_connections',lambda:0)
    monkeypatch.setattr(probe,'ROOT',tmp_path)
    tool=tmp_path/'tool.json';tool.write_text(json.dumps(dict(mode='readonly',complete=True,config_unchanged=True,
        reads=[dict(method='get_tcp',outcome='returned',raw=dict(x=0.,y=0.,z=0.,rx=0.,ry=0.,rz=0.))])))
    scope=tmp_path/'scope.md';scope.write_text('bounded capability only')
    a=dict(approved=True,confirmation=probe.CONFIRM,target=list(probe.TARGET),
        expires_utc=(datetime.now(timezone.utc)+timedelta(minutes=5)).isoformat(),
        program_sha256=probe.preview()['source_sha256'],launcher_sha256=probe.sha(probe.__file__),
        tool_report=str(tool),tool_report_sha256=probe.sha(tool),scope=str(scope),scope_sha256=probe.sha(scope))
    path=tmp_path/'approval.json';path.write_text(json.dumps(a));return path


def evidence():
    return ([['PVAT_CAP_TOOL']+['0']*6,['PVAT_CAP_FK']+['0']*6]
        +[['PVAT_CAP_CLOCK',str(i),str(i*20)] for i in range(1,6)]
        +[['PVAT_CAP_APIS']+['function']*4,['PVAT_CAP_DONE','true','nil']])


def test_one_fixed_program_single_connection_then_close_and_no_reuse(tmp_path,monkeypatch):
    approval=authorization(tmp_path,monkeypatch);calls=[]
    class Connection:
        def __enter__(self):return self
        def __exit__(self,*args):calls.append('closed')
        def sendall(self,data):assert data==probe.PROGRAM.encode();calls.append('submitted')
        def settimeout(self,value):assert 0<value<=1
        def recv(self,size):return ('\n'.join('\t'.join(r) for r in evidence())+'\n').encode()
    def connect(endpoint,timeout):
        assert endpoint==probe.TARGET and timeout==5;calls.append('connected');return Connection()
    output=tmp_path/'once';report=probe.execute(approval,output,connector=connect)
    assert report['complete'] and not report['unconfirmed_program_completion'] and report['robot_writes']==0
    assert calls==['connected','submitted','closed']
    with pytest.raises(FileExistsError):probe.execute(approval,output,connector=connect)
    with pytest.raises(FileExistsError):probe.execute(approval,tmp_path/'different-output',connector=connect)
    copied=tmp_path/'copied-approval.json';copied.write_bytes(approval.read_bytes())
    with pytest.raises(FileExistsError):probe.execute(copied,tmp_path/'another-output',connector=connect)
    assert len(calls)==3


def test_bad_approval_is_rejected_before_connection_or_artifact_reservation(tmp_path,monkeypatch):
    path=authorization(tmp_path,monkeypatch);a=json.loads(path.read_text());a['program_sha256']='a'*64
    path.write_text(json.dumps(a));output=tmp_path/'once'
    with pytest.raises(ValueError,match='scope'):
        probe.execute(path,output,connector=lambda *a,**k:pytest.fail('must not connect'))
    assert not output.exists()


def test_partial_send_failure_remains_unconfirmed_without_retry(tmp_path,monkeypatch):
    path=authorization(tmp_path,monkeypatch);entered=[]
    class Connection:
        def __enter__(self):return self
        def __exit__(self,*a):pass
        def sendall(self,data):entered.append(1);raise TimeoutError('arbitrary vendor text')
    report=probe.execute(path,tmp_path/'once',connector=lambda *a,**k:Connection())
    assert entered==[1] and not report['complete'] and report['unconfirmed_program_completion']
    assert report['error_type']=='TimeoutError' and 'arbitrary' not in json.dumps(report)


def test_missing_clock_samples_or_apis_do_not_admit_controller():
    assert probe.analyze(evidence())['complete']
    assert not probe.analyze(evidence()[1:])['complete']
    rows=evidence();rows[-2][-1]='nil'
    assert not probe.analyze(rows)['complete']


@pytest.mark.parametrize('row',[
    ['PVAT_CAP_DONE'],['PVAT_CAP_DONE','true'],['PVAT_CAP_DONE','maybe','nil'],
    ['PVAT_CAP_DONE','true','error'],['PVAT_CAP_DONE','false',''],
])
def test_malformed_done_never_confirms_program_completion(row):
    assert not probe.completion_known([row])


def test_valid_failed_probe_is_known_completion_but_never_admission():
    rows=[['PVAT_CAP_DONE','false','capability_requires_idle']]
    assert probe.completion_known(rows) and not probe.analyze(rows)['complete']
    rows=evidence();rows[4][2]='0'
    assert not probe.analyze(rows)['complete']
