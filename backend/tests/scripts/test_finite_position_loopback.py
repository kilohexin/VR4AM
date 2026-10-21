import importlib.util
import json
from pathlib import Path
import pytest

ROOT=Path(__file__).resolve().parents[3]


def api():
    path=ROOT/'scripts/finite_position_loopback.py'
    assert path.is_file(),'finite position SDK loopback missing'
    spec=importlib.util.spec_from_file_location('finite_loopback',path)
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m


def test_external_endpoint_overrides_are_not_cli_options():
    m=api()
    for args in (['--ip','172.16.2.162'],['--port','5866'],['--host','0.0.0.0']):
        with pytest.raises(SystemExit):m.parser().parse_args(args)


def test_parser_rejects_unbounded_or_blended_motion_and_other_writes():
    m=api();q=[.0005,-1.,1.,0.,1.57,0.]
    request=dict(jsonrpc='2.0',id=1,method='move_joint',params=[dict(pose=dict(kind='JOINT',joint=dict(joint=q)),
        param=dict(velocity=.025,acc=.05,time=0.,radius=0.))])
    assert m.parse_request(json.dumps(request))['method']=='move_joint'
    for key,value in [('time',.1),('radius',.01),('velocity',1.),('acc',True)]:
        row=json.loads(json.dumps(request));row['params'][0]['param'][key]=value
        with pytest.raises(ValueError):m.parse_request(json.dumps(row))
    request['method']='stop_sys'
    with pytest.raises(ValueError):m.parse_request(json.dumps(request))
