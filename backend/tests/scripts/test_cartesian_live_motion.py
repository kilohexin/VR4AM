import re

import pytest

from scripts import check_cartesian_live_motion as motion
from app.control.cartesian_pvat_reference import JointReference


def test_motion_source_has_finite_sdk_only_actuation_and_single_exception_stop():
    source=motion.program(JointReference((0.,-.8,1.4,-.6,1.57,0.)),'01'*16,'a9'*32)
    assert source.count('lebai:move_pvat(')==1 and source.count('lebai:stop_move()')==1
    assert not re.findall(r'(?<![:.\w])\b(?:move_pvat|movej|movel|start_sys|stop_sys|set_claw)\s*\(',source)
    assert 'actual_entries<128' in source and 'live_actual_fixed_region' in source
    assert 'live_actual_cartesian_witness' in source and 'live_actual_terminal' in source
    assert 'dry_joint_changed' not in source and 'LIVE_EXTRA_' not in source
    assert 'sdk_result.call_void' in source and 'wait_all(main_task,journal_worker)' in source
    assert 'local main_task=async_task(function()' in source
    assert 'wait_all(receiver,sender,clock_service,actual_monitor)' in source
    assert "pcall(set_signal,254,1)" in source
    assert 'CART_ACTUAL_REJECT_RANGE' in source and 'CART_ACTUAL_KIN' in source
    assert 'target_distance<=.001' in source and 'actual_distance<=.002' in source
    assert 'curve_witness.closest' in source and 'actual_model.pose(found.state.q)' in source
    assert 'actual_admit_model()' in source and 'live_actual_model_mismatch' in source


def evidence(entries=1,returns=1,abort=False):
    rows=[['CART_ACTUAL_ENTER',str(i),'200','1','following'] for i in range(1,entries+1)]
    rows.extend(['CART_ACTUAL_RETURNED',str(i),'201','1','following'] for i in range(1,returns+1))
    rows.append(['CART_ACTUAL_STATUS',str(entries),str(returns)])
    if abort:rows.append(['FINITE_ABORT','true','true'])
    return dict(passed=False,raw_output=rows,unknown=[])


def test_unknown_motion_is_never_cleared_by_successful_exception_stop():
    r=motion.analyze(evidence(1,0,True))
    assert r['stop_unverified'] and r['unknown_motion_write'] and not r['passed']


def test_missing_quiet_evidence_remains_unconfirmed_even_when_every_write_returned():
    r=motion.analyze(evidence())
    assert not r['unknown_motion_write'] and r['stop_unverified'] and not r['passed']


def test_zero_command_rejection_does_not_invent_a_motion_stop_obligation():
    r=motion.analyze(evidence(0,0))
    assert not r['passed'] and not r['unknown_motion_write'] and not r['stop_unverified']


def test_preview_has_no_execute_path(monkeypatch,tmp_path,capsys):
    monkeypatch.setattr(motion.sys,'argv',['motion','--container','virtual','--output',str(tmp_path/'unused.json')])
    monkeypatch.setattr(motion,'native_admission',lambda _:dict(container_id='virtual'))
    monkeypatch.setattr(motion.live,'execute',lambda *a,**k:pytest.fail('preview must not execute'))
    assert motion.main()==0 and '"execute": false' in capsys.readouterr().out
