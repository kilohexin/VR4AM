import re

import pytest
from scripts import check_native_bundle_mailbox as probe


def evidence():
    return '\n'.join(['BUNDLE_CHECK\t'+name for name in sorted(probe.CHECKS)]+[
        'BUNDLE_NATIVE_READ\t23\t61','BUNDLE_NATIVE_READ\t24\t60','BUNDLE_MAILBOX_DONE\ttrue\tnil'])


def test_native_program_can_only_access_signals_and_compare_before_after_joints():
    source=probe.program()
    assert set(re.findall(r'lebai:(\w+)\(',source))=={
        'get_kin_data','get_signal','get_signals','set_signal','set_signals'}
    assert 'disable_auto_sync()' in source and 'joint_changed' in source
    assert '__MODULE__' not in source


@pytest.mark.parametrize('change',[
    lambda s:s.replace('DONE\ttrue','DONE\tfalse'),
    lambda s:s.replace('BUNDLE_CHECK\tlossless','MISSING\tlossless'),
    lambda s:s.replace('\t61','\t80'),
    lambda s:s.replace('\t61','\tnan'),
    lambda s:s.replace('READ\t24','READ\t23'),
    lambda s:s+'\nBUNDLE_MAILBOX_DONE\ttrue\tnil',
])
def test_incomplete_or_bad_native_evidence_is_not_success(change):
    assert not probe.analyze(change(evidence()))['passed']


def test_native_success_still_cannot_claim_motion_or_product_admission():
    result=probe.analyze(evidence())
    assert result['passed'] and result['max_read_ms']==61
    assert result['motion_calls']==0 and not result['product_admission_complete']


def test_default_preview_never_runs_native_signal_io(monkeypatch,tmp_path,capsys):
    monkeypatch.setattr(probe.sys,'argv',['probe','--container','isolated','--output',str(tmp_path/'unused.json')])
    monkeypatch.setattr(probe,'native_admission',lambda _:dict(container_id='isolated'))
    monkeypatch.setattr(probe,'execute',lambda _:pytest.fail('preview has no signal writes'))
    assert probe.main()==0 and '"execute": false' in capsys.readouterr().out
