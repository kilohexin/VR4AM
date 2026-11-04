import re

import pytest
from scripts import check_native_bundle_owner as probe


def evidence():
    return '\n'.join(['OWNER_CHECK\t'+name for name in sorted(probe.CHECKS)]+['OWNER_PROBE_DONE\ttrue\tnil'])


def test_compact_fixtures_are_actual_model_approved_complete_tail_bundles():
    packets=probe.fixtures()
    assert len(packets)==3 and all(len(p.expand_tail())==p.braking_ticks for p in packets)
    assert all(a.lead.end==b.lead.start for a,b in zip(packets,packets[1:]))


def test_owner_contract_probe_cannot_actuate_or_write_native_signals():
    source=probe.program()
    assert set(re.findall(r'lebai:(\w+)\(',source))=={'get_kin_data','kinematics_forward'}
    assert 'TEST validator' in source and 'Install the FULL backup BEFORE' in source
    assert '__OWNER__' not in source


@pytest.mark.parametrize('change',[
    lambda s:s.replace('DONE\ttrue','DONE\tfalse'),
    lambda s:s.replace('OWNER_CHECK\tprecise_tail_reconstruction','MISSING'),
    lambda s:s+'\nOWNER_PROBE_DONE\ttrue\tnil',
])
def test_bad_native_contract_evidence_cannot_pass(change):
    assert not probe.analyze(change(evidence()))['passed']


def test_success_is_only_a_zero_motion_ownership_contract():
    result=probe.analyze(evidence())
    assert result['passed'] and result['motion_calls']==0
    assert not result['product_admission_complete']
