import numpy as np
import pytest

from scripts.analyze_cartesian_native_feedback import witness,analyze
from app.control.cartesian_pvat_reference import JointReference,PvatSegment,_evaluate


def test_explicit_witness_is_on_the_assumed_curve_not_an_interpolated_sample_table():
    a=JointReference((0.,)*6)
    b=JointReference((.01,.02,-.03,.04,-.02,.005),(.01,-.02,.01,.03,.02,.001),(.1,.02,-.01,.03,.01,-.01))
    coefficients=PvatSegment(a,b,.08).coefficients();target=_evaluate(coefficients,.371923)
    u,q=witness(coefficients,target)
    assert 0<=u<=1
    np.testing.assert_allclose(q,_evaluate(coefficients,u),atol=0,rtol=0)
    assert np.linalg.norm(q-target)<1e-10


def test_witness_cannot_remove_a_perpendicular_actual_deviation():
    a=JointReference((0.,)*6);b=JointReference((.01,0.,0.,0.,0.,0.))
    c=PvatSegment(a,b,.08).coefficients();target=_evaluate(c,.4);target[2]+=.001
    _,q=witness(c,target)
    assert np.linalg.norm(q-target)>=.001


def test_zero_tool_docker_witness_analysis_rejects_physical_or_unbound_sources():
    with pytest.raises(ValueError,match='isolated_docker'):analyze({'raw_output':[]})


def test_a_nonzero_tool_report_cannot_be_analyzed_as_zero_tool():
    from app.control.cartesian_tool_binding import tool_digest
    from app.commissioning.native_jog_mdh import MdhTool
    tool=MdhTool((.03,-.02,.07),(0.,0.,0.,1.))
    with pytest.raises(ValueError,match='tool_binding_mismatch'):
        analyze({'tool_sha256':tool_digest(tool)})


def test_intermediate_host_report_tool_binding_is_also_enforced():
    from app.control.cartesian_tool_binding import ZERO_TOOL,tool_digest
    from app.commissioning.native_jog_mdh import MdhTool
    tool=MdhTool((.03,-.02,.07),(0.,0.,0.,1.))
    for report in ({'statistics':{'tool_sha256':tool_digest(tool)}},
            {'tool_sha256':tool_digest(ZERO_TOOL),'statistics':{'tool_sha256':tool_digest(tool)}}):
        with pytest.raises(ValueError,match='tool_binding_mismatch'):analyze(report)
    with pytest.raises(ValueError,match='tool_binding_mismatch'):analyze({},tool)
