import numpy as np
import pytest

from app.control.canonical_joint_quintic import coefficients,evaluate
from app.control.cartesian_pvat_reference import JointReference,PvatSegment


def test_ordered_scalar_evaluation_matches_independent_boundary_value_polynomial():
    rng=np.random.default_rng(507)
    for duration in (.08,.8,1.28,4.8):
        first=JointReference(tuple(rng.uniform(-2,2,6)),tuple(rng.uniform(-.2,.2,6)),tuple(rng.uniform(-.2,.2,6)))
        last=JointReference(tuple(rng.uniform(-2,2,6)),tuple(rng.uniform(-.2,.2,6)),tuple(rng.uniform(-.2,.2,6)))
        c=coefficients(first,last,duration);independent=PvatSegment(first,last,duration)
        for time in np.linspace(0,duration,31):
            q,v,a=evaluate(c,float(time),duration);expected=independent.state(float(time))
            np.testing.assert_allclose(q,expected.q,rtol=1e-11,atol=1e-11)
            np.testing.assert_allclose(v,expected.v,rtol=1e-10,atol=1e-10)
            np.testing.assert_allclose(a,expected.a,rtol=1e-9,atol=1e-9)


@pytest.mark.parametrize('duration',[0.,-1.,float('inf'),float('nan'),True])
def test_invalid_duration_is_not_a_polynomial(duration):
    state=JointReference((0.,)*6)
    with pytest.raises(ValueError):coefficients(state,state,duration)
