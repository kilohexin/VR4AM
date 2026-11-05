import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from app.commissioning.native_jog_mdh import MdhTool
from app.control.cartesian_pvat_reference import MdhReferenceModel
from app.control.compiled_reference_geometry import CompiledReferenceModel


def test_missing_acceleration_is_explicitly_rejected(monkeypatch):
    monkeypatch.setitem(sys.modules,'numba',None)
    with pytest.raises(ValueError,match='acceleration_unavailable'):
        CompiledReferenceModel(MdhTool((0.,0.,0.),(0.,0.,0.,1.)),(-3.,)*6,(3.,)*6)


def test_strict_rotation_conversion_matches_oracle_near_all_branches(models):
    _,compiled=models
    rng=np.random.default_rng(5511)
    rotations=[Rotation.from_rotvec(np.zeros(3))]
    rotations += [Rotation.from_rotvec(np.eye(3)[i]*angle) for i in range(3) for angle in (1e-12,np.pi-1e-12,np.pi)]
    rotations += list(Rotation.random(100,random_state=rng))
    for original in rotations:
        actual=compiled.rotation(original.as_matrix())
        assert (actual.inv()*original).magnitude()<2e-14


@pytest.mark.parametrize('matrix',[np.eye(2),np.diag([1.,1.,-1.]),np.diag([1.,1.,1.001]),
    np.full((3,3),float('nan')),np.zeros((3,3))])
def test_strict_rotation_rejects_invalid_geometry_instead_of_repairing_it(models,matrix):
    _,compiled=models
    with pytest.raises(ValueError,match='rotation'):compiled.rotation(matrix)


@pytest.fixture(scope='module',params=[
    MdhTool((0.,0.,0.),(0.,0.,0.,1.)),
    MdhTool((.03,-.02,.07),tuple(float(x) for x in Rotation.from_euler('xyz',(.2,-.4,.3)).as_quat())),
])
def models(request):
    pytest.importorskip('numba')
    return (MdhReferenceModel(request.param,(-3.,)*6,(3.,)*6),
        CompiledReferenceModel(request.param,(-3.,)*6,(3.,)*6))


def test_compiled_geometry_and_derivatives_match_independent_python_oracle(models):
    plain,compiled=models;rng=np.random.default_rng(20261105)
    for _ in range(100):
        q=rng.uniform(-2.9,2.9,6);qd=rng.uniform(-.3,.3,6)
        expected=plain.geometry(q,qd);actual=compiled.geometry(q,qd)
        for a,b in zip(expected,actual):assert a==pytest.approx(b,abs=2e-12,rel=2e-12)
        for a,b in zip(plain.geometry(q),compiled.geometry(q)):assert a==pytest.approx(b,abs=2e-12,rel=2e-12)
    assert compiled._kernel.targetoptions['fastmath'] is False
    assert compiled._kernel.targetoptions['parallel'] is False


def test_compiled_jacobian_rate_matches_independent_pose_differences(models):
    plain,compiled=models;q=np.array([.4,-.8,1.4,-.6,1.2,.3]);v=np.array([.1,-.15,.2,.03,-.1,.07]);h=1e-6
    _,_,jd=compiled.geometry(q,v)
    difference=(plain.geometry(q+v*h)[1]-plain.geometry(q-v*h)[1])/(2*h)
    assert jd==pytest.approx(difference,abs=2e-9)


def test_nonfinite_shape_and_model_changes_never_use_cached_state(models):
    plain,compiled=models
    with pytest.raises(ValueError,match='nonfinite_or_shape'):compiled.geometry((float('nan'),)*6)
    with pytest.raises(ValueError,match='nonfinite_or_shape'):compiled.geometry((0.,)*5)
    with pytest.raises(ValueError,match='nonfinite_or_shape'):compiled.geometry((0.,)*6,(float('inf'),)*6)
    first=compiled.geometry((0.,)*6);first[0][:]=100;first[1][:]=100
    for a,b in zip(compiled.geometry((0.,)*6),plain.geometry((0.,)*6)):assert a==pytest.approx(b,abs=2e-12)
    original=compiled.tool
    try:
        compiled.tool=MdhTool((.01,0.,0.),(0.,0.,0.,1.))
        with pytest.raises(ValueError,match='model_changed'):compiled.geometry((0.,)*6)
    finally:compiled.tool=original
