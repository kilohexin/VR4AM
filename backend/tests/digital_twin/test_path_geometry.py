"""Vectorized local-demo geometry must match the existing scalar checks."""
import numpy as np
import pytest


def test_segment_batch_matches_scalar_for_random_degenerate_and_parallel_segments():
    from app.digital_twin.path_geometry import segment_distances
    from app.sim.self_collision import segment_distance
    rng=np.random.default_rng(42)
    a0,a1,b0,b1=rng.normal(size=(4,512,3))
    a1[:30]=a0[:30]
    b1[20:50]=b0[20:50]
    b1[50:80]=b0[50:80]+a1[50:80]-a0[50:80]
    a1[80:90]=a0[80:90]+1e-10
    b1[85:95]=b0[85:95]+1e-10
    expected=[segment_distance(*p) for p in zip(a0,a1,b0,b1)]
    assert segment_distances(a0,a1,b0,b1)==pytest.approx(expected,abs=1e-12)


def test_segment_batch_rejects_nonfinite_geometry():
    from app.digital_twin.path_geometry import segment_distances
    points=np.zeros((2,3));bad=points.copy();bad[0,0]=np.nan
    with pytest.raises(ValueError,match='invalid_self_collision_geometry'):
        segment_distances(points,points,bad,points)


@pytest.mark.parametrize('count',[1,65])
def test_batch_fk_matches_scalar_joint_origins_and_tcp_for_every_sample(count):
    from app.digital_twin.path_geometry import chain_points_batch
    from app.sim.kinematics import chain_points
    from app.sim.lm3_model import LM3Model
    model=LM3Model();rng=np.random.default_rng(27)
    samples=np.array(model.home_q)+rng.uniform(-1.,1.,(count,6))
    actual=chain_points_batch(samples,model)
    expected=[chain_points(row,model) for row in samples]
    assert tuple(actual)==tuple(expected[0])
    for name in actual:
        np.testing.assert_allclose(actual[name],np.stack([row[name] for row in expected]),atol=1e-13,rtol=1e-13)


@pytest.mark.parametrize('samples',[np.zeros((0,6)),np.zeros((2,5)),np.zeros(6),np.full((1,6),np.nan)])
def test_batch_fk_rejects_empty_wrong_shape_and_nonfinite_samples(samples):
    from app.digital_twin.path_geometry import chain_points_batch
    from app.sim.lm3_model import LM3Model
    with pytest.raises(ValueError,match='invalid_ready_path_samples'):
        chain_points_batch(samples,LM3Model())


def test_ready_path_agrees_with_original_scalar_decisions():
    from dataclasses import replace
    from app.digital_twin.path_geometry import validate_ready_samples
    from app.digital_twin.segmented_demo import demo_settings
    from app.sim.kinematics import forward_pose
    from app.sim.self_collision import is_self_colliding
    from app.sim.lm3_model import LM3Model
    settings=demo_settings().lebai;model=LM3Model();rng=np.random.default_rng(7)
    cases=[(settings.home_q,settings.teleop_ready_q,settings)]
    for _ in range(20):
        start=np.array(settings.teleop_ready_q)+rng.uniform(-.35,.35,6)
        goal=np.array(settings.teleop_ready_q)+rng.uniform(-.35,.35,6)
        cases.append((start,goal,settings))
    cases.extend((settings.home_q,settings.teleop_ready_q,changed) for changed in (
        replace(settings,startup_tcp_min_m=(10.,10.,10.)),
        replace(settings,soft_joint_max_rad=(-10.,)*6)))
    for start,goal,configuration in cases:
        accepted=True
        for fraction in np.linspace(0.,1.,65):
            q=np.array(start)+(np.array(goal)-start)*fraction
            p=np.array(forward_pose(q,model).p)
            if (np.any(q<np.array(configuration.soft_joint_min_rad)+configuration.joint_limit_margin_rad)
                    or np.any(q>np.array(configuration.soft_joint_max_rad)-configuration.joint_limit_margin_rad)
                    or np.any(abs(q-np.array(model.home_q))>model.joint_window_rad)
                    or is_self_colliding(q,model)
                    or np.any(p<np.array(configuration.startup_tcp_min_m))
                    or np.any(p>np.array(configuration.startup_tcp_max_m))):
                accepted=False;break
        if accepted:validate_ready_samples(start,goal,configuration,model)
        else:
            with pytest.raises(ValueError,match='demo_ready_path_invalid'):
                validate_ready_samples(start,goal,configuration,model)


def test_ready_validation_checks_all_65_samples_and_collision_pairs(monkeypatch):
    from app.digital_twin import segmented_demo as demo
    from app.digital_twin import path_geometry as geometry
    from app.sim.lm3_model import LM3Model
    settings=demo.demo_settings().lebai;model=LM3Model()
    demo._validate_ready_path.cache_clear()
    original=geometry.chain_points_batch;seen=[]
    def capture(q,m):seen.extend(map(tuple,q));return original(q,m)
    monkeypatch.setattr(geometry,'chain_points_batch',capture)
    demo._validate_ready_path(settings.home_q,settings.teleop_ready_q,settings,model)
    assert len(seen)==65
    assert seen[0]==settings.home_q and seen[-1]==settings.teleop_ready_q
    demo._validate_ready_path.cache_clear()
    monkeypatch.setattr(geometry,'segment_distances',lambda *args:np.zeros(65))
    with pytest.raises(ValueError,match='demo_ready_path_invalid'):
        demo._validate_ready_path(settings.home_q,settings.teleop_ready_q,settings,model)
    demo._validate_ready_path.cache_clear()
