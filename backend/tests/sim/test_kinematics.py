import math

import numpy as np
import pytest

from app.sim.kinematics import forward_matrix, forward_pose
from app.sim.lm3_model import LM3Model


# Calculated offline from the binding brief's modified-DH formula, independently
# of the production implementation, then recorded here as fixed golden values.
HOME_MATRIX = np.array(
    [
        [-5.103411967256963e-12, -1.856812081584841e-17, -1.0, -0.5552676618397614],
        [-1.0, 1.2246467991504783e-16, 5.1034119672569625e-12, -0.12062999999911492],
        [1.224646799145176e-16, 1.0, -1.0389655909951191e-16, 0.13164213562376706],
        [0.0, 0.0, 0.0, 1.0],
    ]
)
HOME_POSE = np.array(
    [
        -0.5552676618397614,
        -0.12062999999911492,
        0.13164213562376706,
        -0.499999999998724,
        0.5000000000012759,
        0.5000000000012758,
        -0.49999999999872424,
    ]
)
ASYMMETRIC_Q = (0.2, -0.5, 0.8, -0.2, 0.4, 0.1)
ASYMMETRIC_MATRIX = np.array(
    [
        [0.9609150712441403, -0.19474775728485896, -0.1967626409875285, -0.48479941165090046],
        [-0.20056649509544958, 0.00019035135797173307, -0.9796800726826527, -0.38434570812745567],
        [0.19082795104752426, 0.9808533401069084, -0.03887696361761663, 0.16865270570523702],
        [0.0, 0.0, 0.0, 1.0],
    ]
)
ASYMMETRIC_POSE = np.array(
    [
        -0.48479941165090046,
        -0.38434570812745567,
        0.16865270570523702,
        0.7070363646064792,
        -0.13977861400393868,
        -0.0020984387215031543,
        0.6932222693668487,
    ]
)


def test_lm3_theoretical_constants_match_brief() -> None:
    model = LM3Model()
    assert model.home_q == pytest.approx((0.0, -0.7853981634, 1.5707963268, -0.7853981634, 1.5707963268, 0.0))
    assert model.a_prev_m == pytest.approx((0.0, 0.0, -0.28, -0.26, 0.0, 0.0))
    assert model.alpha_prev_rad == pytest.approx((0.0, math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2))
    assert model.d_m == pytest.approx((0.21583, 0.0, 0.0, 0.12063, 0.09833, 0.08343))
    assert model.tcp_offset_m == pytest.approx((0.0, 0.0, 0.09))
    assert model.joint_window_rad == pytest.approx(math.pi)
    assert model.max_joint_speed_radps == pytest.approx(0.5)
    assert model.max_joint_accel_radps2 == pytest.approx(1.0)


@pytest.mark.parametrize(
    ("field_name", "value", "expected_message"),
    [
        ("home_q", (0.0,) * 5, "home_q must contain exactly six finite values"),
        ("home_q", (0.0,) * 5 + (math.nan,), "home_q must contain exactly six finite values"),
        ("a_prev_m", (0.0,) * 7, "a_prev_m must contain exactly six finite values"),
        ("a_prev_m", (0.0,) * 5 + (math.inf,), "a_prev_m must contain exactly six finite values"),
        ("alpha_prev_rad", (0.0,) * 5, "alpha_prev_rad must contain exactly six finite values"),
        (
            "alpha_prev_rad",
            (0.0,) * 5 + (-math.inf,),
            "alpha_prev_rad must contain exactly six finite values",
        ),
        ("d_m", (0.0,) * 7, "d_m must contain exactly six finite values"),
        ("d_m", (0.0,) * 5 + (math.nan,), "d_m must contain exactly six finite values"),
        ("tcp_offset_m", (0.0,) * 2, "tcp_offset_m must contain exactly three finite values"),
        ("tcp_offset_m", (0.0,) * 4, "tcp_offset_m must contain exactly three finite values"),
        (
            "tcp_offset_m",
            (0.0, 0.0, math.inf),
            "tcp_offset_m must contain exactly three finite values",
        ),
    ],
)
def test_lm3_model_rejects_malformed_vectors(
    field_name: str, value: tuple[float, ...], expected_message: str
) -> None:
    with pytest.raises(ValueError, match=expected_message):
        LM3Model(**{field_name: value})


@pytest.mark.parametrize(
    ("q", "expected_matrix", "expected_pose"),
    [
        (LM3Model().home_q, HOME_MATRIX, HOME_POSE),
        (ASYMMETRIC_Q, ASYMMETRIC_MATRIX, ASYMMETRIC_POSE),
    ],
    ids=["home", "asymmetric"],
)
def test_fk_matches_offline_golden_fixtures(
    q: tuple[float, ...], expected_matrix: np.ndarray, expected_pose: np.ndarray
) -> None:
    model = LM3Model()
    matrix = forward_matrix(q, model)
    pose = forward_pose(q, model)
    np.testing.assert_allclose(matrix, expected_matrix, rtol=1e-12, atol=1e-12)
    np.testing.assert_allclose((*pose.p, *pose.q), expected_pose, rtol=1e-12, atol=1e-12)


def test_fk_returns_rigid_transform_and_pose() -> None:
    model = LM3Model()
    matrix = forward_matrix(model.home_q, model)
    pose = forward_pose(model.home_q, model)
    assert matrix.shape == (4, 4)
    assert np.allclose(matrix[3], [0, 0, 0, 1])
    assert np.linalg.det(matrix[:3, :3]) == pytest.approx(1.0)
    assert sum(value * value for value in pose.q) == pytest.approx(1.0)


def test_fk_rejects_wrong_joint_count() -> None:
    with pytest.raises(ValueError, match="six joints"):
        forward_pose([0, 0], LM3Model())


@pytest.mark.parametrize("non_finite", [math.nan, math.inf, -math.inf], ids=["nan", "pos_inf", "neg_inf"])
def test_fk_rejects_non_finite_joints_before_trigonometry(
    non_finite: float, monkeypatch: pytest.MonkeyPatch
) -> None:
    q = [0.0, -0.5, non_finite, -0.2, 0.4, 0.1]

    def unexpected_trigonometry(_: float) -> float:
        raise AssertionError("trigonometry must not run for invalid joints")

    monkeypatch.setattr(np, "cos", unexpected_trigonometry)
    monkeypatch.setattr(np, "sin", unexpected_trigonometry)
    with pytest.raises(ValueError, match="joint values must be finite"):
        forward_matrix(q, LM3Model())


def test_fk_is_repeatable() -> None:
    q = [0.2, -0.5, 0.8, -0.2, 0.4, 0.1]
    assert forward_pose(q, LM3Model()) == forward_pose(q, LM3Model())
