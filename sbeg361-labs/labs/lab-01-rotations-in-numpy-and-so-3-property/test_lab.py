"""Auto-checks for SBEG 361 Lab 1 (rotations, SO(3)).

Run from this folder:
    LAB_IMPL=solution pytest -q      # reference: must pass
    LAB_IMPL=starter  pytest -q      # your work (default module name is 'solution')
Results are compared with the official modern_robotics library wherever it has an equivalent.
"""
import importlib
import os

import numpy as np
import pytest
import modern_robotics as mr

lab = importlib.import_module(os.environ.get("LAB_IMPL", "starter"))
HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "data", "tracker_orientations.csv")
CORRUPTED = {4, 9, 13, 17}
ANGLES = [0.0, 0.3, -1.2, np.pi / 2, np.pi, 2.7]


def mr_rot(axis, theta):
    """Reference rotation from modern_robotics: exp([w_hat] theta)."""
    return mr.MatrixExp3(mr.VecToso3(np.asarray(axis, float) * theta))


# ---------------- Part A ----------------
@pytest.mark.parametrize("theta", ANGLES)
def test_elementary_rotations_match_mr(theta):
    np.testing.assert_allclose(lab.rot_x(theta), mr_rot([1, 0, 0], theta), atol=1e-12)
    np.testing.assert_allclose(lab.rot_y(theta), mr_rot([0, 1, 0], theta), atol=1e-12)
    np.testing.assert_allclose(lab.rot_z(theta), mr_rot([0, 0, 1], theta), atol=1e-12)


def test_axis_angle_matches_mr():
    rng = np.random.default_rng(2)
    for _ in range(25):
        w = rng.normal(size=3)
        w /= np.linalg.norm(w)
        th = rng.uniform(-np.pi, np.pi)
        np.testing.assert_allclose(lab.rot_axis_angle(w, th), mr_rot(w, th), atol=1e-12)


def test_axis_angle_identities():
    w = np.array([1.0, 2.0, 2.0]) / 3.0
    np.testing.assert_allclose(lab.rot_axis_angle(w, 0.7), lab.rot_axis_angle(-w, -0.7), atol=1e-12)  # MR p. 72
    np.testing.assert_allclose(lab.rot_axis_angle(w, 0.0), np.eye(3), atol=1e-12)
    np.testing.assert_allclose(lab.rot_axis_angle(w, 0.9) @ w, w, atol=1e-12)  # axis is unchanged


def test_axis_angle_rejects_non_unit():
    with pytest.raises(ValueError):
        lab.rot_axis_angle([1.0, 1.0, 0.0], 0.5)


# ---------------- Part B ----------------
def test_so3_errors_values():
    e, d = lab.so3_errors(np.eye(3))
    assert e == pytest.approx(0.0) and d == pytest.approx(1.0)
    e, d = lab.so3_errors(2.0 * np.eye(3))
    assert e == pytest.approx(np.sqrt(27.0)) and d == pytest.approx(8.0)
    with pytest.raises(ValueError):
        lab.so3_errors(np.eye(2))


def test_is_rotation_cases():
    Rc = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]], float)  # MR Fig. 3.7
    swap = np.array([[0, 1, 0], [1, 0, 0], [0, 0, 1]], float)  # det = -1
    assert lab.is_rotation(Rc) and mr.TestIfSO3(Rc)
    assert not lab.is_rotation(swap) and not mr.TestIfSO3(swap)
    assert not lab.is_rotation(1.05 * Rc)
    assert not lab.is_rotation(np.eye(2))
    mr_example = np.array([[1.0, 0.0, 0.0], [0.0, 0.1, -0.95], [0.0, 1.0, 0.1]])  # MR library docstring
    assert not lab.is_rotation(mr_example) and not mr.TestIfSO3(mr_example)


def test_group_properties():
    """Closure, inverse = transpose, non-commutativity, length preservation (MR Props. 3.3-3.6)."""
    rng = np.random.default_rng(5)
    for _ in range(20):
        A = mr_rot(rng.normal(size=3) / 1.7, 1.0)
        B = mr_rot(rng.normal(size=3) / 1.7, 1.0)
        assert lab.is_rotation(A @ B)
        np.testing.assert_allclose(lab.rot_inv(A), mr.RotInv(A), atol=1e-12)
        np.testing.assert_allclose(lab.rot_inv(A) @ A, np.eye(3), atol=1e-12)
        x = rng.normal(size=3)
        assert np.linalg.norm(A @ x) == pytest.approx(np.linalg.norm(x))
    X, Y = lab.rot_x(np.pi / 2), lab.rot_y(np.pi / 2)
    assert not np.allclose(X @ Y, Y @ X)
    Z1, Z2 = lab.rot_z(0.4), lab.rot_z(1.1)
    np.testing.assert_allclose(Z1 @ Z2, Z2 @ Z1, atol=1e-12)  # same-axis (planar) rotations commute


def test_project_to_so3():
    M = np.array([[0.675, 0.150, 0.720], [0.370, 0.771, -0.511], [-0.630, 0.619, 0.472]])
    np.testing.assert_allclose(lab.project_to_so3(M), mr.ProjectToSO3(M), atol=1e-8)
    reflect = np.diag([1.0, 1.0, -1.0])
    R = lab.project_to_so3(reflect)
    assert lab.is_rotation(R)


# ---------------- Part C ----------------
def test_mr_figure_3_7():
    Rb = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], float)
    Rc = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]], float)
    pa = np.array([1.0, 1.0, 0.0])
    np.testing.assert_allclose(lab.change_frame(lab.rot_inv(Rb), pa), [1, -1, 0], atol=1e-12)
    np.testing.assert_allclose(lab.change_frame(lab.rot_inv(Rc), pa), [0, -1, -1], atol=1e-12)
    np.testing.assert_allclose(Rb, lab.rot_z(np.pi / 2), atol=1e-12)
    np.testing.assert_allclose(lab.rotate_in_body(Rb, lab.rot_y(-np.pi / 2)), Rc, atol=1e-12)
    np.testing.assert_allclose(lab.rotate_in_fixed(Rb, lab.rot_x(np.pi / 2)), Rc, atol=1e-12)


def test_eod_base_pan_and_wrist():
    """Defence checkpoint: EOD robot gripper, base yaw (fixed z_s) vs wrist roll (body x_b)."""
    R_sb = lab.rot_z(np.pi / 6) @ lab.rot_y(np.pi / 4)
    pan = lab.rot_z(np.pi / 2)
    roll = lab.rot_x(np.pi / 3)
    np.testing.assert_allclose(lab.rotate_in_fixed(R_sb, pan), pan @ R_sb, atol=1e-12)
    np.testing.assert_allclose(lab.rotate_in_body(R_sb, roll), R_sb @ roll, atol=1e-12)
    # roll about the gripper's own x axis leaves that axis (first column) unchanged
    np.testing.assert_allclose(lab.rotate_in_body(R_sb, roll)[:, 0], R_sb[:, 0], atol=1e-12)


def test_planar_compose():
    P = lab.rot_z(np.pi / 3)[:2, :2]
    Q = lab.rot_z(np.pi / 6)[:2, :2]
    R, r = lab.planar_compose(P, [2.0, 1.0], Q, [1.0, 0.5])
    np.testing.assert_allclose(R, [[0, -1], [1, 0]], atol=1e-12)
    np.testing.assert_allclose(r, [2.0 + 0.5 - np.sqrt(3) / 4, 1.0 + np.sqrt(3) / 2 + 0.25], atol=1e-12)


# ---------------- Part D ----------------
def test_tracker_classification():
    Rs = lab.load_tracker_csv(DATA)
    assert Rs.shape == (20, 3, 3)
    ok = lab.classify_samples(Rs)
    assert set(np.flatnonzero(~np.asarray(ok))) == CORRUPTED
    ref = np.array([mr.TestIfSO3(R) for R in Rs])
    assert np.array_equal(np.asarray(ok, bool), ref)


def test_tracker_projection_and_tool():
    Rs = lab.load_tracker_csv(DATA)
    ok = np.asarray(lab.classify_samples(Rs), bool)
    for R in Rs[ok]:
        Rp = lab.project_to_so3(R)
        np.testing.assert_allclose(Rp, mr.ProjectToSO3(R), atol=1e-9)
        assert lab.so3_errors(Rp)[0] < 1e-12
    R_mt = lab.rot_x(np.pi / 6)  # marker-to-tip calibration (illustrative)
    R_st = lab.tool_orientations(Rs[ok], R_mt)
    assert R_st.shape == (ok.sum(), 3, 3)
    np.testing.assert_allclose(R_st[0], Rs[ok][0] @ R_mt, atol=1e-12)
