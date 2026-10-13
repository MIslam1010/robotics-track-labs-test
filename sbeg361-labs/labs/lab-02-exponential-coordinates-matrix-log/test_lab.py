"""Auto-checks for SBEG 361 Lab 2.

Usage:   LAB_IMPL=starter  python -m pytest -q test_lab.py     (your work, file starter.py)
         LAB_IMPL=solution python -m pytest -q test_lab.py     (reference; default)
Ground truth: the official `modern_robotics` library (Lynch & Park) and scipy.spatial.transform.
"""
import importlib
import os
import sys
from pathlib import Path

import numpy as np
import pytest
import modern_robotics as mr
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
lab = importlib.import_module(os.environ.get("LAB_IMPL", "starter"))

DATA = HERE / "imu_knee.csv"
RNG = np.random.default_rng(2026)
RANDOM_R = [Rotation.random(random_state=s).as_matrix() for s in range(25)]


def rot(axis, ang):
    return mr.MatrixExp3(mr.VecToso3(np.asarray(axis, float) * ang))


def same_quat(q, p, tol=1e-9):
    """q and -q represent the same rotation (MR App. B.3)."""
    return np.allclose(q, p, atol=tol) or np.allclose(q, -np.asarray(p), atol=tol)


def scipy_quat(R):
    x, y, z, w = Rotation.from_matrix(R).as_quat()
    return np.array([w, x, y, z])


@pytest.fixture(scope="module")
def imu():
    return np.genfromtxt(DATA, delimiter=",", names=True)


def quats(d, prefix):
    return np.column_stack([d[f"{prefix}_q{i}"] for i in range(4)])


def gyro(d, prefix, noisy=False):
    n = "n" if noisy else ""
    return np.column_stack([d[f"{prefix}_{n}w{c}"] for c in "xyz"])[:-1]


# ------------------------------------------------------------- Checkpoint 1: skew matrices
def test_vec_to_so3_matches_mr():
    for _ in range(10):
        w = RNG.normal(size=3)
        np.testing.assert_allclose(lab.vec_to_so3(w), mr.VecToso3(w), atol=1e-12)


def test_skew_properties():
    a, b = RNG.normal(size=3), RNG.normal(size=3)
    A = lab.vec_to_so3(a)
    np.testing.assert_allclose(A, -A.T, atol=1e-12)                       # [a] = -[a]^T
    np.testing.assert_allclose(A @ b, np.cross(a, b), atol=1e-12)        # [a]b = a x b
    R = RANDOM_R[0]
    np.testing.assert_allclose(R @ A @ R.T, lab.vec_to_so3(R @ a), atol=1e-12)  # MR Eq. 3.31
    np.testing.assert_allclose(lab.so3_to_vec(A), a, atol=1e-12)


def test_axis_ang3():
    w, th = lab.axis_ang3([1.0, 2.0, 2.0])
    ref_w, ref_th = mr.AxisAng3(np.array([1.0, 2.0, 2.0]))
    np.testing.assert_allclose(w, ref_w, atol=1e-12)
    assert th == pytest.approx(ref_th)
    with pytest.raises(ValueError):
        lab.axis_ang3([0.0, 0.0, 0.0])


# ------------------------------------------------------------- Checkpoint 2: exp
def test_matrix_exp3_mr_example_3_12():
    """MR Example 3.12: w1 = (0, 0.866, 0.5), theta1 = 30 deg."""
    w = np.array([0.0, np.sqrt(3) / 2, 0.5])
    R = lab.matrix_exp3(lab.vec_to_so3(w * np.pi / 6))
    expected = np.array([[0.866, -0.250, 0.433], [0.250, 0.967, 0.058], [-0.433, 0.058, 0.899]])
    np.testing.assert_allclose(R, expected, atol=1.5e-3)


def test_matrix_exp3_matches_mr_and_is_rotation():
    for _ in range(20):
        so3 = mr.VecToso3(RNG.normal(size=3) * 2.0)
        R = lab.matrix_exp3(so3)
        np.testing.assert_allclose(R, mr.MatrixExp3(so3), atol=1e-12)
        np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-12)
        assert np.linalg.det(R) == pytest.approx(1.0)
    np.testing.assert_allclose(lab.matrix_exp3(np.zeros((3, 3))), np.eye(3))


# ------------------------------------------------------------- Checkpoint 3: log
def test_matrix_log3_matches_mr_random():
    for R in RANDOM_R:
        np.testing.assert_allclose(lab.matrix_log3(R), mr.MatrixLog3(R), atol=1e-9)


def test_matrix_log3_identity_and_pi_cases():
    np.testing.assert_allclose(lab.matrix_log3(np.eye(3)), np.zeros((3, 3)))
    for w in ([0, 0, 1], [0, 1, 0], [1, 0, 0], [1, 1, 0], [1, 0, 1], [0, 1, 1]):
        w = np.array(w, float) / np.linalg.norm(w)
        R = rot(w, np.pi)
        L = lab.matrix_log3(R)
        v = lab.so3_to_vec(L)
        assert np.linalg.norm(v) == pytest.approx(np.pi)
        np.testing.assert_allclose(mr.MatrixExp3(L), R, atol=1e-9)        # either +w or -w is fine


def test_exp_log_round_trip_small_and_large_angles():
    for th in (1e-5, 1e-3, 0.5, 2.0, 3.0, np.pi - 1e-4):
        w = RNG.normal(size=3)
        w /= np.linalg.norm(w)
        R = mr.MatrixExp3(mr.VecToso3(w * th))
        v = lab.so3_to_vec(lab.matrix_log3(R))
        np.testing.assert_allclose(v, w * th, atol=1e-6)


# ------------------------------------------------------------- Checkpoint 4: quaternions
def test_rot_to_quat_matches_scipy():
    for R in RANDOM_R + [rot([1, 1, 1], np.pi), rot([0, 1, 0], np.pi - 1e-7), np.eye(3)]:
        q = lab.rot_to_quat(R)
        assert np.linalg.norm(q) == pytest.approx(1.0)
        assert q[0] >= -1e-12
        assert same_quat(q, scipy_quat(R), tol=1e-7)


def test_quat_to_rot_round_trip():
    for R in RANDOM_R:
        np.testing.assert_allclose(lab.quat_to_rot(lab.rot_to_quat(R)), R, atol=1e-12)
        np.testing.assert_allclose(lab.quat_to_rot(-lab.rot_to_quat(R)), R, atol=1e-12)


def test_quat_mul_is_matrix_product():
    for Ra, Rb in zip(RANDOM_R[:12], RANDOM_R[12:24]):
        q = lab.quat_mul(lab.rot_to_quat(Ra), lab.rot_to_quat(Rb))
        assert same_quat(q, scipy_quat(Ra @ Rb), tol=1e-9)
        qc = lab.quat_conj(lab.rot_to_quat(Ra))
        np.testing.assert_allclose(lab.quat_to_rot(qc), Ra.T, atol=1e-12)


def test_workshop_quaternion_example():
    """Rot(x, 90 deg) Rot(y, 90 deg) -> q = (1, 1, 1, 1)/2, a 120 deg rotation about (1,1,1)/sqrt 3."""
    qx = np.array([np.cos(np.pi / 4), np.sin(np.pi / 4), 0, 0])
    qy = np.array([np.cos(np.pi / 4), 0, np.sin(np.pi / 4), 0])
    np.testing.assert_allclose(lab.quat_mul(qx, qy), 0.5 * np.ones(4), atol=1e-12)


# ------------------------------------------------------------- Checkpoint 5: IMU data
def test_gyro_exponential_integration_reproduces_truth(imu):
    for seg in ("thigh", "shank"):
        q = quats(imu, seg)
        R0 = lab.quat_to_rot(q[0])
        Rs = lab.integrate_gyro_exp(R0, gyro(imu, seg), 0.01)
        assert Rs.shape == (len(q), 3, 3)
        err = max(lab.rotation_angle(Rs[k], Rotation.from_quat(np.r_[q[k, 1:], q[k, 0]]).as_matrix())
                  for k in range(len(q)))
        assert err < 1e-8


def test_euler_integration_drifts_off_SO3(imu):
    q = quats(imu, "thigh")
    Rs = lab.integrate_gyro_euler(lab.quat_to_rot(q[0]), gyro(imu, "thigh"), 0.01)
    assert Rs.shape == (len(q), 3, 3)
    # forward Euler does not preserve orthonormality: det grows above 1
    assert np.linalg.det(Rs[-1]) > 1.01
    Re = lab.integrate_gyro_exp(lab.quat_to_rot(q[0]), gyro(imu, "thigh"), 0.01)
    assert np.linalg.det(Re[-1]) == pytest.approx(1.0, abs=1e-12)


def test_knee_flexion_from_quaternions(imu):
    Rth = [lab.quat_to_rot(q) for q in quats(imu, "thigh")]
    Rsh = [lab.quat_to_rot(q) for q in quats(imu, "shank")]
    kap = np.array([lab.knee_flexion(a, b) for a, b in zip(Rth, Rsh)])
    np.testing.assert_allclose(kap, imu["kappa"], atol=1e-8)


def test_noisy_gyro_drift_is_bounded_but_nonzero(imu):
    """Bias makes integrated orientation drift (reflection question); check the order of magnitude."""
    q = quats(imu, "shank")
    Rs = lab.integrate_gyro_exp(lab.quat_to_rot(q[0]), gyro(imu, "shank", noisy=True), 0.01)
    drift = lab.rotation_angle(Rs[-1], lab.quat_to_rot(q[-1]))
    assert 0.01 < drift < 0.2


# ------------------------------------------------------------- Checkpoint 6: EOD gripper error
def test_orientation_error_eod_gripper():
    """Defence/EOD: gripper wrist must turn from R_cur to R_des; error in the gripper frame."""
    R_cur = rot([0, 0, 1], np.deg2rad(40)) @ rot([0, 1, 0], np.deg2rad(-20))
    R_des = R_cur @ rot([1, 0, 0], np.deg2rad(35))
    e = lab.orientation_error(R_cur, R_des)
    np.testing.assert_allclose(e, [np.deg2rad(35), 0, 0], atol=1e-12)
    for Ra, Rb in zip(RANDOM_R[:10], RANDOM_R[10:20]):
        np.testing.assert_allclose(lab.orientation_error(Ra, Rb),
                                   mr.so3ToVec(mr.MatrixLog3(Ra.T @ Rb)), atol=1e-9)
        assert 0 <= lab.rotation_angle(Ra, Rb) <= np.pi + 1e-12
