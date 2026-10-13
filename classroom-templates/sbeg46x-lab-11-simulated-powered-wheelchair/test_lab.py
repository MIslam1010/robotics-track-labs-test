"""Auto-checks for SBEG 46X Lab 11 (simulated powered wheelchair: odometry, EKF landmark localisation, tracking control).

    LAB_IMPL=starter   python -m pytest -q test_lab.py   # untouched starter: every test that calls your code fails
                                                         # with NotImplementedError; the 4 data tests pass
    LAB_IMPL=lab11_<ID> python -m pytest -q test_lab.py  # your renamed file

LAB_IMPL is the name of the module to test; it is imported from this folder.
Ground truth: the wheel models of Modern Robotics (H(0) of MR Eqs. 13.9 and 13.10, F of MR Eq. 13.34), the matrix
exponential of the chassis twist (modern_robotics MatrixExp6, MR Sec. 13.4), finite differences of the odometry and
measurement models and of the mobile-manipulator pose, closed-form cases, the course model srs_7r_feeding
(course_models.py), and reference values in data/expected.json.
"""
import importlib
import json
import math
import os
import sys
from pathlib import Path

import modern_robotics as mr
import numpy as np
import pytest
from scipy.linalg import expm

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
lab = importlib.import_module(os.environ.get("LAB_IMPL", "starter"))
import course_models as cm  # noqa: E402

P = json.loads((HERE / "data" / "wheelchair.json").read_text())
EXP = json.loads((HERE / "data" / "expected.json").read_text())
CH = P["chair"]
r, d = CH["r"], CH["d"]
RNG = np.random.default_rng(1112)
Rm = np.diag([P["ekf"]["sigma_range"] ** 2, math.radians(P["ekf"]["sigma_bearing_deg"]) ** 2])


def wrapped(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def exp_odometry(q, Vb):
    """Independent odometry: T_sb' = T_sb e^[Vb6] (MR Sec. 13.4), matrix exponential by scipy.linalg.expm
    (modern_robotics MatrixExp6 treats |omega| < 1e-6 as zero, too coarse for finite differences)."""
    w, vx, vy = Vb
    Tsb = mr.RpToTrans(mr.MatrixExp3(mr.VecToso3([0, 0, q[0]])), [q[1], q[2], 0])
    T = Tsb @ expm(mr.VecTose3([0, 0, w, vx, vy, 0]))
    return np.array([q[0] + w, T[0, 3], T[1, 3]])


# ------------------------------------------------------------------------------ data and provided code (pass)
def test_data_files_consistent():
    log, meas = lab.load_log(), lab.load_meas()
    assert len(log["t"]) == EXP["N"] and len(meas) == EXP["n_meas"]
    assert np.all(np.diff(log["t"]) > 0)
    assert all(z["k"] % P["sensor"]["every"] == 0 for z in meas)


def test_reference_route_closes():
    ref = lab.reference_trajectory(P)
    lap = sum(s[1] if s[0] == "line" else s[1] * math.radians(s[2]) for s in P["route"]["segments"])
    assert abs(lap - (20.0 + 3 * math.pi)) < 1e-9
    assert np.allclose(ref["qd"][-1, 1:], [0, 0], atol=P["route"]["v"] * CH["h"])
    assert abs(ref["qd"][-1, 0] - 4 * math.pi) < P["route"]["v"] * CH["h"] / 1.5   # last sample is on the last arc


def test_log_ground_truth_moves_like_a_diff_drive():
    # the true chair obeys the nonholonomic constraint xdot sin(phi) - ydot cos(phi) = 0 (MR p. 528) to O(h^2)
    log = lab.load_log()
    dx, dy = np.diff(log["x"]), np.diff(log["y"])
    phm = 0.5 * (log["phi"][1:] + log["phi"][:-1])
    assert np.max(np.abs(dx * np.sin(phm) - dy * np.cos(phm))) < 1e-5


def test_wrap_and_landmarks():
    assert abs(lab.wrap(3 * math.pi / 2) + math.pi / 2) < 1e-12
    assert set(lab.landmark_table(P)) == set(range(1, 9))


# ------------------------------------------------------------------------------ Part A: kinematics and odometry
def test_wheel_row_three_omni_mr_eq_13_9():
    dd, rr = 0.20, 0.05
    s3 = math.sin(math.pi / 3)
    wheels = [(rr, 0.0, 0.0, 0.0, dd),
              (rr, 0.0, -2 * math.pi / 3, dd * math.cos(-math.pi / 6), dd * math.sin(-math.pi / 6)),
              (rr, 0.0, 2 * math.pi / 3, -dd * math.cos(-math.pi / 6), dd * math.sin(-math.pi / 6))]
    H = lab.H0_matrix(wheels)
    H_mr = np.array([[-dd, 1, 0], [-dd, -0.5, -s3], [-dd, -0.5, s3]]) / rr
    assert np.allclose(H, H_mr, atol=1e-12)
    assert np.allclose(lab.wheel_row(*wheels[1]), H_mr[1], atol=1e-12)


def test_H0_mecanum_mr_eq_13_10():
    rr, l, w = 0.0475, 0.235, 0.15
    pi4 = math.pi / 4
    wheels = [(rr, -pi4, 0.0, l, w), (rr, pi4, 0.0, l, -w), (rr, -pi4, 0.0, -l, -w), (rr, pi4, 0.0, -l, w)]
    H_mr = np.array([[-l - w, 1, -1], [l + w, 1, 1], [l + w, 1, -1], [-l - w, 1, 1]]) / rr
    H = lab.H0_matrix(wheels)
    assert np.allclose(H, H_mr, atol=1e-12)
    assert np.linalg.matrix_rank(H) == 3


def test_diffdrive_F_is_pseudoinverse_of_H0():
    F = lab.diffdrive_F(r, d)
    H = lab.H0_matrix([(r, 0.0, 0.0, 0.0, d), (r, 0.0, 0.0, 0.0, -d)])   # left wheel at y = +d, right at -d
    assert F.shape == (3, 2)
    assert np.allclose(F, np.linalg.pinv(H), atol=1e-12)
    assert np.allclose(H @ F, np.eye(2), atol=1e-12)


def test_wheel_speeds_inverse_of_F():
    for v, w in [(0.8, 0.4), (0.0, 1.0), (-0.3, 0.2)]:
        u = np.array(lab.wheel_speeds(v, w, r, d))
        assert np.allclose(lab.diffdrive_F(r, d) @ u, [w, v, 0.0], atol=1e-12)
    assert np.allclose(lab.wheel_speeds(0.8, 0.4, r, d), [(0.8 - 0.112) / 0.17, (0.8 + 0.112) / 0.17])


@pytest.mark.parametrize("F", ["diff", "mecanum"])
def test_odometry_step_matches_matrix_exponential(F):
    if F == "diff":
        Fm = lab.diffdrive_F(r, d)
        cases = [np.array([2.0, 2.0]), np.array([1.0, 3.0]), np.array([-1.5, 1.5]), np.array([0.3, -0.2])]
    else:
        rr, l, w = 0.0475, 0.235, 0.15
        Fm = rr / 4 * np.array([[-1 / (l + w), 1 / (l + w), 1 / (l + w), -1 / (l + w)], [1, 1, 1, 1], [-1, 1, -1, 1]])
        cases = [RNG.normal(0, 2, 4) for _ in range(4)] + [np.array([-1.0, 1.0, -1.0, 1.0])]
    for dth in cases:
        q = np.array([RNG.uniform(-3, 3), RNG.normal(), RNG.normal()])
        assert np.allclose(lab.odometry_step(q, dth, Fm), exp_odometry(q, Fm @ dth), atol=1e-10)


def test_odometry_step_hand_values():
    # Workshop Problem 3a: q = (0, 0, 0), dtheta = (2.0, 2.5) rad -> V_b = (0.15179, 0.3825, 0)
    q1 = lab.odometry_step([0.0, 0.0, 0.0], [2.0, 2.5], lab.diffdrive_F(r, d))
    w = 0.17 * 0.5 / 0.56
    assert np.allclose(q1, [w, 0.3825 * math.sin(w) / w, 0.3825 * (1 - math.cos(w)) / w], atol=1e-12)


def test_dead_reckoning_on_the_log():
    log = lab.load_log()
    enc = np.c_[log["enc_L"], log["enc_R"]]
    q0 = [log["phi"][0], log["x"][0], log["y"][0]]
    qd = lab.dead_reckoning(q0, enc, lab.diffdrive_F(r, d))
    assert qd.shape == (len(enc), 3)
    assert np.allclose(qd[-1], EXP["odo_final"], atol=1e-6)
    e = math.hypot(qd[-1, 1] - log["x"][-1], qd[-1, 2] - log["y"][-1])
    assert abs(e - EXP["odo_final_pos_err"]) < 1e-6


# ------------------------------------------------------------------------------ Part B: EKF localisation
@pytest.mark.parametrize("dth", [[2.0, 2.5], [1.0, 1.0], [1.0, 1.0 + 1e-6], [-0.7, 0.7], [0.05, 0.08]])
def test_odometry_jacobians_finite_differences(dth):
    dth = np.array(dth, float)
    q = np.array([0.7, 1.0, -2.0])
    qn, Fq, Fu = lab.odometry_jacobians(q, dth, r, d)
    assert np.allclose(qn, lab.odometry_step(q, dth, lab.diffdrive_F(r, d)), atol=1e-12)
    eps = 1e-6
    Fq_n = np.column_stack([(exp_odometry(q + eps * e, lab.diffdrive_F(r, d) @ dth)
                             - exp_odometry(q - eps * e, lab.diffdrive_F(r, d) @ dth)) / (2 * eps) for e in np.eye(3)])
    Fu_n = np.column_stack([(exp_odometry(q, lab.diffdrive_F(r, d) @ (dth + eps * e))
                             - exp_odometry(q, lab.diffdrive_F(r, d) @ (dth - eps * e))) / (2 * eps) for e in np.eye(2)])
    assert np.allclose(Fq, Fq_n, atol=1e-7) and np.allclose(Fu, Fu_n, atol=1e-7)


def test_wheel_noise_cov():
    Q = lab.wheel_noise_cov([2.0, -1.0], r, 4e-5)
    assert np.allclose(Q, np.diag([4e-5 * 2.0 / r, 4e-5 * 1.0 / r]), atol=1e-15)


def test_ekf_predict_formula_and_growth():
    q, Pm = np.array([0.3, 1.0, 2.0]), np.diag([1e-3, 4e-4, 4e-4])
    dth = np.array([1.2, 1.5])
    qn, Pn = lab.ekf_predict(q, Pm, dth, r, d, 4e-5)
    _, Fq, Fu = lab.odometry_jacobians(q, dth, r, d)
    assert np.allclose(qn, lab.odometry_step(q, dth, lab.diffdrive_F(r, d)), atol=1e-12)
    assert np.allclose(Pn, Fq @ Pm @ Fq.T + Fu @ lab.wheel_noise_cov(dth, r, 4e-5) @ Fu.T, atol=1e-15)
    assert np.allclose(Pn, Pn.T) and np.linalg.det(Pn) > np.linalg.det(Pm)   # uncertainty grows (Corke Fig. 6.5)


def test_range_bearing_values_and_jacobian():
    q = np.array([math.pi / 2, 1.0, 1.0])
    z = lab.range_bearing(q, [4.0, 5.0])
    assert np.allclose(z, [5.0, wrapped(math.atan2(4, 3) - math.pi / 2)], atol=1e-12)
    m = np.array([-2.0, 3.5])
    for _ in range(5):
        q = np.array([RNG.uniform(-3, 3), RNG.normal(), RNG.normal()])
        H = lab.range_bearing_jacobian(q, m)
        eps = 1e-6
        cols = []
        for e in np.eye(3):
            dz = lab.range_bearing(q + eps * e, m) - lab.range_bearing(q - eps * e, m)
            cols.append([dz[0] / (2 * eps), wrapped(dz[1]) / (2 * eps)])
        assert np.allclose(H, np.array(cols).T, atol=1e-6)


def test_ekf_update_first_reading_of_the_log():
    fu = EXP["first_update"]
    lm = lab.landmark_table(P)
    q1, P1, info = lab.ekf_update(np.array(fu["q_prior"]), np.array(fu["P_prior"]), fu["z"], lm[fu["id"]], Rm)
    assert info["accepted"]
    assert np.allclose(q1, fu["q_post"], atol=1e-9) and np.allclose(P1, fu["P_post"], atol=1e-12)
    assert abs(info["d2"] - fu["d2"]) < 1e-7
    # Joseph form equals (I - K H) P for the optimal gain
    H = lab.range_bearing_jacobian(fu["q_prior"], lm[fu["id"]])
    assert np.allclose(P1, (np.eye(3) - info["K"] @ H) @ np.array(fu["P_prior"]), atol=1e-12)


def test_ekf_update_wraps_bearing_and_gates():
    q, Pm = np.array([0.0, 0.0, 0.0]), np.diag([1e-3, 1e-2, 1e-2])
    m = np.array([-3.0, 0.0])                      # marker straight behind: bearing = pi
    z = np.array([3.0, -math.pi + 0.01])           # measured just across the +-pi cut
    q1, _, info = lab.ekf_update(q, Pm, z, m, Rm)
    assert abs(info["nu"][1] - 0.01) < 1e-12 and abs(q1[0]) < 0.02
    q2, P2, info2 = lab.ekf_update(q, Pm, [2.0, math.pi], m, Rm, gate=9.21)   # 1 m too short: a reflection
    assert not info2["accepted"] and info2["d2"] > 9.21
    assert np.allclose(q2, q) and np.allclose(P2, Pm)


def test_ekf_localise_on_the_log():
    log, meas = lab.load_log(), lab.load_meas()
    res = lab.ekf_localise(log, meas, P)
    e = res["q"] - np.c_[log["phi"], log["x"], log["y"]]
    rms = math.sqrt(np.mean(e[:, 1] ** 2 + e[:, 2] ** 2))
    assert res["n_acc"] == EXP["ekf"]["n_acc"] and res["n_rej"] == EXP["ekf"]["n_rej"]
    assert abs(rms - EXP["ekf"]["rms_pos"]) < 1e-6
    assert np.allclose(res["q"][-1], EXP["ekf"]["final"], atol=1e-6)
    assert np.allclose(res["P"][-1], EXP["ekf_P_final"], atol=1e-10)
    assert rms < 0.02                               # odometry alone: about 1.6 m rms


# ------------------------------------------------------------------------------ Part C: tracking control
def test_error_coordinates_mr_eq_13_30():
    qd = np.array([math.pi / 2, 1.0, 2.0])
    e = lab.error_coordinates([math.pi / 2 + 0.1, 1.3, 1.8], qd)
    assert np.allclose(e, [0.1, -0.2, -0.3], atol=1e-12)
    assert abs(lab.error_coordinates([3.1, 0, 0], [-3.1, 0, 0])[0] - (6.2 - 2 * math.pi)) < 1e-12


def test_tracking_control_mr_eq_13_31():
    k1, k2, k3 = 2.0, 4.0, 3.0
    assert np.allclose(lab.tracking_control([0.3, 1, 2], [0.3, 1, 2], 0.6, 0.4, k1, k2, k3), [0.6, 0.4])
    v, w = lab.tracking_control([0.1, 0.1, -0.2], [0.0, 0.0, 0.0], 0.6, 0.0, k1, k2, k3)
    pe, xe, ye = 0.1, 0.1, -0.2                    # q_d = 0: {d} = {s}
    v_ref = (0.6 - k1 * 0.6 * (xe + ye * math.tan(pe))) / math.cos(pe)
    w_ref = 0.0 - (k2 * 0.6 * ye + k3 * 0.6 * math.tan(pe)) * math.cos(pe) ** 2
    assert np.allclose([v, w], [v_ref, w_ref], atol=1e-12)


def test_point_controls_mr_eq_13_21():
    for phi, xr, yr in [(0.4, 0.5, 0.0), (-2.0, 0.3, 0.1)]:
        v, w = 0.7, -0.4
        pdot = np.array([v * math.cos(phi), v * math.sin(phi)]) + w * np.array(
            [[-math.sin(phi), -math.cos(phi)], [math.cos(phi), -math.sin(phi)]]) @ [xr, yr]      # MR Eq. 13.20
        assert np.allclose(lab.point_controls(phi, xr, yr, pdot), [v, w], atol=1e-12)


def test_point_tracking_control_zero_error_gives_nominal():
    qd = [0.8, 2.0, 1.0]
    assert np.allclose(lab.point_tracking_control(qd, qd, 0.6, 0.4, 0.5, 1.5), [0.6, 0.4], atol=1e-12)


def test_saturate_wheels_keeps_curvature():
    v, w = lab.saturate_wheels(1.0, 3.0, r, d, 9.0)
    uL, uR = lab.wheel_speeds(v, w, r, d)
    assert abs(max(abs(uL), abs(uR)) - 9.0) < 1e-12 and abs(w / v - 3.0) < 1e-12
    assert np.allclose(lab.saturate_wheels(2.0, 0.0, r, d, 9.0, 1.2), [1.2, 0.0])
    assert np.allclose(lab.saturate_wheels(0.5, 0.2, r, d, 9.0, 1.2), [0.5, 0.2])


@pytest.mark.parametrize("est", ["truth", "odometry", "ekf"])
def test_closed_loop_runs(est):
    sim = lab.simulate(P, est, "nonlinear")
    m = lab.tracking_metrics(sim)
    ref = EXP[f"track_{est}_nonlinear"]
    for key in ("rms_pos", "max_pos", "final_pos", "peak_u"):
        assert abs(m[key] - ref[key]) < 1e-6, key


# ------------------------------------------------------------------------------ Part D (optional): mobile manipulation
def test_base_jacobian_finite_difference_and_expected():
    arm = cm.srs_7r_feeding()
    Tb0 = np.eye(4)
    Tb0[:3, 3] = P["arm_mount"]["p_b0"]
    th = np.array(P["arm_mount"]["theta_feed"])
    F = lab.diffdrive_F(r, d)
    J = lab.mobile_manip_jacobian(F, Tb0, arm["Blist"], arm["M"], th)
    assert J.shape == (6, 9)
    assert np.allclose(J[:, 2:], mr.JacobianBody(arm["Blist"], th), atol=1e-12)
    assert np.allclose(J[:, :2], EXP["J_base_feed"], atol=1e-10)
    T0e = mr.FKinBody(arm["M"], arm["Blist"], th)
    q = np.array([0.4, 1.0, 2.0])

    def X(qq):
        Tsb = mr.RpToTrans(mr.MatrixExp3(mr.VecToso3([0, 0, qq[0]])), [qq[1], qq[2], 0.17])
        return Tsb @ Tb0 @ T0e
    eps = 1e-6
    for u in (np.array([1.0, 1.0]), np.array([-1.0, 1.0])):
        Ve = mr.se3ToVec(mr.MatrixLog6(mr.TransInv(X(q)) @ X(exp_odometry(q, F @ (eps * u))))) / eps
        assert np.allclose(lab.base_jacobian(F, Tb0, T0e) @ u, Ve, atol=1e-5)
