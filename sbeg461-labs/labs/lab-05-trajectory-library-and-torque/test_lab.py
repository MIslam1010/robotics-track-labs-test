"""Auto-checks for SBEG 461 Lab 5 (trajectory library; torque feasibility on the course arm).

Usage:   LAB_IMPL=starter  python -m pytest -q test_lab.py     (your work, file starter.py)
         LAB_IMPL=solution python -m pytest -q test_lab.py     (reference; default)
Ground truth: modern_robotics (CubicTimeScaling, QuinticTimeScaling, JointTrajectory, InverseDynamics,
InverseDynamicsTrajectory), the numbers printed in MR Sec. 9.2-9.3 (Figs. 9.3, 9.4, 9.8; Eqs. 9.16-9.29),
scipy.interpolate.BSpline, and a brute-force bisection on the trajectory duration.
"""
import importlib
import os
import sys
from pathlib import Path

import numpy as np
import pytest
import modern_robotics as mr

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
lab = importlib.import_module(os.environ.get("LAB_IMPL", "starter"))

RNG = np.random.default_rng(461)


@pytest.fixture(scope="module")
def arm():
    return lab.load_course_arm(payload=True, derated=True)


@pytest.fixture(scope="module")
def tasks():
    return lab.load_tasks()


# ---------------------------------------------------------------- Part A: point-to-point time scalings

@pytest.mark.parametrize("T", [0.7, 2.0, 5.0])
def test_cubic_matches_modern_robotics_and_mr_fig_9_3(T):
    t = np.linspace(0, T, 41)
    s, sd, sdd = lab.cubic_time_scaling(T, t)
    assert np.allclose(s, [mr.CubicTimeScaling(T, ti) for ti in t], atol=1e-12)
    assert np.isclose(np.max(sd), 3 / (2 * T), atol=1e-9)              # MR Fig. 9.3: peak 3/(2T) at T/2
    assert np.isclose(sdd[0], 6 / T**2) and np.isclose(sdd[-1], -6 / T**2)
    assert np.allclose(np.gradient(s, t)[1:-1], sd[1:-1], atol=0.02 / T)


@pytest.mark.parametrize("T", [0.7, 2.0, 5.0])
def test_quintic_matches_modern_robotics_and_mr_fig_9_4(T):
    t = np.linspace(0, T, 2001)
    s, sd, sdd = lab.quintic_time_scaling(T, t)
    assert np.allclose(s, [mr.QuinticTimeScaling(T, ti) for ti in t], atol=1e-12)
    assert np.isclose(np.max(sd), 15 / (8 * T), rtol=1e-6)              # MR Fig. 9.4
    assert np.isclose(np.max(sdd), 10 / (np.sqrt(3) * T**2), rtol=1e-5)
    assert abs(sd[0]) < 1e-12 and abs(sdd[0]) < 1e-12 and abs(sdd[-1]) < 1e-9
    assert np.allclose(np.gradient(sd, t)[1:-1], sdd[1:-1], atol=0.01 / T**2)


def test_time_scalings_accept_scalars_and_clip():
    for f in (lab.cubic_time_scaling, lab.quintic_time_scaling):
        s, sd, sdd = f(2.0, 1.0)
        assert np.isclose(float(s), 0.5)
        assert np.isclose(float(f(2.0, 3.0)[0]), 1.0) and np.isclose(float(f(2.0, -1.0)[0]), 0.0)


def test_trapezoid_three_stage_mr_eqs_9_16_to_9_24():
    v, a = 0.5, 1.25                                  # v^2/a = 0.2 <= 1: three stages
    T, ta, vp = lab.trapezoid_params(v, a)
    assert np.isclose(T, (a + v**2) / (v * a)) and np.isclose(T, 2.4)
    assert np.isclose(ta, 0.4) and np.isclose(vp, 0.5)
    t = np.linspace(0, T, 4801)
    s, sd, sdd = lab.trapezoid_time_scaling(v, a, t)
    assert np.isclose(s[-1], 1.0) and np.isclose(s[0], 0.0)
    assert np.isclose(np.max(sd), v) and np.isclose(np.max(sdd), a) and np.isclose(np.min(sdd), -a)
    assert np.all(np.diff(s) >= -1e-12)
    assert np.allclose(np.gradient(s, t), sd, atol=2e-3)
    k = np.searchsorted(t, 1.0)                       # cruise phase, MR Eq. 9.21
    assert np.isclose(s[k], v * t[k] - v**2 / (2 * a))


def test_trapezoid_bang_bang_when_v2_over_a_exceeds_1():
    v, a = 2.0, 1.0                                   # v^2/a = 4 > 1 (MR Ex. 9.10, 9.13)
    T, ta, vp = lab.trapezoid_params(v, a)
    assert np.isclose(T, 2.0) and np.isclose(ta, 1.0) and np.isclose(vp, 1.0)
    s, sd, _ = lab.trapezoid_time_scaling(v, a, np.linspace(0, T, 1001))
    assert np.isclose(s[-1], 1.0) and np.isclose(np.max(sd), 1.0)


def test_trapezoid_from_limits_mr_exercise_9_4_data():
    # MR Exercise 9.4 data: theta (0,0) -> (pi, pi/3), |qd| <= 2, |qdd| <= 0.5 -> bang-bang
    v, a, T = lab.trapezoid_from_limits([0, 0], [np.pi, np.pi / 3], [2, 2], [0.5, 0.5])
    assert np.isclose(v, 2 / np.pi) and np.isclose(a, 0.5 / np.pi)
    assert np.isclose(T, 2 * np.sqrt(np.pi / 0.5))
    # a joint that does not move imposes no limit
    v2, a2, _ = lab.trapezoid_from_limits([0, 1], [1, 1], [1, 1e-9], [1, 1e-9])
    assert np.isclose(v2, 1.0) and np.isclose(a2, 1.0)


@pytest.mark.parametrize("method", [3, 5])
def test_joint_trajectory_matches_modern_robotics(method):
    q0, q1 = RNG.uniform(-2, 2, 6), RNG.uniform(-2, 2, 6)
    t, Q, Qd, Qdd = lab.joint_trajectory(q0, q1, 1.7, 35, method)
    assert Q.shape == (35, 6) and Qd.shape == (35, 6) and Qdd.shape == (35, 6)
    assert np.allclose(t, np.linspace(0, 1.7, 35))
    assert np.allclose(Q, np.array(mr.JointTrajectory(q0, q1, 1.7, 35, method)), atol=1e-12)
    dtm = t[1] - t[0]
    assert np.allclose(np.gradient(Q, dtm, axis=0)[2:-2], Qd[2:-2], atol=0.05)


def test_joint_trajectory_trapezoid_respects_limits(tasks):
    q0, q1 = tasks["A"], tasks["B"]
    vlim, alim = np.full(6, 1.3962634), np.array(tasks["qddot_lim"])
    v, a, T = lab.trapezoid_from_limits(q0, q1, vlim, alim)
    t, Q, Qd, Qdd = lab.joint_trajectory(q0, q1, None, 501, "trap", v=v, a=a)
    assert np.isclose(t[-1], T) and np.allclose(Q[-1], q1) and np.allclose(Q[0], q0)
    assert np.all(np.abs(Qd) <= vlim + 1e-9) and np.all(np.abs(Qdd) <= alim + 1e-9)
    assert np.isclose(np.max(np.abs(Qd[:, 0])), vlim[0])            # joint 1 saturates (largest move)


# ---------------------------------------------------------------- Part B: via points and B-splines

def test_via_cubic_coefficients_formula_and_continuity():
    beta, bdot, Tm = [0.0, 1.0, 0.5, 2.0], [0.0, 0.8, -0.3, 0.0], [0.0, 1.0, 2.5, 3.0]
    A = lab.via_cubic_coefficients(beta, bdot, Tm)
    assert A.shape == (3, 4)
    for j in range(3):                                       # each segment meets its four constraints
        dT = Tm[j + 1] - Tm[j]
        a0, a1, a2, a3 = A[j]
        assert np.isclose(a0, beta[j]) and np.isclose(a1, bdot[j])
        assert np.isclose(a0 + a1 * dT + a2 * dT**2 + a3 * dT**3, beta[j + 1])
        assert np.isclose(a1 + 2 * a2 * dT + 3 * a3 * dT**2, bdot[j + 1])


def test_via_trajectory_mr_figure_9_7a():
    # MR p. 337: vias (0,0), (0,1), (1,1), (1,0) at T = 0, 1, 2, 3, velocities (0,0), (1,0), (0,-1), (0,0)
    P = np.array([[0, 0], [0, 1], [1, 1], [1, 0]], float)
    V = np.array([[0, 0], [1, 0], [0, -1], [0, 0]], float)
    Tm = [0, 1, 2, 3]
    B, Bd, Bdd = lab.via_trajectory(P, V, Tm, np.array([0, 1, 2, 3]))
    assert np.allclose(B, P) and np.allclose(Bd, V)
    t = np.linspace(0, 3, 3001)
    B, Bd, _ = lab.via_trajectory(P, V, Tm, t)
    assert np.allclose(np.gradient(B, t, axis=0)[1:-1], Bd[1:-1], atol=2e-3)
    assert B[:, 0].min() < -0.05            # x dips below 0 in segment 1 (Fig. 9.7a): not in the convex hull


def test_heuristic_via_velocities():
    P = np.array([[0.0, 0.0], [0.10, 0.20], [0.30, 0.20], [0.40, 0.0]])
    V = lab.heuristic_via_velocities(P, [0.0, 1.0, 2.0, 3.0])
    assert np.allclose(V, [[0, 0], [0.15, 0.0], [0.15, 0.0], [0, 0]])          # flat side -> 0
    V2 = lab.heuristic_via_velocities(np.array([[0.0], [1.0], [0.0]]), [0, 1, 2])
    assert np.allclose(V2, 0.0)                                     # turning point -> zero


def test_bspline_basis_partition_of_unity():
    u = np.linspace(0, 1, 11)
    b = lab.bspline_basis(u)
    assert b.shape == (4, 11)
    assert np.allclose(b.sum(axis=0), 1.0) and np.all(b >= -1e-15)
    assert np.allclose(lab.bspline_basis(0.0), [1 / 6, 4 / 6, 1 / 6, 0])


def test_uniform_cubic_bspline_matches_scipy_and_convex_hull():
    from scipy.interpolate import BSpline
    P = RNG.uniform(-1, 1, (7, 2))
    n = len(P)
    spl = BSpline(np.arange(n + 4, dtype=float), P, 3)      # uniform knots; valid on [3, n]
    for u in np.linspace(0, n - 3, 37):
        p, dp, ddp = lab.uniform_cubic_bspline(P, u)
        assert np.allclose(p, spl(u + 3), atol=1e-12)
        assert np.allclose(dp, spl.derivative(1)(u + 3), atol=1e-10)
        assert np.allclose(ddp, spl.derivative(2)(u + 3), atol=1e-9)
        i = min(int(np.floor(u)), n - 4)                    # local convex hull: inside the box of P_i..P_i+3
        box = P[i:i + 4]
        assert np.all(p >= box.min(axis=0) - 1e-12) and np.all(p <= box.max(axis=0) + 1e-12)


# ---------------------------------------------------------------- Part C: torque feasibility on the course arm

def test_course_arm_data(arm):
    assert arm["Slist"].shape == (6, 6) and len(arm["Mlist"]) == 7 and len(arm["Glist"]) == 6
    assert np.allclose(arm["tau_max"], 0.6 * np.array([1.8, 3.0, 1.5, 0.6, 0.6, 0.4]))
    assert np.allclose(arm["qdot_max"], 0.5 * np.deg2rad(160.0))


def test_torque_profile_matches_modern_robotics(arm, tasks):
    t, Q, Qd, Qdd = lab.joint_trajectory(tasks["A"], tasks["B"], 1.8, 25, 5)
    Tau = lab.torque_profile(arm, Q, Qd, Qdd)
    ref = np.array(mr.InverseDynamicsTrajectory(Q, Qd, Qdd, arm["g"], np.zeros((25, 6)),
                                                arm["Mlist"], arm["Glist"], arm["Slist"]))
    assert Tau.shape == (25, 6) and np.allclose(Tau, ref, atol=1e-10)


def test_check_limits(arm):
    Qd = np.zeros((3, 6))
    Qd[1, 2] = 0.5 * arm["qdot_max"][2]
    Tau = np.zeros((3, 6))
    Tau[2, 3] = -1.5 * arm["tau_max"][3]
    out = lab.check_limits(arm, Qd, Tau)
    assert np.isclose(out["vel_ratio"][2], 0.5) and np.isclose(out["tau_ratio"][3], 1.5)
    assert out["ok"] is False
    assert lab.check_limits(arm, Qd, 0 * Tau)["ok"] is True


def _bisect_min_T(arm, q0, q1, method, N=201):
    def ok(T):
        _, Q, Qd, Qdd = lab.joint_trajectory(q0, q1, T, N, method)
        Tau = np.array(mr.InverseDynamicsTrajectory(Q, Qd, Qdd, arm["g"], np.zeros((N, 6)),
                                                    arm["Mlist"], arm["Glist"], arm["Slist"]))
        return np.all(np.abs(Qd) <= arm["qdot_max"] + 1e-12) and np.all(np.abs(Tau) <= arm["tau_max"] + 1e-12)
    lo, hi = 0.01, 10.0
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        lo, hi = (lo, mid) if ok(mid) else (mid, hi)
    return hi


@pytest.mark.parametrize("goal, method, binding", [("B", 5, "velocity"), ("D", 5, "torque"), ("D", 3, "torque")])
def test_min_time_scale_equals_bisection(arm, tasks, goal, method, binding):
    _, Q, Qd, Qdd = lab.joint_trajectory(tasks["A"], tasks[goal], 1.0, 201, method)
    res = lab.min_time_scale(arm, Q, Qd, Qdd)
    assert np.isclose(res["c"], max(res["c_vel"], res["c_tau"]))
    assert res["binding"][0] == binding
    assert np.isclose(res["c"], _bisect_min_T(arm, tasks["A"], tasks[goal], method), rtol=1e-6)


def test_min_time_scale_checkpoint_values(arm, tasks):
    _, Q, Qd, Qdd = lab.joint_trajectory(tasks["A"], tasks["B"], 1.0, 401, 5)
    r = lab.min_time_scale(arm, Q, Qd, Qdd)
    assert np.isclose(r["c_vel"], 1.6114, atol=1e-3) and r["binding"] == ("velocity", 0)
    _, Q, Qd, Qdd = lab.joint_trajectory(tasks["A"], tasks["D"], 1.0, 401, 5)
    r = lab.min_time_scale(arm, Q, Qd, Qdd)
    assert np.isclose(r["c_tau"], 0.2348, atol=1e-3) and r["binding"] == ("torque", 3)   # joint 4!


def test_min_time_scale_rejects_static_overload(arm, tasks):
    heavy = dict(arm, tau_max=0.1 * arm["tau_max"])           # cannot even hold the arm against gravity
    _, Q, Qd, Qdd = lab.joint_trajectory(tasks["A"], tasks["B"], 1.0, 21, 5)
    with pytest.raises(ValueError):
        lab.min_time_scale(heavy, Q, Qd, Qdd)


def test_via_trajectory_on_course_arm_is_feasible_and_stretch_works(arm, tasks):
    P = np.array([tasks["A"], tasks["W"], tasks["B"]])
    Tm = tasks["via_times"]
    V = lab.heuristic_via_velocities(P, Tm)
    t = np.linspace(Tm[0], Tm[-1], 481)
    B, Bd, Bdd = lab.via_trajectory(P, V, Tm, t)
    res = lab.min_time_scale(arm, B, Bd, Bdd)
    assert res["c"] < 1.0                                     # the given timing is feasible
    c = res["c"]                                              # stretched trajectory touches a limit
    Tau = lab.torque_profile(arm, B, Bd / c, Bdd / c**2)
    out = lab.check_limits(arm, Bd / c, Tau)
    assert out["ok"] and np.isclose(max(out["vel_ratio"].max(), out["tau_ratio"].max()), 1.0, atol=1e-9)
