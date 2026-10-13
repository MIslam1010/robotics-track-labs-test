"""SBEG 461 Lab 5 -- trajectory library and torque feasibility on the course arm.
STARTER -- you edit this file. Replace every `raise NotImplementedError` with your code.
Run the auto-checks with:  LAB_IMPL=starter python -m pytest -q test_lab.py
Rules: do not call modern_robotics or scipy in this file (the tests use them as ground truth);
you may use numpy and the provided helpers (rnea, load_course_arm, load_tasks, export_csv, ...).

Notation follows Modern Robotics (MR) Ch. 9: a trajectory theta(t) = theta(s(t)) is a path theta(s),
s in [0, 1], composed with a time scaling s(t), t in [0, T] (MR Sec. 9.1, Eqs. 9.1-9.2).
Dynamics in MR list form (MR Sec. 8.3.2): Mlist, Glist, Slist, gravity g.

Parts
  A. Point-to-point time scalings: cubic, quintic, trapezoid       MR Sec. 9.2.2, Eqs. 9.9-9.24
     + straight-line joint trajectories                             MR Eqs. 9.3-9.5
  B. Via points: cubic via-point interpolation                      MR Sec. 9.3, Eqs. 9.25-9.29
     + uniform cubic B-spline                                       MR p. 338; Gotor Sec. 7.1.4
  C. Torque feasibility on the course arm                           MR Eqs. 8.50-8.54 (RNEA), 9.30-9.31
     + uniform time scaling (stretching a trajectory by c > 0)      MR Sec. 9.7 (Hollerbach)

These functions become the trajectory module of your `sbeg461_dynamics` library; Lab 6 (time-optimal
scaling, hardware run) and the project (`sbeg461_project/traj.py`) use the same names.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
DATA = HERE / "data"

# ==========================================================================================
# Provided helpers (do not edit)
# ==========================================================================================


def skew(w):
    """[w] for a 3-vector (MR Eq. 3.30)."""
    return np.array([[0.0, -w[2], w[1]], [w[2], 0.0, -w[0]], [-w[1], w[0], 0.0]])


def TransInv(T):
    R, p = T[:3, :3], T[:3, 3]
    out = np.eye(4)
    out[:3, :3], out[:3, 3] = R.T, -R.T @ p
    return out


def Adjoint(T):
    """[Ad_T] (MR Def. 3.20)."""
    R, p = T[:3, :3], T[:3, 3]
    Ad = np.zeros((6, 6))
    Ad[:3, :3], Ad[3:, :3], Ad[3:, 3:] = R, skew(p) @ R, R
    return Ad


def ad(V):
    """[ad_V] (MR Eq. 8.38)."""
    out = np.zeros((6, 6))
    out[:3, :3], out[3:, :3], out[3:, 3:] = skew(V[:3]), skew(V[3:]), skew(V[:3])
    return out


def MatrixExp6(S, theta):
    """e^{[S] theta} for a screw axis with ||omega|| = 1 or (omega = 0, ||v|| = 1) (MR Prop. 3.25)."""
    w, v = np.asarray(S[:3], float), np.asarray(S[3:], float)
    T = np.eye(4)
    if np.linalg.norm(w) < 1e-12:
        T[:3, 3] = v * theta
        return T
    W = skew(w)
    R = np.eye(3) + np.sin(theta) * W + (1 - np.cos(theta)) * W @ W
    G = np.eye(3) * theta + (1 - np.cos(theta)) * W + (theta - np.sin(theta)) * W @ W
    T[:3, :3], T[:3, 3] = R, G @ v
    return T


def rnea(thetalist, dthetalist, ddthetalist, g, Ftip, Mlist, Glist, Slist):
    """Recursive Newton-Euler inverse dynamics (MR Eqs. 8.50-8.54), same interface as
    modern_robotics.InverseDynamics.  This is the Lab 2/Lab 4 function; you may paste your own."""
    n = len(thetalist)
    Slist = np.asarray(Slist, float)
    Mi = np.eye(4)
    Ai = np.zeros((6, n))
    AdTi = [None] * (n + 1)
    Vi = np.zeros((6, n + 1))
    Vdi = np.zeros((6, n + 1))
    Vdi[3:, 0] = -np.asarray(g, float)
    AdTi[n] = Adjoint(TransInv(Mlist[n]))
    Fi = np.asarray(Ftip, float).copy()
    tau = np.zeros(n)
    for i in range(n):
        Mi = Mi @ Mlist[i]
        Ai[:, i] = Adjoint(TransInv(Mi)) @ Slist[:, i]
        Ti = MatrixExp6(-Ai[:, i], thetalist[i]) @ TransInv(Mlist[i])
        AdTi[i] = Adjoint(Ti)
        Vi[:, i + 1] = AdTi[i] @ Vi[:, i] + Ai[:, i] * dthetalist[i]
        Vdi[:, i + 1] = (AdTi[i] @ Vdi[:, i] + Ai[:, i] * ddthetalist[i]
                         + ad(Vi[:, i + 1]) @ Ai[:, i] * dthetalist[i])
    for i in range(n - 1, -1, -1):
        Fi = (AdTi[i + 1].T @ Fi + Glist[i] @ Vdi[:, i + 1]
              - ad(Vi[:, i + 1]).T @ (Glist[i] @ Vi[:, i + 1]))
        tau[i] = Fi @ Ai[:, i]
    return tau


def load_course_arm(payload=True, derated=True):
    """The course arm (from the course robot models in Course resources, numbers identical).

    Returns a dict with Slist (6x6), Mlist (7 4x4), Glist (6 6x6), g (3,), qdot_max (6,), tau_max (6,),
    limits (6x2), q_home (6,).  payload=True uses the 0.25 kg maximum payload at the tool point.
    derated=True applies the hardware-run derating (50 % of rated speed, 60 % of rated torque).
    All inertial data and ratings are ILLUSTRATIVE values."""
    d = json.loads((DATA / "course_arm.json").read_text())
    kf = d["derated_speed_fraction"] if derated else 1.0
    tf = d["derated_current_fraction"] if derated else 1.0
    return dict(Slist=np.array(d["Slist"]), M=np.array(d["M"]),
                Mlist=[np.array(m) for m in d["Mlist"]],
                Glist=[np.array(G) for G in (d["Glist_payload"] if payload else d["Glist"])],
                g=np.array(d["g"]), qdot_max=kf * np.array(d["qdot_max"]),
                tau_max=tf * np.array(d["tau_max"]), limits=np.array(d["limits"]),
                q_home=np.array(d["q_home"]))


def load_tasks():
    """Task configurations A, B, C, W (rad) and via times (s) of the lab (illustrative)."""
    t = json.loads((DATA / "tasks.json").read_text())
    return {k: (np.array(v) if isinstance(v, list) else v) for k, v in t.items()}


# ==========================================================================================
# Part A -- point-to-point time scalings (MR Sec. 9.2.2)
# ==========================================================================================


def cubic_time_scaling(T, t):
    """Cubic rest-to-rest time scaling s = 3(t/T)^2 - 2(t/T)^3 (MR Eq. 9.9 with a2 = 3/T^2, a3 = -2/T^3).

    t may be a scalar or an array; it is clipped to [0, T].  Returns (s, sdot, sddot) with the same
    shape as t.  At t = 0 and t = T return the one-sided values (sddot = +-6/T^2)."""
    # TODO: MR Eqs. 9.9-9.13: s = a2 t^2 + a3 t^3 with a2 = 3/T^2, a3 = -2/T^3; differentiate twice. Clip t to [0, T].
    raise NotImplementedError("cubic_time_scaling")


def quintic_time_scaling(T, t):
    """Quintic rest-to-rest time scaling s = 10 r^3 - 15 r^4 + 6 r^5, r = t/T, with zero end velocities
    and accelerations (MR Sec. 9.2.2.1, Exercise 9.5; Fig. 9.4).  Returns (s, sdot, sddot)."""
    # TODO: Solve MR Exercise 9.5 (six boundary conditions) or use r = t/T: s = 10 r^3 - 15 r^4 + 6 r^5; remember the 1/T and 1/T^2 factors.
    raise NotImplementedError("quintic_time_scaling")


def trapezoid_params(v, a):
    """Total time and acceleration time of a trapezoidal time scaling with s-units v (1/s) and a (1/s^2).

    Three-stage profile if v^2/a <= 1:  T = (a + v^2)/(v a), t_a = v/a  (MR p. 334).
    If v^2/a > 1 the cruise speed is never reached (MR Exercise 9.10): bang-bang profile with
    t_a = T/2 = 1/sqrt(a) and peak speed sqrt(a).  Returns (T, t_a, v_peak)."""
    # TODO: MR p. 334: t_a = v/a and T = (a + v^2)/(v a) if v^2/a <= 1; otherwise bang-bang (MR Ex. 9.10, 9.13).
    raise NotImplementedError("trapezoid_params")


def trapezoid_time_scaling(v, a, t):
    """Trapezoidal time scaling (MR Eqs. 9.16-9.24); bang-bang (triangle) if v^2/a > 1.
    Returns (s, sdot, sddot) for scalar or array t (clipped to [0, T])."""
    # TODO: Three phases, MR Eqs. 9.16-9.24 (use v_peak from trapezoid_params so the bang-bang case also works). np.where helps for arrays.
    raise NotImplementedError("trapezoid_time_scaling")


def trapezoid_from_limits(theta_start, theta_end, qdot_lim, qddot_lim):
    """Fastest trapezoidal straight-line motion (MR Eqs. 9.14-9.15): the largest v and a (s-units) with
    |(theta_end - theta_start) v| <= qdot_lim and |(theta_end - theta_start) a| <= qddot_lim, joint by
    joint (joints that do not move impose no limit).  Returns (v, a, T)."""
    # TODO: MR Eqs. 9.14-9.15 joint by joint: v = min_i qdot_lim_i/|D_i|, a = min_i qddot_lim_i/|D_i| over joints with D_i != 0.
    raise NotImplementedError("trapezoid_from_limits")


def joint_trajectory(theta_start, theta_end, T, N, method=5, v=None, a=None):
    """Straight line in joint space theta(s) = theta_start + s (theta_end - theta_start) (MR Eq. 9.3),
    time-scaled with method = 3 (cubic), 5 (quintic) or "trap" (trapezoid; needs v and a and ignores T).

    Returns (t, Q, Qd, Qdd): t (N,), Q/Qd/Qdd (N, n), with Qd = sdot D and Qdd = sddot D (MR Eqs. 9.4-9.5;
    the path is straight so d2theta/ds2 = 0).  The rows of Q equal modern_robotics.JointTrajectory."""
    # TODO: MR Eqs. 9.3-9.5: Q = q0 + s D, Qd = sdot D, Qdd = sddot D (np.outer is handy).
    raise NotImplementedError("joint_trajectory")


# ==========================================================================================
# Part B -- via points (MR Sec. 9.3) and B-splines
# ==========================================================================================


def via_cubic_coefficients(beta, beta_dot, times):
    """Cubic via-point interpolation of ONE coordinate (MR Eqs. 9.26-9.29).

    beta, beta_dot: (k,) via positions and velocities; times: (k,) increasing via times T_1 = 0 .. T_k.
    Returns a (k-1, 4) array whose row j is (a_j0, a_j1, a_j2, a_j3) for segment j."""
    # TODO: MR Eqs. 9.26-9.29, one row (a_j0, a_j1, a_j2, a_j3) per segment.
    raise NotImplementedError("via_cubic_coefficients")


def heuristic_via_velocities(points, times):
    """Via velocities from positions and times (MR p. 337: "a heuristic could be used").
    Rule used in this course (as in Lecture B), per coordinate: zero at the first and last via; at an
    interior via, the average of the slopes of the two adjacent segments if both slopes have the same
    strict sign, otherwise zero (the via is a turning point, or one side is flat).  points: (k, m); returns (k, m)."""
    # TODO: Slopes m1, m2 of the two segments around via i; average if m1*m2 > 0, otherwise 0. End vias: 0.
    raise NotImplementedError("heuristic_via_velocities")


def via_trajectory(points, velocities, times, t):
    """Evaluate the cubic via-point trajectory of all m coordinates at times t.
    points, velocities: (k, m); times: (k,).  Returns (B, Bd, Bdd), each (len(t), m)."""
    # TODO: Find the segment of each t (np.searchsorted), dt = t - T_j, evaluate the cubic and its two derivatives.
    raise NotImplementedError("via_trajectory")


def bspline_basis(u):
    """The four uniform cubic B-spline blending functions at local parameter u in [0, 1]:
    b0 = (1-u)^3/6, b1 = (3u^3 - 6u^2 + 4)/6, b2 = (-3u^3 + 3u^2 + 3u + 1)/6, b3 = u^3/6.
    Returns an array of shape (4,) (or (4, len(u)))."""
    # TODO: Four cubic blending functions; they must sum to 1 for every u.
    raise NotImplementedError("bspline_basis")


def uniform_cubic_bspline(ctrl, u):
    """Uniform cubic B-spline with control points ctrl (n, m), n >= 4, at global parameter
    u in [0, n-3] (segment i = floor(u) uses P_i..P_{i+3}, local parameter u - i).
    Returns (p, dp/du, d2p/du2), each (m,).  The curve is C2 and lies in the convex hull of the
    control points (MR p. 338)."""
    # TODO: Segment i = floor(u) (at most n-4), local w = u - i, p = sum_k b_k(w) P_{i+k}; differentiate the b_k for dp/du, d2p/du2.
    raise NotImplementedError("uniform_cubic_bspline")


# ==========================================================================================
# Part C -- torque feasibility (MR Eqs. 9.30-9.31) and uniform time scaling
# ==========================================================================================


def torque_profile(robot, Q, Qd, Qdd):
    """Joint torques tau_k = M(q_k) qdd_k + c(q_k, qd_k) + g(q_k) along a sampled trajectory, by the RNEA.
    robot: dict from load_course_arm.  Returns (N, n)."""
    # TODO: Call rnea(...) with Ftip = 0 for every sample (MR Eq. 9.30: tau = M qdd + c + g).
    raise NotImplementedError("torque_profile")


def check_limits(robot, Qd, Tau, tol=1e-9):
    """Worst |velocity|/limit and |torque|/limit per joint along a trajectory.
    Returns dict(vel_ratio=(n,), tau_ratio=(n,), ok=bool)."""
    # TODO: Ratios |Qd|/qdot_max and |Tau|/tau_max, maximum over samples (axis 0).
    raise NotImplementedError("check_limits")


def min_time_scale(robot, Q, Qd, Qdd):
    """Smallest uniform stretch factor c > 0 that makes a sampled trajectory feasible.

    Stretching time by c (theta_c(t) = theta(t/c)) divides velocities by c and accelerations by c^2, and
    because c(theta, thetadot) is quadratic in thetadot (MR Sec. 9.4, c = thetadot^T Gamma thetadot)
        tau_c = g(theta) + d / c^2,   d = tau(Q, Qd, Qdd) - g(theta)     (MR Sec. 9.7, Hollerbach 1984).
    Velocity limits give c >= max |qd| / qdot_max.  Torque limits give, sample by sample and joint by joint,
        d > 0:  c^2 >= d / (tau_max - g),      d < 0:  c^2 >= -d / (tau_max + g),
    which needs |g| < tau_max (otherwise the robot cannot even hold the configuration: raise ValueError).

    Returns dict(c_vel, c_tau, c=max(c_vel, c_tau), binding=("velocity"|"torque", joint index 0-based)).
    For a rest-to-rest trajectory computed with T = 1, c is the minimum feasible duration in seconds."""
    # TODO: G = torques at zero velocity/acceleration; D = full torques - G; apply the two c^2 inequalities of the docstring sample by sample; c_vel = max |Qd|/qdot_max.
    raise NotImplementedError("min_time_scale")


def export_csv(path, t, Q, Qd):
    """Write t, q1..qn, qd1..qdn (SI units) to a CSV file (the quintic baseline for Lab 6; not a hardware command)."""
    n = Q.shape[1]
    header = "t," + ",".join(f"q{i + 1}" for i in range(n)) + "," + ",".join(f"qd{i + 1}" for i in range(n))
    np.savetxt(path, np.column_stack([t, Q, Qd]), delimiter=",", header=header, comments="", fmt="%.6f")
