"""SBEG 461 Lab 6 -- time-optimal time scaling (MR Sec. 9.4): phase plane, bang-bang, TOPP (toppra)
versus quintic, and a hardware command file for the course arm.  STARTER: complete every function marked TODO.

These functions become `sbeg461_dynamics/timescaling.py` in your cumulative library.

Notation (MR Ch. 9): path theta(s), s in [0, 1]; time scaling s(t); theta' = d theta/ds,
theta'' = d^2 theta/ds^2.  Along the path the dynamics are (MR Eqs. 9.32-9.33)

    m(s) sdd + c(s) sd^2 + g(s) = tau,
    m(s) = M(theta) theta',  c(s) = M(theta) theta'' + theta'^T Gamma(theta) theta',  g(s) = g(theta),

and the actuator limits tau_min <= tau <= tau_max become L(s, sd) <= sdd <= U(s, sd) (MR Eqs. 9.36-9.37).
Units: SI (m, kg, s, rad, N m).
"""
from __future__ import annotations

import csv
from pathlib import Path

import modern_robotics as mr
import numpy as np

import course_arm as _ca

HERE = Path(__file__).resolve().parent
DATA = HERE / "data"
EPS_M = 1e-9            # |m_i(s)| below this is treated as a zero-inertia point (MR Sec. 9.4.4)


# =============================================================================================
# Provided: paths and robot models
# =============================================================================================
try:                                         # toppra paths derive from AbstractGeometricPath
    from toppra.interpolator import AbstractGeometricPath as _PathBase
except Exception:                            # pragma: no cover - toppra missing
    _PathBase = object


class StraightLinePath(_PathBase):
    """theta(s) = q0 + s (q1 - q0), s in [0, 1] (MR Eq. 9.3).  Call as path(s, order) like toppra paths:
    order 0 -> theta, 1 -> theta' = q1 - q0, 2 -> theta'' = 0.  `s` may be a float or an array."""

    def __init__(self, q0, q1):
        self.q0 = np.asarray(q0, float)
        self.q1 = np.asarray(q1, float)

    def __call__(self, s, order: int = 0):
        s_arr = np.asarray(s, float)
        d = self.q1 - self.q0
        if order == 0:
            out = self.q0 + np.multiply.outer(s_arr, d)
        elif order == 1:
            out = np.broadcast_to(d, s_arr.shape + d.shape).copy()
        else:
            out = np.zeros(s_arr.shape + d.shape)
        return out

    @property
    def dof(self) -> int:
        return len(self.q0)

    @property
    def path_interval(self):
        return np.array([0.0, 1.0])

    @property
    def waypoints(self):
        return np.array([0.0, 1.0]), np.vstack([self.q0, self.q1])


def waypoint_path(waypoints):
    """C2 cubic-spline path through the joint-space waypoints, s_k = k/(K-1), with theta'' = 0 at the
    ends ("natural" spline).  The robot still starts and ends at rest because sdot = 0 there.
    (A "clamped" spline, theta' = 0 at the ends, would make m(s) = M theta' = 0 at s = 0 and 1:
    zero-inertia points, MR Sec. 9.4.4, which the discretised solver handles poorly.)"""
    import toppra as ta
    W = np.asarray(waypoints, float)
    return ta.SplineInterpolator(np.linspace(0.0, 1.0, len(W)), W, bc_type="natural")


def mr_2r_model(L1=1.0, L2=1.0, m1=1.0, m2=1.0, g=0.0) -> dict:
    """The MR 2R arm (MR Fig. 8.1, Eqs. 8.9-8.10): point masses at the link ends, gravity g along -y
    (g = 0: horizontal plane, as in MR Exercise 9.25)."""
    Mlist = [mr.RpToTrans(np.eye(3), [L1, 0, 0]), mr.RpToTrans(np.eye(3), [L2, 0, 0]), np.eye(4)]
    Glist = [np.diag([0, 0, 0, m1, m1, m1]).astype(float), np.diag([0, 0, 0, m2, m2, m2]).astype(float)]
    Slist = np.array([[0, 0, 1, 0, 0, 0], [0, 0, 1, 0, -L1, 0]], float).T
    return dict(name="MR 2R", n=2, Mlist=Mlist, Glist=Glist, Slist=Slist, g=np.array([0.0, -g, 0.0]))


def elbow_exo_model() -> dict:
    """Workshop 6 Problem 2 / Week 4 Workshop Problem 4: elbow exoskeleton + forearm + 1 kg bottle,
    vertical plane, theta = 0 with the forearm hanging down.  Lumped link: m = 3.0 kg, sum(m r) =
    0.698 kg m, inertia about the elbow 0.19734 kg m^2 (illustrative values)."""
    m = 1.6 + 0.4 + 1.0
    mr_sum = 1.6 * 0.18 + 0.4 * 0.15 + 1.0 * 0.35
    I_elbow = 0.012 + 1.6 * 0.18**2 + 0.002 + 0.4 * 0.15**2 + 1.0 * 0.35**2
    rc = mr_sum / m
    Ic = I_elbow - m * rc**2
    Mlist = [mr.RpToTrans(np.eye(3), [0, -rc, 0]), mr.RpToTrans(np.eye(3), [0, -(0.35 - rc), 0])]
    Glist = [np.diag([Ic, Ic, Ic, m, m, m])]
    Slist = np.array([[0, 0, 1, 0, 0, 0]], float).T
    return dict(name="elbow exoskeleton", n=1, Mlist=Mlist, Glist=Glist, Slist=Slist,
                g=np.array([0.0, -9.81, 0.0]))


def gantry_model(Mx=40.0, my=15.0) -> dict:
    """Workshop 6 Problem 3: X-Y gantry (two prismatic joints, horizontal plane) of a humanitarian-
    demining sensor head.  Joint 1 (x) moves Mx in total (its own carriage Mx - my plus the y axis);
    joint 2 (y) moves my.  Illustrative values."""
    Mlist = [np.eye(4), np.eye(4), np.eye(4)]
    Glist = [np.diag([0, 0, 0, Mx - my, Mx - my, Mx - my]).astype(float), np.diag([0, 0, 0, my, my, my]).astype(float)]
    Slist = np.array([[0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 1, 0]], float).T
    return dict(name="gantry", n=2, Mlist=Mlist, Glist=Glist, Slist=Slist, g=np.zeros(3))


class SemicirclePath(_PathBase):
    """MR Fig. 9.9: x = xc + R cos(pi s), y = yc - R sin(pi s), s in [0, 1]."""

    def __init__(self, R=0.3, xc=0.0, yc=0.0):
        self.R, self.xc, self.yc = R, xc, yc

    def __call__(self, s, order: int = 0):
        s = np.asarray(s, float)
        R, a = self.R, np.pi * s
        if order == 0:
            out = np.stack([self.xc + R * np.cos(a), self.yc - R * np.sin(a)], -1)
        elif order == 1:
            out = np.stack([-np.pi * R * np.sin(a), -np.pi * R * np.cos(a)], -1)
        else:
            out = np.stack([-np.pi**2 * R * np.cos(a), np.pi**2 * R * np.sin(a)], -1)
        return out

    @property
    def dof(self) -> int:
        return 2

    @property
    def path_interval(self):
        return np.array([0.0, 1.0])

    @property
    def waypoints(self):
        return np.array([0.0, 1.0]), self(np.array([0.0, 1.0]))


def course_arm_model(payload: float = 0.0) -> dict:
    """The course arm (the local model file course_arm.py, numbers identical to the course robot models)."""
    return _ca.course_arm(payload=payload)


def inverse_dynamics(model: dict, q, dq, ddq, gravity: bool = True) -> np.ndarray:
    """modern_robotics RNEA for a model dict; gravity=False sets g = 0."""
    g = model["g"] if gravity else np.zeros(3)
    return np.asarray(mr.InverseDynamics(np.asarray(q, float), np.asarray(dq, float), np.asarray(ddq, float),
                                         g, np.zeros(6), model["Mlist"], model["Glist"], model["Slist"]))


def load_task(name: str = "lab6_task.json") -> dict:
    """Task data for Part C (waypoints, limit fractions, payload, servo period)."""
    import json
    with open(DATA / name) as fh:
        return json.load(fh)


# =============================================================================================
# Part A -- the path-constrained dynamics and the motion cone (MR Secs. 9.4, 9.4.1)
# =============================================================================================
def path_dynamics(model: dict, path, s: float):
    """Return (m, c, g), each shape (n,), of MR Eq. 9.33 at path parameter s.

    Use three calls of the RNEA (no new dynamics code):
      g(s) = ID(theta, 0, 0)            with gravity,
      m(s) = ID(theta, 0, theta')       without gravity  (= M theta'),
      c(s) = ID(theta, theta', theta'') without gravity  (= M theta'' + theta'^T Gamma theta').
    """
    # TODO: three calls of inverse_dynamics(model, ...) as in the docstring; return m, c, g
    raise NotImplementedError("path_dynamics: TODO")


def accel_limits(m, c, g, sdot: float, tau_min, tau_max):
    """(L, U) of MR Eqs. 9.36-9.37 at one state: the smallest and largest sdd that satisfy every
    tau_min_i <= m_i sdd + c_i sdot^2 + g_i <= tau_max_i.

    If m_i < 0 the two bounds swap.  If |m_i| < EPS_M (zero-inertia point) joint i gives no bound on
    sdd, but if c_i sdot^2 + g_i lies outside [tau_min_i, tau_max_i] the state is inadmissible: return
    (L, U) = (+inf, -inf).  The state is admissible when L <= U.
    """
    # TODO: loop over joints; MR Eq. 9.36 (swap the bounds if m_i < 0); handle |m_i| < EPS_M;
    #       L = max_i L_i, U = min_i U_i (MR Eq. 9.37)
    raise NotImplementedError("accel_limits: TODO")


def velocity_limit(model: dict, path, s: float, tau_min, tau_max, qdot_max=None,
                   sdot_max: float = 100.0, tol: float = 1e-9) -> float:
    """sdot_lim(s): the largest sdot with L(s, sdot) <= U(s, sdot) (the velocity limit curve, MR Sec. 9.4.1),
    combined with the kinematic limit |theta'_i| sdot <= qdot_max_i if qdot_max is given.
    Bisection on sdot in [0, sdot_max] (assumes one admissible interval [0, sdot_lim], MR Sec. 9.4.4).
    Returns sdot_max if no limit is found below it."""
    # TODO: kinematic bound min_i qdot_max_i/|theta'_i|, then bisection on sdot with accel_limits
    raise NotImplementedError("velocity_limit: TODO")


# =============================================================================================
# Part B -- bang-bang time scaling (MR Sec. 9.4.1, Fig. 9.12a) and TOPP with toppra
# =============================================================================================
def _limits_x(model, path, s, x, tau_min, tau_max):
    m, c, g = path_dynamics(model, path, s)
    return accel_limits(m, c, g, np.sqrt(max(x, 0.0)), tau_min, tau_max)


def bang_bang(model: dict, path, tau_min, tau_max, N: int = 2000) -> dict:
    """Single-switch time-optimal scaling (MR Sec. 9.4.1, Fig. 9.12a).

    Work with x = sdot^2, for which dx/ds = 2 sdd (since d(sdot^2)/ds = 2 sdot dsdot/ds = 2 sdd).
    On the grid s_k = k/N:
      * curve A: integrate dx/ds = 2 U(s, sqrt x) forward from (0, 0)  (Heun / RK2);
      * curve F: integrate dx/ds = 2 L(s, sqrt x) backward from (1, 0) until s = 0 or until it
        penetrates the velocity limit curve (L > U), where F simply ends (MR Sec. 9.4.2, step 2);
      * the switch s* is where A meets F (linear interpolation of A - F between grid points);
      * the optimal profile is A before s* and F after it; the time is T = sum 2 ds / (sdot_k + sdot_k+1)
        (constant sdd on each interval, exact for MR's piecewise-constant-acceleration model).
    If A penetrates the velocity limit curve before it meets F, raise RuntimeError: a multi-switch
    solution (MR Sec. 9.4.2, steps 4-6) is needed -- use toppra.

    Returns dict(s, sdot, xA, xF, s_switch, sdot_switch, T).
    """
    # TODO: grid s_k = k/N; F: integrate dx/ds = 2L backward from x=0 at s=1 (stop where L > U);
    #       A: integrate dx/ds = 2U forward from x=0 at s=0 until A >= F (raise RuntimeError if L > U first);
    #       use _limits_x; interpolate the crossing; T = sum 2h/(sd_k + sd_k+1)
    raise NotImplementedError("bang_bang: TODO")


def topp(model: dict, path, tau_max, qdot_max=None, qddot_max=None, N: int = 200,
         tau_min=None) -> dict:
    """Time-optimal time scaling with toppra (TOPP-RA, a discretised, reachability-based solver of the
    same problem as MR Sec. 9.4).  Constraints: joint torques tau_min <= tau <= tau_max (default
    tau_min = -tau_max) via the model's RNEA, and optionally |qdot| <= qdot_max, |qddot| <= qddot_max.
    Start and end at rest.  Returns dict(T, traj, s_grid, sdot_grid) where traj(t, order) gives
    q, qdot or qddot, and sdot_grid are the optimal path velocities on the grid (the phase-plane curve).
    Raises RuntimeError if toppra finds no feasible time scaling."""
    # TODO: build toppra constraints (JointTorqueConstraint with inverse_dynamics, JointVelocityConstraint,
    #       JointAccelerationConstraint), algo.TOPPRA(..., gridpoints=..., parametrizer="ParametrizeConstAccel"),
    #       compute_trajectory(0, 0); read problem_data.sd_vec
    raise NotImplementedError("topp: TODO")


def quintic_s(t, T):
    """Quintic time scaling s(t), sdot, sdd (MR Sec. 9.2.2.1, Fig. 9.4)."""
    tau = np.clip(np.asarray(t, float) / T, 0.0, 1.0)
    s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
    sd = (30 * tau**2 - 60 * tau**3 + 30 * tau**4) / T
    sdd = (60 * tau - 180 * tau**2 + 120 * tau**3) / T**2
    return s, sd, sdd


def quintic_traj(path, T: float):
    """theta(s(t)) with a quintic s(t); returns a callable traj(t, order) like a toppra trajectory
    (order 0, 1, 2 via MR Eqs. 9.1-9.2)."""
    def traj(t, order: int = 0):
        s, sd, sdd = quintic_s(t, T)
        if order == 0:
            return path(s, 0)
        d1 = path(s, 1)
        if order == 1:
            return d1 * np.asarray(sd)[..., None]
        return d1 * np.asarray(sdd)[..., None] + path(s, 2) * np.asarray(sd)[..., None] ** 2
    traj.duration = T
    return traj


def quintic_min_time(model: dict, path, tau_max, qdot_max=None, qddot_max=None, N: int = 400,
                     tol: float = 1e-4, tau_min=None) -> float:
    """Smallest T for which the quintic time scaling of `path` satisfies the same limits as `topp`
    (checked at N+1 instants).  Bisection on T (feasibility is monotone in T when gravity can be held
    statically: slowing down moves the torques toward g(theta))."""
    # TODO: precompute m(s), c(s), g(s) at the quintic samples; feasible(T) checks torque, speed and
    #       acceleration limits; bracket and bisect on T
    raise NotImplementedError("quintic_min_time: TODO")


def peak_usage(model: dict, traj, T: float, tau_max, qdot_max, N: int = 1000) -> dict:
    """Provided.  Peak |qdot|/qdot_max and |tau|/tau_max per joint, and the peak |jerk| (finite
    differences of qddot) along a trajectory callable traj(t, order)."""
    t = np.linspace(0.0, T, N + 1)
    q, qd, qdd = traj(t, 0), traj(t, 1), traj(t, 2)
    tau = np.array([inverse_dynamics(model, a, b, c) for a, b, c in zip(q, qd, qdd)])
    jerk = np.diff(qdd, axis=0) / np.diff(t)[:, None]
    return dict(vel_ratio=np.max(np.abs(qd), 0) / np.asarray(qdot_max),
                tau_ratio=np.max(np.abs(tau), 0) / np.asarray(tau_max),
                jerk_max=float(np.max(np.abs(jerk))), tau=tau, t=t)


# =============================================================================================
# Part C -- hardware command file, safety check, and sim-vs-hardware metrics
# =============================================================================================
def command_table(traj, T: float, dt: float = 0.02, slow: float = 1.0):
    """Sample a trajectory for the servo stream.  `slow` >= 1 stretches time uniformly
    (q_cmd(t) = q(t / slow)), which divides all speeds by `slow` and accelerations by slow^2.
    Returns (t, q): t = 0, dt, ..., slow*T (the last sample is exactly slow*T), q shape (len(t), n)."""
    # TODO: t = 0, dt, ..., slow*T (last sample exactly slow*T); q = traj(t / slow, 0)
    raise NotImplementedError("command_table: TODO")


def check_command(t, q, model: dict, speed_fraction: float, margin_deg: float = 10.0,
                  tol: float = 0.01) -> dict:
    """Safety check of a command table before it goes to the arm (course-arm hardware safety checklist,
    rules 1 and 3; Course resources):
      * within_limits: every q stays at least `margin_deg` (checklist: 10 deg) inside the joint limits;
      * peak_speed_ratio: max over joints and samples of |dq/dt| / (speed_fraction * qdot_max)
        (finite differences of the samples);  speed_ok = ratio <= 1 + tol;
      * starts_at_rest / ends_at_rest: first and last steps are smaller than 1e-3 rad;
      * ok = all of the above.
    """
    # TODO: joint-limit margin, finite-difference speeds against speed_fraction*qdot_max, rest at both ends
    raise NotImplementedError("check_command: TODO")


def write_log(path, t, cmd, meas=None, cur=None) -> None:
    """Provided.  Write a CSV in the project log format (project starter `hardware.py`):
    t, cmd_1..cmd_n[, meas_1..meas_n][, cur_1..cur_n]."""
    t = np.asarray(t, float)
    cmd = np.asarray(cmd, float).reshape(len(t), -1)
    n = cmd.shape[1]
    cols, header = [t[:, None], cmd], ["t"] + [f"cmd_{j + 1}" for j in range(n)]
    if meas is not None:
        cols.append(np.asarray(meas, float).reshape(len(t), -1))
        header += [f"meas_{j + 1}" for j in range(n)]
    if cur is not None:
        cols.append(np.asarray(cur, float).reshape(len(t), -1))
        header += [f"cur_{j + 1}" for j in range(n)]
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(header)
        w.writerows(np.hstack(cols).tolist())


def load_log(path) -> dict:
    """Provided.  Read a log written by `write_log` (or by the station logger)."""
    with open(path, newline="") as fh:
        header = [h.strip() for h in next(csv.reader(fh))]
    data = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    col = {h: i for i, h in enumerate(header)}

    def group(prefix):
        names = sorted((h for h in header if h.startswith(prefix + "_")), key=lambda h: int(h.split("_")[1]))
        return data[:, [col[h] for h in names]] if names else None

    return dict(t=data[:, col["t"]], cmd=group("cmd"), meas=group("meas"), cur=group("cur"))


def simulate_servo(t, cmd, wn: float = 25.0, zeta: float = 0.9, delay: float = 0.02,
                   enc_res: float = 2 * np.pi / 4096) -> np.ndarray:
    """Provided (simulation fallback).  Model of a position-controlled smart servo: pure delay, then a
    second-order loop  qdd = wn^2 (q_cmd(t - delay) - q) - 2 zeta wn qd  (semi-implicit Euler on a
    1 ms grid), then 12-bit encoder quantisation.  Illustrative values; identify wn, zeta and delay
    from a step response on your unit (project brief Sec. 5.1)."""
    t = np.asarray(t, float)
    cmd = np.asarray(cmd, float).reshape(len(t), -1)
    h = 1e-3
    tf = np.arange(t[0], t[-1] + h / 2, h)
    cf = np.column_stack([np.interp(tf - delay, t, cmd[:, j], left=cmd[0, j]) for j in range(cmd.shape[1])])
    q = cmd[0].copy()
    qd = np.zeros_like(q)
    out = np.empty_like(cf)
    for k in range(len(tf)):
        out[k] = q
        qdd = wn**2 * (cf[k] - q) - 2 * zeta * wn * qd
        qd = qd + h * qdd
        q = q + h * qd
    meas = np.column_stack([np.interp(t, tf, out[:, j]) for j in range(cmd.shape[1])])
    return np.round(meas / enc_res) * enc_res


def tracking_metrics(t, cmd, meas, max_lag: float = 0.2) -> dict:
    """Sim-vs-hardware metrics of one run (HWk-2 style):
      * rms (n,): RMS of meas - cmd per joint [rad];   * max_abs (n,): max |meas - cmd| [rad];
      * lag: the time shift d in [0, max_lag] (multiple of the sample time) that minimises the total
        RMS of meas(t) - cmd(t - d), i.e. how far the measured motion runs behind the command [s];
      * rms_after_lag (n,): the RMS error after removing that lag.
    Assumes a uniform time grid."""
    # TODO: errors, RMS and max per joint; search the shift k*dt in [0, max_lag] minimising the RMS
    raise NotImplementedError("tracking_metrics: TODO")
