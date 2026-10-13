"""Auto-checks for SBEG 461 Lab 6 (time-optimal time scaling, TOPP vs quintic, hardware command file).

Usage:   python -m pytest -q test_lab.py      (your work, file starter.py; default, same as LAB_IMPL=starter)
Ground truth: modern_robotics (RNEA), the closed-form MR 2R dynamics (MR Eqs. 8.9-8.10), closed-form
phase-plane results of Workshop 6 (elbow exoskeleton, demining gantry), and toppra.
"""
import importlib
import math
import os
import sys
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
lab = importlib.import_module(os.environ.get("LAB_IMPL", "starter"))
import course_arm as CA  # noqa: E402  (the local course-arm model file)

PI = math.pi
TAU_2R = np.array([20.0, 10.0])
F_GANTRY = np.array([120.0, 60.0])


@pytest.fixture(scope="module")
def two_r():
    return lab.mr_2r_model(), lab.StraightLinePath([0.0, 0.0], [PI / 2, PI / 2])


@pytest.fixture(scope="module")
def gantry():
    return lab.gantry_model(), lab.SemicirclePath(0.3)


@pytest.fixture(scope="module")
def elbow():
    return lab.elbow_exo_model(), lab.StraightLinePath([0.0], [2 * PI / 3])


@pytest.fixture(scope="module")
def arm_task():
    task = lab.load_task()
    r = lab.course_arm_model(task["payload_kg"])
    return task, r, lab.waypoint_path(task["waypoints"])


@pytest.fixture(scope="module")
def hw_topp(arm_task):
    task, r, path = arm_task
    tl = task["torque_fraction"] * r["tau_max"]
    return lab.topp(r, path, tl, qdot_max=task["hw_speed_fraction"] * r["qdot_max"],
                    qddot_max=task["hw_qddot_max"], N=task["grid_N"])


def M2(th):
    c2 = math.cos(th[1])
    return np.array([[3 + 2 * c2, 1 + c2], [1 + c2, 1.0]])


def c2r(th, dth):
    s2 = math.sin(th[1])
    return np.array([-s2 * (2 * dth[0] * dth[1] + dth[1] ** 2), s2 * dth[0] ** 2])


# ------------------------------------------------------------------ Part A: path dynamics, cone, VLC

@pytest.mark.parametrize("s", [0.2, 0.5, 0.9])
def test_path_dynamics_2r_closed_form(two_r, s):
    """Straight line: theta'' = 0, so m = M(theta) Delta and c = c(theta, Delta) (MR Eqs. 9.32, 8.9-8.10)."""
    model, path = two_r
    m, c, g = lab.path_dynamics(model, path, s)
    D = np.array([PI / 2, PI / 2])
    th = s * D
    assert np.allclose(m, M2(th) @ D, atol=1e-9)
    assert np.allclose(c, c2r(th, D), atol=1e-9)
    assert np.allclose(g, 0.0, atol=1e-12)


def test_path_dynamics_reproduces_rnea_on_course_arm(arm_task):
    """m sdd + c sdot^2 + g = ID(theta, theta' sdot, theta' sdd + theta'' sdot^2) for any (sdot, sdd)."""
    _, r, path = arm_task
    rng = np.random.default_rng(6)
    for s in (0.13, 0.5, 0.77):
        m, c, g = lab.path_dynamics(r, path, s)
        for _ in range(3):
            sd, sdd = rng.uniform(0, 2), rng.uniform(-5, 5)
            th, d1, d2 = path(s, 0), path(s, 1), path(s, 2)
            tau = lab.inverse_dynamics(r, th, d1 * sd, d1 * sdd + d2 * sd**2)
            assert np.allclose(m * sdd + c * sd**2 + g, tau, atol=1e-10)


def test_accel_limits_quiz_state():
    """Quiz 5 Q2: m = (2, -0.5), c = (0.4, 0.3), g = (1, 0), |tau| <= (5, 2), sdot = 2 -> L = -1.6, U = 1.2;
    sdot = 3 is inadmissible (sdot_lim^2 = 7.5)."""
    m, c, g, t = [2.0, -0.5], [0.4, 0.3], [1.0, 0.0], np.array([5.0, 2.0])
    L, U = lab.accel_limits(m, c, g, 2.0, -t, t)
    assert L == pytest.approx(-1.6) and U == pytest.approx(1.2)
    L, U = lab.accel_limits(m, c, g, 3.0, -t, t)
    assert L > U
    L, U = lab.accel_limits(m, c, g, math.sqrt(7.5) - 1e-6, -t, t)
    assert L <= U


def test_accel_limits_zero_inertia():
    """A zero component of m(s) gives a pure velocity constraint (MR Sec. 9.4.4)."""
    m, c, g, t = [0.0, 2.0], [1.0, 0.0], [0.0, 0.0], np.array([1.0, 1.0])
    L, U = lab.accel_limits(m, c, g, 0.9, -t, t)
    assert L == pytest.approx(-0.5) and U == pytest.approx(0.5)
    L, U = lab.accel_limits(m, c, g, 1.1, -t, t)
    assert L > U


def test_velocity_limit_2r_workshop(two_r):
    """Workshop 6 Problem 4: at s = 0.5 the VLC is where the lower bound of joint 1 meets the upper bound
    of joint 2: (-20 - c1 x)/m1 = (10 - c2 x)/m2, x = sdot^2."""
    model, path = two_r
    m, c, _ = lab.path_dynamics(model, path, 0.5)
    x = (10 / m[1] + 20 / m[0]) / (c[1] / m[1] - c[0] / m[0])
    assert lab.velocity_limit(model, path, 0.5, -TAU_2R, TAU_2R) == pytest.approx(math.sqrt(x), abs=1e-6)
    assert math.sqrt(x) == pytest.approx(2.1546, abs=1e-4)


def test_velocity_limit_gantry_closed_form(gantry):
    """Workshop 6 Problem 3 (MR Fig. 9.9 path): closed-form VLC values at s = 0, 0.25, 0.5."""
    model, path = gantry
    R, Mx, my, fx, fy = 0.3, 40.0, 15.0, 120.0, 60.0
    a = math.sqrt(0.5)
    A1, A2 = fx / (Mx * PI * R * a), fy / (my * PI * R * a)
    vl = lambda s: lab.velocity_limit(model, path, s, -F_GANTRY, F_GANTRY)
    assert vl(0.25) == pytest.approx(math.sqrt((A1 + A2) / (2 * PI)), abs=1e-6)
    assert vl(0.5) == pytest.approx(math.sqrt(fy / (my * PI**2 * R)), abs=1e-6)
    assert vl(0.0) == pytest.approx(math.sqrt(fx / (Mx * PI**2 * R)), abs=1e-6)


def test_velocity_limit_kinematic(two_r):
    model, path = two_r
    big = np.array([1e6, 1e6])
    v = lab.velocity_limit(model, path, 0.3, -big, big, qdot_max=[2.0, 2.0])
    assert v == pytest.approx(2 / (PI / 2), abs=1e-6)


# ------------------------------------------------------------------ Part B: bang-bang and TOPP

def test_bang_bang_elbow_closed_form(elbow):
    """Workshop 6 Problem 2: s* = 1/2 + k(1 - cos D)/(2 tau D); T from the closed-form phase curves."""
    model, path = elbow
    I, k, D, tm = 0.19734, 0.698 * 9.81, 2 * PI / 3, 10.0
    out = lab.bang_bang(model, path, [-tm], [tm], N=2000)
    s_star = 0.5 + k * (1 - math.cos(D)) / (2 * tm * D)
    x_star = 2 / (I * D) * (tm * s_star - k / D * (1 - math.cos(D * s_star)))
    assert out["s_switch"] == pytest.approx(s_star, abs=1e-4)
    assert out["sdot_switch"] == pytest.approx(math.sqrt(x_star), rel=1e-4)
    assert out["T"] == pytest.approx(0.388528, rel=2e-3)


def test_bang_bang_matches_toppra_2r(two_r):
    model, path = two_r
    bb = lab.bang_bang(model, path, -TAU_2R, TAU_2R, N=2000)
    tp = lab.topp(model, path, TAU_2R, N=400)
    assert bb["T"] == pytest.approx(1.3683, abs=2e-3)
    assert tp["T"] == pytest.approx(bb["T"], rel=0.01)


def test_bang_bang_detects_velocity_limit_curve(gantry):
    """On the semicircle the forward curve hits the VLC: one switch is not enough (MR Fig. 9.12b)."""
    model, path = gantry
    try:
        lab.bang_bang(model, path, -F_GANTRY, F_GANTRY, N=1000)
    except NotImplementedError:            # a subclass of RuntimeError: an unfinished starter must fail
        raise
    except RuntimeError:
        return
    pytest.fail("bang_bang should raise RuntimeError when the forward curve hits the velocity limit curve")


def test_topp_gantry(gantry):
    model, path = gantry
    tp = lab.topp(model, path, F_GANTRY, N=400)
    assert tp["T"] == pytest.approx(1.1655, abs=3e-3)
    sd = np.interp(0.5, tp["s_grid"], tp["sdot_grid"])
    assert sd <= lab.velocity_limit(model, path, 0.5, -F_GANTRY, F_GANTRY) + 1e-3


def test_topp_torque_scaling(two_r):
    """g = 0 and pure torque limits: scaling the limits by k scales T by 1/sqrt(k) (Workshop 6 P5)."""
    model, path = two_r
    T1 = lab.topp(model, path, TAU_2R, N=300)["T"]
    T2 = lab.topp(model, path, 0.6 * TAU_2R, N=300)["T"]
    assert T2 == pytest.approx(T1 / math.sqrt(0.6), rel=5e-3)


def test_topp_course_arm_respects_limits(arm_task, hw_topp):
    task, r, _ = arm_task
    tl = task["torque_fraction"] * r["tau_max"]
    vmax = task["hw_speed_fraction"] * r["qdot_max"]
    pu = lab.peak_usage(r, hw_topp["traj"], hw_topp["T"], tl, vmax, N=2000)
    assert np.all(pu["vel_ratio"] <= 1.01)
    assert np.all(pu["tau_ratio"] <= 1.03)
    t = np.linspace(0, hw_topp["T"], 2001)
    assert np.max(np.abs(hw_topp["traj"](t, 2))) <= task["hw_qddot_max"] * 1.01
    assert hw_topp["T"] == pytest.approx(1.5656, abs=0.01)
    q = hw_topp["traj"](np.array([0.0, hw_topp["T"]]), 0)
    assert np.allclose(q, np.array(task["waypoints"])[[0, -1]], atol=1e-6)


def test_quintic_min_time_elbow(elbow):
    """Torque-limited quintic: 0.5231 s; speed-limited (3 rad/s): T = (15/8) D / 3 = 1.3090 s."""
    model, path = elbow
    assert lab.quintic_min_time(model, path, [10.0]) == pytest.approx(0.52312, abs=1e-3)
    T = lab.quintic_min_time(model, path, [10.0], qdot_max=[3.0])
    assert T == pytest.approx(15 / 8 * (2 * PI / 3) / 3, abs=1e-3)


def test_topp_beats_quintic_on_course_arm(arm_task, hw_topp):
    task, r, path = arm_task
    tl = task["torque_fraction"] * r["tau_max"]
    Tq = lab.quintic_min_time(r, path, tl, qdot_max=task["hw_speed_fraction"] * r["qdot_max"],
                              qddot_max=task["hw_qddot_max"])
    assert Tq == pytest.approx(2.2964, abs=0.01)
    assert 1.2 < Tq / hw_topp["T"] < 2.0


# ------------------------------------------------------------------ Part C: hardware command file and metrics

def test_command_table(arm_task, hw_topp):
    task, r, _ = arm_task
    dt = task["servo_period_s"]
    t, q = lab.command_table(hw_topp["traj"], hw_topp["T"], dt=dt, slow=2.0)
    assert t[0] == 0.0 and t[-1] == pytest.approx(2 * hw_topp["T"])
    assert np.all(np.diff(t) <= dt + 1e-12) and np.all(np.diff(t) > 0)
    assert np.allclose(q[0], task["waypoints"][0], atol=1e-9)
    assert np.allclose(q[-1], task["waypoints"][-1], atol=1e-9)
    assert q.shape == (len(t), 6)


def test_check_command(arm_task, hw_topp):
    task, r, _ = arm_task
    dt = task["servo_period_s"]
    t, q = lab.command_table(hw_topp["traj"], hw_topp["T"], dt=dt, slow=2.0)
    ok = lab.check_command(t, q, r, task["first_run_speed_fraction"], task["joint_margin_deg"])
    assert ok["ok"] and ok["within_limits"] and ok["starts_at_rest"] and ok["ends_at_rest"]
    assert 0.9 < ok["peak_speed_ratio"] <= 1.01
    t1, q1 = lab.command_table(hw_topp["traj"], hw_topp["T"], dt=dt, slow=1.0)
    bad = lab.check_command(t1, q1, r, task["first_run_speed_fraction"], task["joint_margin_deg"])
    assert not bad["ok"] and bad["peak_speed_ratio"] > 1.8
    q2 = q.copy()
    q2[:, 1] += np.deg2rad(165 - 8) - q2[:, 1].max()     # push joint 2 to 8 deg from its end stop
    assert not lab.check_command(t, q2, r, 1.0, task["joint_margin_deg"])["within_limits"]
    assert not lab.check_command(t, q2, r, 1.0)["within_limits"]          # default margin = 10 deg
    assert lab.check_command(t, q2, r, 1.0, 5.0)["within_limits"]         # a 5 deg margin would let it pass


def test_task_follows_safety_checklist():
    """The Week 6 hardware run uses the course-arm checklist values (Course resources)."""
    task = lab.load_task()
    s = CA.COURSE_ARM_SAFETY
    assert task["joint_margin_deg"] == s["soft_limit_margin_deg"] == 10.0
    assert task["first_run_speed_fraction"] == s["speed_fraction_first_run"] == 0.25
    assert task["hw_speed_fraction"] == s["speed_fraction"] == 0.5
    assert task["payload_kg"] <= s["payload_max_kg"] == pytest.approx(0.125)
    assert task["servo_period_s"] == 0.02
    assert task["hw_speed_fraction"] / task["first_run_speed_fraction"] == 2.0   # stretch factor slow = 2


def test_tracking_metrics_synthetic():
    t = np.arange(0, 4, 0.01)
    cmd = np.column_stack([np.sin(t), 0.5 * np.cos(2 * t)])
    meas = np.column_stack([np.sin(t - 0.06), 0.5 * np.cos(2 * (t - 0.06))])
    out = lab.tracking_metrics(t, cmd, meas)
    assert out["lag"] == pytest.approx(0.06, abs=1e-9)
    assert np.allclose(out["rms"], np.sqrt(np.mean((meas - cmd) ** 2, axis=0)))
    assert np.allclose(out["max_abs"], np.max(np.abs(meas - cmd), axis=0))
    assert np.all(out["rms_after_lag"] < 1e-9)


def test_tracking_metrics_on_fallback_log():
    log = lab.load_log(lab.DATA / "sim_servo_log.csv")
    out = lab.tracking_metrics(log["t"], log["cmd"], log["meas"])
    assert 0.06 <= out["lag"] <= 0.12           # ~ delay + 2 zeta / wn = 0.092 s
    assert np.all(out["rms_after_lag"] <= out["rms"] + 1e-12)
    assert out["rms"].max() == pytest.approx(0.0419, abs=2e-3)

