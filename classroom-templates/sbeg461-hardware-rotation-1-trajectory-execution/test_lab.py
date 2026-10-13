"""Auto-checks for SBEG 461 Hardware rotation 1 (Week 8): command file, dry run, servo identification,
gravity check, sim-vs-hardware comparison, safe stop and the pre-flight record.

Usage:   LAB_IMPL=starter  python -m pytest -q test_lab.py     (your work, file starter.py; always set LAB_IMPL)
Ground truth: closed forms (quintic peaks, second-order step response, Astrom-Murray Table 6.1),
modern_robotics (RNEA), synthetic signals with known answers, and the synthetic station logs in data/
(made by the course team; the reference values are quoted in the lab handout checkpoints).
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

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
lab = importlib.import_module(os.environ.get("LAB_IMPL", "starter"))
ref = lab                                       # provided helpers (logs, simulator) are in starter.py too


@pytest.fixture(scope="module")
def task():
    return ref.load_task()


@pytest.fixture(scope="module")
def model(task):
    return ref.course_arm_model(task["payload_kg"])


@pytest.fixture(scope="module")
def cmd(task, model):
    v = task["first_run_speed_fraction"] * model["qdot_max"]
    return lab.waypoint_command(task["waypoints"], v, task["qddot_max"], task["servo_period_s"], task["dwell_s"])


@pytest.fixture(scope="module")
def steps(task):
    L = ref.load_log(ref.DATA / "step_log.csv")
    out = []
    for j, (a, b) in zip(task["step_test"]["joints"], task["step_test"]["windows_s"]):
        m = (L["t"] >= a - 1e-9) & (L["t"] <= b + 1e-9)
        out.append((L["t"][m], L["cmd"][m, j - 1], L["meas"][m, j - 1]))
    return out


# ------------------------------------------------------------------ Part A: pre-flight
def test_servo_model_conversion(task):
    w0 = np.array(task["waypoints"][0])
    off = task["servo_zero_offset_deg"]
    s = lab.model_to_servo(w0, off)
    assert np.allclose(s, [1.5, -36.3775, 80.7141, -137.5099, -90.9544, 0.0], atol=1e-4)
    assert np.allclose(lab.servo_to_model(s, off), w0, atol=1e-12)


def test_segment_time_hand_values(task, model):
    W = np.array(task["waypoints"])
    v = 0.25 * model["qdot_max"]
    T = [lab.segment_time(W[k + 1] - W[k], v, 2.0, 0.02) for k in range(3)]
    assert np.allclose(T, [2.42, 1.62, 4.04], atol=1e-9)
    # first segment: speed-limited, 15/8 * 0.9 / 0.6981 = 2.417 s, rounded up to 2.42 s
    assert math.isclose(15 / 8 * 0.9 / v[0], 2.4172, abs_tol=1e-4)
    # acceleration-limited case: tiny speed limit removed
    assert math.isclose(lab.segment_time([1.0], [100.0], [1.0], 1e-6), math.sqrt(10 / math.sqrt(3)), abs_tol=1e-5)


def test_waypoint_command_structure(task, cmd):
    t, q, qd = cmd["t"], cmd["q"], cmd["qd"]
    assert len(t) == 505 and math.isclose(t[-1], 10.08, abs_tol=1e-9)
    assert np.allclose(np.diff(t), 0.02)
    assert np.allclose(cmd["seg_T"], [2.42, 1.62, 4.04]) and np.allclose(cmd["t_start"], [0.5, 3.42, 5.54])
    W = np.array(task["waypoints"])
    for k, ts in enumerate(cmd["t_start"]):             # at rest at every waypoint
        i = int(round(ts / 0.02))
        assert np.allclose(q[i], W[k]) and np.allclose(qd[i], 0)
    assert np.allclose(q[-1], W[-1])
    fd = np.gradient(q, t, axis=0)                       # analytic qd agrees with the samples
    assert np.max(np.abs(fd - qd)) < 0.02


def test_command_passes_first_run_check(cmd, model):
    c = ref.check_command(cmd["t"], cmd["q"], model, 0.25, 10.0)
    assert c["ok"] and 0.99 < c["peak_speed_ratio"] <= 1.0
    assert not ref.check_command(cmd["t"], cmd["q"], model, 0.2, 10.0)["ok"]


def test_dry_run(model, cmd):
    d = lab.dry_run(model, cmd, 0.25, 0.6)
    k = 150                                               # one sample checked directly with modern_robotics
    tau = mr.InverseDynamics(cmd["q"][k], cmd["qd"][k], cmd["qdd"][k], model["g"], np.zeros(6),
                             model["Mlist"], model["Glist"], model["Slist"])
    assert np.allclose(d["tau"][k], tau, atol=1e-10)
    assert np.allclose(d["peak_tau"][:4], [0.0257, 0.8916, 0.3828, 0.1302], atol=2e-4)
    assert math.isclose(d["torque_ratio"][1], 0.4953, abs_tol=2e-4)
    assert d["ok"] and np.max(d["speed_ratio"]) <= 1.0
    assert not lab.dry_run(model, cmd, 0.25, 0.25)["ok"]  # 25 % torque: joint 2 would exceed


# ------------------------------------------------------------------ Part B: step test, identification
def test_zeta_wn_from_step_closed_form():
    zeta, wn = 0.6, 20.0
    Mp = math.exp(-math.pi * zeta / math.sqrt(1 - zeta**2))
    tp = math.pi / (wn * math.sqrt(1 - zeta**2))
    z, w = lab.zeta_wn_from_step(Mp, tp)
    assert math.isclose(z, 0.6, abs_tol=1e-12) and math.isclose(w, 20.0, abs_tol=1e-9)
    assert math.isclose(lab.zeta_wn_from_step(0.16, 0.2)[0], 0.5038, abs_tol=1e-4)   # A-M Table 6.1: 16 % <-> 0.5


def test_step_response_matches_servo_ode():
    t = np.arange(0, 1.0, 0.002)
    c = np.where(t >= 0.1, 1.0, 0.0)[:, None]
    y = ref.simulate_servo(t, c, 15.0, 0.4, 0.0, enc_res=0)[:, 0]
    assert np.max(np.abs(y - ref.step_response(t - 0.1, 15.0, 0.4))) < 0.01


def test_step_features_synthetic():
    t = np.arange(0, 2.0, 0.001)
    c = np.where(t >= 0.5, 0.05, 0.0)
    y = 0.05 * ref.step_response(t - 0.53, 20.0, 0.6)
    f = lab.step_features(t, c, y)
    assert math.isclose(f["t_step"], 0.5, abs_tol=1e-9) and math.isclose(f["A"], 0.05)
    assert math.isclose(f["overshoot"], math.exp(-0.6 * math.pi / 0.8), abs_tol=2e-4)
    assert math.isclose(f["t_peak_abs"] - 0.53, math.pi / 16, abs_tol=2e-3)
    assert 0.53 < f["t_move"] < 0.53 + 0.06


def test_identify_servo_noise_free():
    t = np.arange(0, 1.5, 0.02)
    c = np.where(t >= 0.4, 0.06, 0.0)[:, None]
    y = ref.simulate_servo(t, c, 18.0, 0.5, 0.04, enc_res=0)[:, 0]
    r = lab.identify_servo(t, c[:, 0], y)
    assert abs(r["wn"] - 18.0) < 0.4 and abs(r["zeta"] - 0.5) < 0.02 and abs(r["delay"] - 0.04) < 0.004


def test_identify_servo_station_log(steps):
    r2 = lab.identify_servo(*steps[0])
    r3 = lab.identify_servo(*steps[1])
    for r, (wn, z, d) in ((r2, (19.75, 0.595, 0.0288)), (r3, (19.98, 0.602, 0.0297))):
        assert abs(r["wn"] - wn) < 0.3 and abs(r["zeta"] - z) < 0.01 and abs(r["delay"] - d) < 0.002
        assert r["rms"] < 5e-4


# ------------------------------------------------------------------ Part C: static poses
def test_torque_constant_and_normalised_rms():
    i = np.array([0.2, 0.4, 0.6])
    assert math.isclose(lab.torque_constant(i, 1.25 * i), 1.25)
    assert math.isclose(lab.normalised_rms([1, 2, 3], [1, 2, 3]), 0.0)
    assert math.isclose(lab.normalised_rms([0, 0, 0], [1, -1, 1]), 1.0)
    assert math.isclose(lab.normalised_rms([1.1, 2.2], [1.0, 2.0]), 0.1)


def test_loop_stiffness():
    tau = np.array([-0.9, -0.6, -0.3])
    assert math.isclose(lab.loop_stiffness(tau / 30.0, tau), 30.0)


def test_gravity_check_station_log(task, model):
    st = ref.average_approaches(ref.load_log(ref.DATA / "static_log.csv"))
    gc = lab.gravity_check(st, model, fit=task["static_fit_poses"], check=task["static_check_poses"])
    g = np.array([mr.InverseDynamics(q, np.zeros(6), np.zeros(6), model["g"], np.zeros(6), model["Mlist"],
                                     model["Glist"], model["Slist"]) for q in st["meas"]])
    assert np.allclose(gc["g"], g, atol=1e-10)
    assert np.allclose(gc["k"], [1.1839, 1.1635, 0.4487], atol=2e-3)
    assert np.allclose(gc["K"], [28.66, 13.95, 5.356], atol=0.05)
    assert np.all(gc["nrms"] < 0.3)


# ------------------------------------------------------------------ Part D: comparison, stop, pre-flight
def test_compare_runs_synthetic():
    t = np.linspace(0, 2, 101)
    c = np.column_stack([np.sin(t), np.cos(t)])
    out = lab.compare_runs(t, c, c + [0.01, -0.02], c + [0.01, 0.0])
    assert np.allclose(out["rms_cmd_meas"], [0.01, 0.02]) and np.allclose(out["max_cmd_meas"], [0.01, 0.02])
    assert np.allclose(out["rms_sim_meas"], [0.0, 0.02], atol=1e-12)
    lagged = np.column_stack([np.sin(t - 0.08), np.cos(t - 0.08)])
    assert math.isclose(lab.compare_runs(t, c, lagged, c)["lag"], 0.08, abs_tol=1e-9)


def test_identified_model_beats_nominal(task, model, steps):
    T = ref.load_log(ref.DATA / "traj_log.csv")
    ids = [lab.identify_servo(*s) for s in steps]
    wn, z, d = (np.mean([r[k] for r in ids]) for k in ("wn", "zeta", "delay"))
    st = ref.average_approaches(ref.load_log(ref.DATA / "static_log.csv"))
    K = np.full(6, 1e9)
    K[1:4] = lab.gravity_check(st, model)["K"]
    nom = lab.compare_runs(T["t"], T["cmd"], T["meas"], ref.simulate_servo(T["t"], T["cmd"], **ref.NOMINAL_SERVO))
    idm = lab.compare_runs(T["t"], T["cmd"], T["meas"],
                           ref.simulate_servo(T["t"], T["cmd"], wn, z, d, stiffness=K, model=model))
    assert math.isclose(nom["lag"], 0.10, abs_tol=1e-9)
    assert np.allclose(nom["rms_cmd_meas"][:3], [0.0395, 0.0329, 0.0332], atol=3e-4)
    assert np.allclose(nom["rms_sim_meas"][1:4], [0.0282, 0.0261, 0.0243], atol=3e-4)
    assert np.all(idm["rms_sim_meas"] < 1e-3) and np.all(idm["rms_sim_meas"][1:4] < nom["rms_sim_meas"][1:4] / 20)


def test_stop_time(task):
    t = np.arange(0, 2, 0.02)
    q = np.where(t < 1.0, 0.5 * t, 0.5)[:, None]          # moves at 0.5 rad/s, stops dead at t = 1
    # window 0.1 s -> +-2 samples: the last centre sample that still sees motion is t = 1.02 s
    assert math.isclose(lab.stop_time(t, q, 0.9), 0.14, abs_tol=1e-9)
    P = ref.load_log(ref.DATA / "stop_log.csv")
    ts = lab.stop_time(P["t"], P["meas"], task["stop_test"]["t_trigger_s"])
    assert math.isclose(ts, 0.34, abs_tol=1e-9) and ts < task["stop_test"]["pass_s"]
    assert lab.stop_time(t, (0.5 * t)[:, None], 0.5) == math.inf


def _good_record():
    return dict(members_at_arm=["Salma", "Omar"], inducted=["Salma", "Omar", "Youssef"], supervisor="TA K.",
                estop_tested=True, estop_holder="Omar", keepout_clear=True, workspace_ready=True,
                first_run=True, check={"ok": True, "peak_speed_ratio": 0.999}, check_speed_fraction=0.25,
                margin_deg=10.0, dry_run_speed_ratio=0.999, dry_run_torque_ratio=0.50,
                current_limit_fraction=0.6, payload_kg=0.1)


def test_preflight():
    assert lab.preflight(_good_record()) == []
    with open(ref.DATA / "session_record_template.json") as fh:
        assert len(lab.preflight(json.load(fh))) == len(ref.PREFLIGHT_RULES)   # an empty record fails all
    cases = [("inducted", ["Salma"], 0), ("supervisor", " ", 1), ("estop_tested", False, 2),
             ("estop_holder", "Youssef", 3), ("keepout_clear", False, 4), ("workspace_ready", False, 5),
             ("check_speed_fraction", 0.5, 6), ("margin_deg", 5.0, 6), ("dry_run_torque_ratio", 1.2, 7),
             ("current_limit_fraction", 0.8, 8), ("payload_kg", 0.2, 9), ("payload_kg", None, 9)]
    for key, val, rule in cases:
        r = _good_record()
        r[key] = val
        assert lab.preflight(r) == [ref.PREFLIGHT_RULES[rule]], key
    r = _good_record()
    r.update(first_run=False, check_speed_fraction=0.5)   # a repeat run may be checked at 50 %
    assert lab.preflight(r) == []
    r = _good_record()
    r["payload_kg"] = 0.125                               # rule A4: exactly 0.125 kg is allowed
    assert lab.preflight(r) == []


def test_joint_limit_margin_is_checklist_value(model):
    """Rule A1 (brief Sec. 5.2, shared checklist): software limits >= 10 deg inside the end stops (<= +-155 deg, J6 <= +-165 deg)."""
    import inspect
    m = ref._ca.COURSE_ARM_SAFETY["soft_limit_margin_deg"]
    assert m == 10.0
    assert inspect.signature(ref.check_command).parameters["margin_deg"].default == m
    assert np.allclose(np.rad2deg(model["limits"][:, 1]) - m, [155] * 5 + [165])
    t = np.arange(0, 1.02, 0.02)
    for deg, ok in ((150.0, True), (157.0, False)):      # 157 deg: inside a 5 deg margin, outside 10 deg
        q = np.zeros((len(t), 6))
        q[:, 0] = np.deg2rad(deg)
        assert ref.check_command(t, q, model, 0.25)["within_limits"] is ok, deg
