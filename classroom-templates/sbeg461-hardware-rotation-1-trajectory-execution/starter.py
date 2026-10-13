"""SBEG 461 Hardware rotation 1 (Week 8) -- trajectory execution and logging on the course arm,
servo identification and the simulation-versus-hardware comparison.  STARTER: complete every function marked TODO.

These functions become `sbeg461_dynamics/hwrun.py` in your cumulative library.  The project
(`project/starter/sbeg461_project/hardware.py`) uses the same log format and the same metrics, so the
code you write today is the code you use in the two project hardware sessions (Weeks 10-13).

What runs on the arm.  The course arm has position-controlled smart servos (project brief Sec. 5.1).
A command file of joint angles is streamed every 20 ms; the servo's own loop tracks it.  We model one
joint of that loop as a pure delay followed by a second-order system (Astrom-Murray Eq. 6.22)

    q_dd = wn^2 (q_cmd(t - delay) - q) - 2 zeta wn q_d ,

plus a static sag  q_meas = q - g(q) / K  caused by the finite stiffness K (N m/rad) of the position loop
under the gravity torque g(q) (MR Eq. 8.1 with theta_dot = theta_dd = 0), plus 12-bit encoder quantisation.
All device numbers are ILLUSTRATIVE VALUES; identify them on your own unit.

Notation: Modern Robotics (MR).  Units: SI (s, rad, rad/s, N m, A) unless a name ends in _deg.
"""
from __future__ import annotations

import csv
import json
from pathlib import Path

import modern_robotics as mr
import numpy as np

import course_arm as _ca

HERE = Path(__file__).resolve().parent
DATA = HERE / "data"
DT = 0.02                                    # servo stream / logger period (s), 50 Hz
ENC_RES = 2 * np.pi / 4096                   # 12-bit encoder (rad)
NOMINAL_SERVO = dict(wn=25.0, zeta=0.9, delay=0.02)   # the Lab 6 servo model (not identified)
QUINTIC_V = 15.0 / 8.0                       # peak of ds/dr for the quintic 10r^3 - 15r^4 + 6r^5
QUINTIC_A = 10.0 / np.sqrt(3.0)              # peak of |d2s/dr2| for the quintic


# =============================================================================================
# Provided: model, task, logs, servo simulator
# =============================================================================================
def course_arm_model(payload: float = 0.0) -> dict:
    """The course arm (the lab's own copy of the shared course-arm model, see course_arm.py)."""
    return _ca.course_arm(payload=payload)


def inverse_dynamics(model: dict, q, dq, ddq, gravity: bool = True) -> np.ndarray:
    """modern_robotics RNEA (MR Eqs. 8.50-8.54) for a model dict; gravity=False sets g = 0."""
    g = model["g"] if gravity else np.zeros(3)
    return np.asarray(mr.InverseDynamics(np.asarray(q, float), np.asarray(dq, float),
                                         np.asarray(ddq, float), g, np.zeros(6),
                                         model["Mlist"], model["Glist"], model["Slist"]))


def gravity_torques(model: dict, q) -> np.ndarray:
    """g(theta): RNEA with zero velocity and acceleration (MR Eq. 8.1).  q shape (n,) or (N, n)."""
    q = np.asarray(q, float)
    if q.ndim == 1:
        return inverse_dynamics(model, q, np.zeros_like(q), np.zeros_like(q))
    return np.array([gravity_torques(model, qk) for qk in q])


def load_task(name: str = "rotation1_task.json") -> dict:
    """Task data: waypoints, limit fractions, payload, step test, static poses, stop test."""
    with open(DATA / name) as fh:
        return json.load(fh)


def write_log(path, t, cmd, meas=None, cur=None) -> None:
    """Write a CSV in the project log format (project starter `hardware.py`):
    t, cmd_1..cmd_n[, meas_1..meas_n][, cur_1..cur_n]  (s, rad, A)."""
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
        w.writerows(np.round(np.hstack(cols), 7).tolist())


def load_log(path) -> dict:
    """Read a log written by `write_log` or by the station logger.  Returns t (N,), cmd, meas,
    cur (N, n) (None when absent) and any extra columns (e.g. `pose`, `dir`) as 1-D arrays."""
    with open(path, newline="") as fh:
        header = [h.strip() for h in next(csv.reader(fh))]
    data = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    col = {h: i for i, h in enumerate(header)}

    def group(prefix):
        names = sorted((h for h in header if h.startswith(prefix + "_")),
                       key=lambda h: int(h.split("_")[1]))
        return data[:, [col[h] for h in names]] if names else None

    out = dict(t=data[:, col["t"]] if "t" in col else None,
               cmd=group("cmd"), meas=group("meas"), cur=group("cur"))
    for h in header:
        if "_" not in h and h != "t":
            out[h] = data[:, col[h]]
    return out


def simulate_servo(t, cmd, wn=25.0, zeta=0.9, delay=0.02, stiffness=None, model=None,
                   enc_res: float = ENC_RES) -> np.ndarray:
    """Simulated servo response to a streamed command (the simulation fallback and the "sim" of the
    sim-vs-hardware comparison).

    * The command is a zero-order hold: each set-point is held until the next one arrives.
    * Pure delay `delay`, then  q_dd = wn^2 (q_cmd - q) - 2 zeta wn q_d  (semi-implicit Euler, 1 ms).
    * If `stiffness` K (n,) and `model` are given, the static sag  g(q)/K  is subtracted
      (K in N m/rad; `model` provides g(q) through the RNEA).
    * Encoder quantisation `enc_res` (rad); use enc_res=0 for an ideal sensor.
    wn, zeta, delay may be scalars or per-joint arrays.  Returns meas (N, n) on the time grid t."""
    t = np.asarray(t, float)
    cmd = np.asarray(cmd, float).reshape(len(t), -1)
    n = cmd.shape[1]
    wn = np.broadcast_to(np.asarray(wn, float), (n,))
    zeta = np.broadcast_to(np.asarray(zeta, float), (n,))
    delay = np.broadcast_to(np.asarray(delay, float), (n,))
    h = 1e-3
    tf = np.arange(t[0], t[-1] + h / 2, h)
    cf = np.empty((len(tf), n))
    for j in range(n):                       # zero-order hold of the delayed command
        k = np.searchsorted(t, tf - delay[j] + 1e-9, side="right") - 1
        cf[:, j] = cmd[np.clip(k, 0, len(t) - 1), j]
    q = cmd[0].copy()
    qd = np.zeros(n)
    out = np.empty_like(cf)
    for k in range(len(tf)):
        out[k] = q
        qdd = wn**2 * (cf[k] - q) - 2 * zeta * wn * qd
        qd = qd + h * qdd
        q = q + h * qd
    meas = np.column_stack([np.interp(t, tf, out[:, j]) for j in range(n)])
    if stiffness is not None:
        meas = meas - gravity_torques(model, meas) / np.asarray(stiffness, float)
    if enc_res:
        meas = np.round(meas / enc_res) * enc_res
    return meas


def check_command(t, q, model: dict, speed_fraction: float, margin_deg: float = 10.0,
                  tol: float = 0.01) -> dict:
    """Lab 6 safety check of a command table (project brief Sec. 5.2, rules A1 and A3): joint limits with a
    margin, peak finite-difference speed against speed_fraction * rated speed, rest at both ends."""
    t = np.asarray(t, float)
    q = np.asarray(q, float)
    lim = np.asarray(model["limits"], float)
    m = np.deg2rad(margin_deg)
    within = bool(np.all(q >= lim[:, 0] + m) and np.all(q <= lim[:, 1] - m))
    v = np.diff(q, axis=0) / np.diff(t)[:, None]
    ratio = float(np.max(np.abs(v) / (speed_fraction * np.asarray(model["qdot_max"], float))))
    steps = np.abs(np.diff(q, axis=0)).max(axis=1)
    start_rest, end_rest = bool(steps[0] < 1e-3), bool(steps[-1] < 1e-3)
    speed_ok = ratio <= 1.0 + tol
    return dict(ok=within and speed_ok and start_rest and end_rest, within_limits=within,
                peak_speed_ratio=ratio, speed_ok=speed_ok, starts_at_rest=start_rest,
                ends_at_rest=end_rest)


def tracking_metrics(t, cmd, meas, max_lag: float = 0.2) -> dict:
    """Lab 6 metrics: RMS and max |meas - cmd| per joint, the lag d (multiple of the sample time)
    that minimises the RMS of meas(t) - cmd(t - d), and the RMS after removing it."""
    t = np.asarray(t, float)
    cmd = np.asarray(cmd, float).reshape(len(t), -1)
    meas = np.asarray(meas, float).reshape(len(t), -1)
    e = meas - cmd
    dt = float(np.median(np.diff(t)))
    best = (np.inf, 0)
    for k in range(int(round(max_lag / dt)) + 1):
        ek = meas[k:] - cmd[:len(t) - k]
        r = float(np.sqrt(np.mean(ek**2)))
        if r < best[0] - 1e-15:
            best = (r, k)
    k = best[1]
    ek = meas[k:] - cmd[:len(t) - k]
    return dict(rms=np.sqrt(np.mean(e**2, axis=0)), max_abs=np.max(np.abs(e), axis=0), lag=k * dt,
                rms_after_lag=np.sqrt(np.mean(ek**2, axis=0)))


def average_approaches(log: dict) -> dict:
    """Static-pose log (columns pose, dir, cmd_j, meas_j, cur_j; each pose approached from below,
    dir = +1, and from above, dir = -1): average the two approaches of every pose, so that static
    friction cancels to first order.  Returns pose ids and the averaged cmd, meas, cur (P, n)."""
    poses = np.unique(log["pose"]).astype(int)
    out = {k: [] for k in ("cmd", "meas", "cur")}
    for p in poses:
        m = log["pose"] == p
        for k in out:
            out[k].append(log[k][m].mean(axis=0))
    return dict(pose=poses, **{k: np.array(v) for k, v in out.items()})


# =============================================================================================
# Part A -- pre-flight: units, the command file and the dry run
# =============================================================================================
def servo_to_model(q_servo_deg, offset_deg) -> np.ndarray:
    """Servo reading (degrees, servo zero) -> model angle (rad, PoE zero of the course robot models):
    q_model = q_servo - offset, converted to radians."""
    # TODO: implement (see the docstring and the lab handout)
    raise NotImplementedError


def model_to_servo(q_model, offset_deg) -> np.ndarray:
    """Inverse of `servo_to_model`: model angle (rad) -> servo command (degrees)."""
    # TODO: implement (see the docstring and the lab handout)
    raise NotImplementedError


def segment_time(delta, v_max, a_max, dt: float = DT) -> float:
    """Shortest duration T of a joint-synchronised rest-to-rest QUINTIC segment (MR §9.2.2.1, p. 332) that moves
    the joints by `delta` (n,) with |qdot_j| <= v_max_j and |qddot_j| <= a_max_j.

    For theta = theta_start + s(t/T) delta with the quintic s: peak speed (15/8)|delta_j|/T and peak
    acceleration (10/sqrt 3)|delta_j|/T^2.  T is the largest of the 2n resulting bounds, rounded UP to
    a whole number of servo periods dt (so that the segment ends on a sample)."""
    # TODO: implement (see the docstring and the lab handout)
    raise NotImplementedError


def waypoint_command(waypoints, v_max, a_max, dt: float = DT, dwell: float = 0.5):
    """Command table through `waypoints` (K, n): a rest-to-rest quintic segment between consecutive
    waypoints (duration from `segment_time`), a dwell of `dwell` s at the start, at every intermediate
    waypoint and at the end.  Sampled every dt from t = 0.
    Returns dict(t (N,), q, qd, qdd (N, n), seg_T (K-1,) segment durations, t_start (K-1,) start time
    of each segment).  The last sample is the end of the final dwell."""
    # TODO: implement (see the docstring and the lab handout)
    raise NotImplementedError


def dry_run(model: dict, cmd: dict, speed_fraction: float = 0.25, torque_fraction: float = 0.6) -> dict:
    """Dry-run prediction written on the session log before the arm is powered (brief Sec. 5.2, rule B2).
    `cmd` is the dict of `waypoint_command`.  Uses the ANALYTIC qd, qdd of the command.
      * tau (N, n): RNEA torques along the command (MR Eqs. 8.50-8.54);
      * peak_tau (n,): max |tau_j|;  torque_ratio (n,): peak_tau / (torque_fraction * tau_max);
      * speed_ratio (n,): max |qd_j| / (speed_fraction * qdot_max);
      * ok: every ratio <= 1."""
    # TODO: implement (see the docstring and the lab handout)
    raise NotImplementedError


# =============================================================================================
# Part B -- step test and servo identification
# =============================================================================================
def zeta_wn_from_step(overshoot: float, t_peak: float):
    """Hand method (Astrom-Murray Table 6.1): from the fractional overshoot Mp = exp(-pi zeta/sqrt(1-zeta^2))
    and the peak time t_peak = pi/(wn sqrt(1-zeta^2)) (measured from the START OF MOTION, i.e. after the
    delay) return (zeta, wn).  Requires 0 < overshoot < 1."""
    # TODO: implement (see the docstring and the lab handout)
    raise NotImplementedError


def step_features(t, cmd, meas, threshold: float = 0.1) -> dict:
    """Features of ONE step in one joint signal (1-D arrays).
      * t_step: first sample time where cmd differs from cmd[0];  A: cmd[-1] - cmd[0];
      * y0, y_ss: mean of meas before t_step, and mean over the last 0.3 s;
      * t_move: first time (linear interpolation between samples) at which (meas - y0)/(y_ss - y0)
        reaches `threshold` ... used only as a rough delay estimate;
      * overshoot: (max(meas) - y_ss)/(y_ss - y0) for a positive step (min for a negative one);
      * t_peak_abs: time of that extreme sample."""
    # TODO: implement (see the docstring and the lab handout)
    raise NotImplementedError


def step_response(tau, wn: float, zeta: float) -> np.ndarray:
    """Provided.  Unit step response of q_dd + 2 zeta wn q_d + wn^2 q = wn^2 u (Astrom-Murray Eq. 6.22,
    k = 1) at times tau >= 0 after the start of motion (0 for tau < 0)."""
    tau = np.asarray(tau, float)
    x = np.clip(tau, 0.0, None)
    if zeta < 1.0 - 1e-6:
        wd = wn * np.sqrt(1 - zeta**2)
        y = 1 - np.exp(-zeta * wn * x) * (np.cos(wd * x) + zeta / np.sqrt(1 - zeta**2) * np.sin(wd * x))
    elif zeta > 1.0 + 1e-6:
        a = wn * (zeta + np.sqrt(zeta**2 - 1)); b = wn * (zeta - np.sqrt(zeta**2 - 1))
        y = 1 - (a * np.exp(-b * x) - b * np.exp(-a * x)) / (a - b)
    else:
        y = 1 - np.exp(-wn * x) * (1 + wn * x)
    return np.where(tau > 0, y, 0.0)


def identify_servo(t, cmd, meas, window: float = 1.0) -> dict:
    """Least-squares fit of (wn, zeta, delay) of the servo model to ONE step (1-D arrays) on
    [t_step - 0.1, t_step + window]:  meas(t) ~ y0 + A_fit * step_response(t - t_step - delay).
    y0 and A_fit (which absorbs a change of sag) are solved linearly for every trial (wn, zeta, delay).
    Start from the hand method (`step_features` + `zeta_wn_from_step`) and refine with
    scipy.optimize.least_squares (bounds wn in [2, 200], zeta in [0.05, 3], delay in [0, 0.2]).
    Returns dict(wn, zeta, delay, y0, A_fit, rms) with rms the fit residual (rad)."""
    # TODO: implement (see the docstring and the lab handout)
    raise NotImplementedError


# =============================================================================================
# Part C -- static poses: gravity check, torque constant, loop stiffness
# =============================================================================================
def torque_constant(current, tau_model) -> float:
    """Least-squares k (N m/A, gear ratio included) with tau_model ~ k * current (one joint, or all
    entries pooled).  Same definition as project `hardware.torque_constant`."""
    # TODO: implement (see the docstring and the lab handout)
    raise NotImplementedError


def normalised_rms(predicted, measured) -> float:
    """RMS(predicted - measured) / RMS(measured) (project `hardware.normalised_rms`; HWk-2 gravity
    check passes at <= 0.3)."""
    # TODO: implement (see the docstring and the lab handout)
    raise NotImplementedError


def loop_stiffness(sag, tau_model) -> float:
    """Least-squares stiffness K (N m/rad) of one joint's position loop from static poses:
    sag = cmd - meas ~ tau_model / K, i.e. 1/K = sum(tau*sag)/sum(tau^2)."""
    # TODO: implement (see the docstring and the lab handout)
    raise NotImplementedError


def gravity_check(static: dict, model: dict, joints=(1, 2, 3), fit=(0, 2, 4), check=(1, 3, 5)) -> dict:
    """Gravity check of HW2-2(b)/HW3-2 type on the averaged static poses (`average_approaches`).
    For each joint index j in `joints` (0-based; default joints 2, 3 and 4, the gravity-loaded ones):
      * the model gravity torque at the MEASURED pose, g_j(q_meas);
      * k_j = torque_constant(cur_j[fit], g_j[fit]);   tau_cur_j = k_j * cur_j;
      * nrms_j = normalised_rms(g_j[check], tau_cur_j[check]) on the check poses only;
      * K_j = loop_stiffness(cmd_j - meas_j, g_j) on all poses.
    Returns dict(k, nrms, K) of arrays over `joints`, and g (P, n), tau_cur (P, len(joints))."""
    # TODO: implement (see the docstring and the lab handout)
    raise NotImplementedError


# =============================================================================================
# Part D -- simulation versus hardware, safe stop, pre-flight record
# =============================================================================================
def compare_runs(t, cmd, meas, sim, skip: float = 0.0) -> dict:
    """Sim-vs-hardware metrics after time `skip` (project brief Sec. 5.4), all per joint (rad):
    rms_cmd_meas, max_cmd_meas (tracking error of the device), rms_sim_meas, max_sim_meas (model error:
    simulation of the SAME command against the measurement), and lag (s) of meas behind cmd from
    `tracking_metrics`.  All arrays are on the same time grid t."""
    # TODO: implement (see the docstring and the lab handout)
    raise NotImplementedError


def stop_time(t, q, t_trigger: float, threshold: float = np.deg2rad(2.0), window: float = 0.1) -> float:
    """Safe-stop time (HWk-4): time from `t_trigger` until the joint speed, estimated by a central
    difference over `window` s (q(t + w/2) - q(t - w/2))/w to suppress encoder steps, stays below
    `threshold` (rad/s, all joints) for the rest of the log.  Returns inf if it never settles.
    The speed estimate is assigned to the centre sample; samples within w/2 of the ends are ignored."""
    # TODO: implement (see the docstring and the lab handout)
    raise NotImplementedError


PAYLOAD_MAX_KG = _ca.COURSE_ARM_SAFETY["payload_max_kg"]   # 0.125 kg (brief Sec. 5.2, rule A4)

# labels = the rule labels of project brief Sec. 5.2 (A1-A7 course arm, B1-B6 session rules)
PREFLIGHT_RULES = (
    "A5: every member who touches the arm has a signed induction",
    "A7: a TA or the instructor is named as supervisor",
    "A7: the e-stop was tested at the start of the session",
    "B4: a named member holds the e-stop",
    "A2/A6: the keep-out zone is taped and clear",
    "B5: the workspace is ready (mat, payload fixed, cables tidy)",
    "A1/A3: the command passed check_command at the required speed fraction with >= 10 deg margin",
    "B2: the dry run predicts speed and torque ratios <= 1 and is written on the log",
    "A (current): the servo current limit is <= 60 % of rated",
    "A4: the payload is <= 0.125 kg",
)


def preflight(record: dict) -> list:
    """Checks a session record (see data/session_record_template.json) against the hardware safety
    checklist of project brief Sec. 5.2 and returns the list of FAILED rules (strings of
    PREFLIGHT_RULES, in that order); an empty list means the session may start.

    Record keys: members_at_arm (list of names), inducted (list of names), supervisor (str),
    estop_tested (bool), estop_holder (str, must be one of members_at_arm), keepout_clear (bool),
    workspace_ready (bool), first_run (bool), check (dict from check_command, with keys ok and
    peak_speed_ratio), check_speed_fraction (float), margin_deg (float), dry_run_speed_ratio (float),
    dry_run_torque_ratio (float), current_limit_fraction (float), payload_kg (float, weighed payload
    at the tool point, at most PAYLOAD_MAX_KG).
    The required speed fraction is 0.25 for a first run of a motion and 0.5 otherwise; a check made
    at a HIGHER fraction than required does not count.  A missing key counts as a failure."""
    # TODO: implement (see the docstring and the lab handout)
    raise NotImplementedError
