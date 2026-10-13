"""SBEG 46X Lab 11 -- simulated powered wheelchair: odometry, EKF landmark localisation, trajectory-tracking control
(formal report 3).
STARTER CODE -- you edit this file.  Its functions join the module `mobile` of your library sbeg46x_medrob.
Write every function marked TODO (keep the names and signatures); the helpers in the "Provided" section are given
(do not change them).  Run the checks from your Lab code folder with:   LAB_IMPL=starter python -m pytest -q test_lab.py
(rename the file to lab11_<ID>.py before you submit and run LAB_IMPL=lab11_<ID>).

Model (Modern Robotics Ch. 13; all values illustrative, data/wheelchair.json):
    chassis configuration q = (phi, x, y) in the ward frame {s}; chassis frame {b} at the midpoint of the drive-wheel
    axle, x_b forward; planar body twist V_b = (omega_bz, v_bx, v_by).
    Differential drive (MR Eq. 13.15): drive-wheel radius r, half-track d, wheel speeds u = (u_L, u_R);
    canonical controls (v, omega):  u_L = (v - omega d)/r,  u_R = (v + omega d)/r  (MR p. 528).
    Odometry (MR Sec. 13.4): V_b = F Delta_theta (MR Eqs. 13.32-13.34), Delta_q_b from MR Eq. 13.35, rotated to {s}
    with the previous heading phi_k (MR Eq. 13.36), q_{k+1} = q_k + Delta_q.
    Range-bearing sensor at the origin of {b}: z = (rho, beta) to a marker m = (m_x, m_y) with a known ID
    (Corke Sec. 6.2, Eq. 6.9, written here for the state ordering (phi, x, y) of MR).
    EKF (Corke Secs. 6.1.2 and 6.2, Eqs. 6.4-6.7 and 6.10-6.16; Week 10 notation Q, R, except that
    the Kalman gain is called K as in Corke -- the lecture notes and Week 10 call it L):
        predict  q- = f(q, Dtheta),  P- = F_q P F_q^T + F_u Q_u F_u^T
        update   nu = z - h(q-) (bearing wrapped),  S = H P- H^T + R,  K = P- H^T S^-1,
                 q = q- + K nu,  P = (I - K H) P- (I - K H)^T + K R K^T   (Joseph form = (I - K H) P- in exact arithmetic)
    Trajectory tracking (MR Sec. 13.3.4): error coordinates MR Eq. 13.30, nonlinear law MR Eq. 13.31; reference-point
    law MR Eq. 13.29 (with the feed-forward of the reference-point velocity) converted to (v, omega) by MR Eq. 13.21.
    Mobile manipulation (MR Sec. 13.5): J_base = [Ad_{T_0e^-1 T_b0^-1}] F6,  J_e = [J_base  J_arm].
Angles are not wrapped in the state (phi grows continuously); innovations and errors are wrapped to (-pi, pi].
"""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
DATA = HERE / "data"


# ==========================================================================================
# Provided (do not edit): parameters, data, helpers, reference route, simulator, metrics
# ==========================================================================================
def load_params(path=DATA / "wheelchair.json") -> dict:
    """Lab parameters (data/wheelchair.json, illustrative values): 'chair' (nominal model), 'truth' (the simulated
    chair, unknown to the filter), 'sensor', 'landmarks' ([id, x, y] rows), 'ekf', 'route', 'control', 'arm_mount'."""
    with open(path) as f:
        return json.load(f)


def landmark_table(p: dict, ids=None) -> dict:
    """{id: np.array([x, y])} of the ward markers (optionally only the listed IDs)."""
    return {int(i): np.array([x, y], float) for i, x, y in p["landmarks"] if ids is None or int(i) in ids}


def load_log(path=DATA / "ward_log.csv") -> dict:
    """Recorded run (simulation): arrays t (s), enc_L, enc_R (cumulative wheel angles from the encoders, rad,
    quantised to 4096 counts/rev) and the ground truth phi, x, y (from the simulator -- 'motion capture')."""
    with open(path) as f:
        rows = list(csv.DictReader(f))
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}


def load_meas(path=DATA / "ward_meas.csv") -> list:
    """Landmark readings of the recorded run: list of dicts {k, t, id, range, bearing}; reading k belongs to sample k
    of the log (it is taken after the motion from sample k-1 to k)."""
    with open(path) as f:
        rows = list(csv.DictReader(f))
    return [dict(k=int(r["k"]), t=float(r["t"]), id=int(r["id"]), range=float(r["range"]),
                 bearing=float(r["bearing"])) for r in rows]


def wrap(a):
    """Wrap an angle (or array of angles) to (-pi, pi]."""
    return -((-np.asarray(a, float) + np.pi) % (2 * np.pi) - np.pi)


def rot2(phi: float) -> np.ndarray:
    c, s = math.cos(phi), math.sin(phi)
    return np.array([[c, -s], [s, c]])


def _arc(q, w, vx, vy):
    """Exact displacement of a constant planar body twist (w, vx, vy) applied for unit time from q = (phi, x, y).
    Used by the simulator's 'true' chair (it does not call your code)."""
    if abs(w) < 1e-12:
        dqb = np.array([0.0, vx, vy])
    else:
        dqb = np.array([w, (vx * math.sin(w) + vy * (math.cos(w) - 1)) / w,
                        (vy * math.sin(w) + vx * (1 - math.cos(w))) / w])
    dq = np.r_[dqb[0], rot2(q[0]) @ dqb[1:]]
    return np.asarray(q, float) + dq


def reference_trajectory(p: dict) -> dict:
    """Reference q_d(t) = (phi_d, x_d, y_d) and nominal controls (v_d, omega_d) of the ward loop of p['route'],
    sampled every h: constant speed v along straight lines and left arcs.  Returns dict t, qd (N x 3), vd, wd."""
    rt, h = p["route"], p["chair"]["h"]
    v = rt["v"]
    segs = []
    q0 = np.array(rt["start"], float)
    for _ in range(rt["laps"]):
        for s in rt["segments"]:
            if s[0] == "line":
                L, k = s[1], 0.0
            else:
                L, k = s[1] * math.radians(s[2]), 1.0 / s[1]
            segs.append((q0.copy(), L, k))
            if k == 0.0:
                q0 = q0 + np.array([0.0, L * math.cos(q0[0]), L * math.sin(q0[0])])
            else:
                ph = q0[0] + k * L
                q0 = np.array([ph, q0[1] + (math.sin(ph) - math.sin(q0[0])) / k,
                               q0[2] - (math.cos(ph) - math.cos(q0[0])) / k])
    total = sum(L for _, L, _ in segs)
    N = int(math.floor(total / (v * h) + 1e-9)) + 1
    t = np.arange(N) * h
    qd, vd, wd = np.zeros((N, 3)), np.full(N, v), np.zeros(N)
    for i, ti in enumerate(t):
        s = v * ti
        for qs, L, k in segs:
            if s <= L + 1e-12:
                break
            s -= L
        if k == 0.0:
            qd[i] = qs + np.array([0.0, s * math.cos(qs[0]), s * math.sin(qs[0])])
        else:
            ph = qs[0] + k * s
            qd[i] = [ph, qs[1] + (math.sin(ph) - math.sin(qs[0])) / k, qs[2] - (math.cos(ph) - math.cos(qs[0])) / k]
        wd[i] = v * k
    return dict(t=t, qd=qd, vd=vd, wd=wd)


def pose_errors(q_est, q_true) -> dict:
    """Errors of an estimate: arrays e_pos (m, Euclidean in x-y) and e_phi (rad, wrapped)."""
    q_est, q_true = np.atleast_2d(q_est), np.atleast_2d(q_true)
    return dict(e_pos=np.hypot(q_est[:, 1] - q_true[:, 1], q_est[:, 2] - q_true[:, 2]),
                e_phi=wrap(q_est[:, 0] - q_true[:, 0]))


def distance_travelled(q_true) -> np.ndarray:
    """Cumulative path length of the chair (m) along a trajectory (N x 3)."""
    q_true = np.atleast_2d(q_true)
    return np.r_[0.0, np.cumsum(np.hypot(np.diff(q_true[:, 1]), np.diff(q_true[:, 2])))]


def tracking_metrics(sim: dict, t_from: float = 5.0) -> dict:
    """Tracking metrics of a closed-loop run (true pose vs reference): rms and max position error and rms heading
    error for t >= t_from, final position error, peak |u_L|, |u_R| and the fraction of samples whose wheel commands
    were scaled down by saturate_wheels."""
    e = pose_errors(sim["q_true"][:-1], sim["qd"])
    m = sim["t"] >= t_from - 1e-12
    return dict(rms_pos=float(np.sqrt(np.mean(e["e_pos"][m] ** 2))), max_pos=float(np.max(e["e_pos"][m])),
                rms_phi=float(np.sqrt(np.mean(e["e_phi"][m] ** 2))), final_pos=float(e["e_pos"][-1]),
                peak_u=float(np.max(np.abs(np.c_[sim["uL"], sim["uR"]]))), frac_sat=float(np.mean(sim["sat"])))


def simulate(p: dict, estimator: str = "ekf", controller: str = "nonlinear", seed: int = 1211, offset=None,
             landmark_ids=None, use_gate: bool = True, k_s=None, gains=None) -> dict:
    """Closed-loop run of the wheelchair on the ward loop.

    estimator  'truth' (the controller sees the true pose), 'odometry' (dead reckoning, odometry_step) or 'ekf'
               (ekf_predict + ekf_update with the landmark readings)
    controller 'nonlinear' (tracking_control, MR Eq. 13.31) or 'point' (point_tracking_control, MR Eqs. 13.29/13.21)
    offset     initial pose error (dphi, dx, dy) of the chair w.r.t. the start of the reference (default
               p['control']['q0_offset']); the estimator starts from the TRUE initial pose (docking station).
    landmark_ids  markers that exist (default all); use_gate  Mahalanobis gate on/off; k_s  filter odometry-noise
               coefficient (default p['ekf']['k_s']); gains  dict overriding k1, k2, k3, kp, xr.
    The noise sequences depend only on `seed`, so runs with different estimators see the same slip and sensor noise.
    Each sample k: control from the estimate -> saturate_wheels -> wheel_speeds -> wheels turn u h -> the true chair
    moves (true radii, half-track, slip) -> encoders (quantised) -> estimator predict -> every p['sensor']['every']
    samples: readings of all markers in range (+ outliers) -> ekf_update for each reading in ID order.
    Returns dict t, qd, vd, wd (N), q_true, q_est (N + 1 x 3), P (N + 1 x 3 x 3, EKF only), v, w, uL, uR, sat (N),
    enc (N + 1 x 2, quantised cumulative wheel angles), meas (list of readings), n_acc, n_rej."""
    ch, tr, se, ek, ct = p["chair"], p["truth"], p["sensor"], p["ekf"], dict(p["control"])
    if gains:
        ct.update(gains)
    r, d, h = ch["r"], ch["d"], ch["h"]
    res = 2 * math.pi / ch["enc_counts_per_rev"]
    ref = reference_trajectory(p)
    N = len(ref["t"])
    lm = landmark_table(p)
    ids = sorted(lm)
    present = set(ids if landmark_ids is None else landmark_ids)
    rng = np.random.default_rng(seed)
    slip = rng.standard_normal((N, 2))
    mn = rng.standard_normal((N + 1, len(ids), 2))
    ou = rng.random((N + 1, len(ids)))
    oa = rng.uniform(se["outlier_short"][0], se["outlier_short"][1], (N + 1, len(ids)))
    sb = math.radians(se["sigma_bearing_deg"])
    Rm = np.diag([ek["sigma_range"] ** 2, math.radians(ek["sigma_bearing_deg"]) ** 2])
    ks = ek["k_s"] if k_s is None else k_s
    F = diffdrive_F(r, d) if estimator != "truth" else None
    off = np.array(ct["q0_offset"] if offset is None else offset, float)
    q = ref["qd"][0] + off
    qh = q.copy()
    P = np.diag(np.array(ek["P0_sigma"]) ** 2)
    th = np.zeros(2)
    encq = np.zeros(2)
    out = dict(t=ref["t"], qd=ref["qd"], vd=ref["vd"], wd=ref["wd"], q_true=[q.copy()], q_est=[qh.copy()],
               P=[P.copy()], v=np.zeros(N), w=np.zeros(N), uL=np.zeros(N), uR=np.zeros(N), sat=np.zeros(N, bool),
               enc=[encq.copy()], meas=[], n_acc=0, n_rej=0)
    for k in range(N):
        qc = q if estimator == "truth" else qh
        if controller == "nonlinear":
            v, w = tracking_control(qc, ref["qd"][k], ref["vd"][k], ref["wd"][k], ct["k1"], ct["k2"], ct["k3"])
        elif controller == "point":
            v, w = point_tracking_control(qc, ref["qd"][k], ref["vd"][k], ref["wd"][k], ct["xr"], ct["kp"])
        else:
            raise ValueError(controller)
        vs, ws = saturate_wheels(v, w, r, d, ch["u_max"], ch["v_max"])
        out["sat"][k] = abs(vs - v) > 1e-12 or abs(ws - w) > 1e-12
        uL, uR = wheel_speeds(vs, ws, r, d)
        out["v"][k], out["w"][k], out["uL"][k], out["uR"][k] = vs, ws, uL, uR
        dth = np.array([uL, uR]) * h
        # true chair: true radii and half-track, random slip on each wheel's ground distance
        sL = tr["r_L"] * dth[0] + math.sqrt(tr["k_slip"] * abs(r * dth[0])) * slip[k, 0]
        sR = tr["r_R"] * dth[1] + math.sqrt(tr["k_slip"] * abs(r * dth[1])) * slip[k, 1]
        q = _arc(q, (sR - sL) / (2 * tr["d"]), (sL + sR) / 2, 0.0)
        th = th + dth
        enc_new = np.floor(th / res + 1e-9) * res
        ddth = enc_new - encq
        encq = enc_new
        if estimator == "odometry":
            qh = odometry_step(qh, ddth, F)
        elif estimator == "ekf":
            qh, P = ekf_predict(qh, P, ddth, r, d, ks)
        if (k + 1) % se["every"] == 0:
            for j, i in enumerate(ids):
                if i not in present:
                    continue
                dx, dy = lm[i] - q[1:]
                rho = math.hypot(dx, dy)
                if rho > se["max_range"]:
                    continue
                z = np.array([rho + se["sigma_range"] * mn[k + 1, j, 0],
                              float(wrap(math.atan2(dy, dx) - q[0] + sb * mn[k + 1, j, 1]))])
                if ou[k + 1, j] < se["p_outlier"]:
                    z[0] = max(0.05, z[0] - oa[k + 1, j])
                out["meas"].append(dict(k=k + 1, t=(k + 1) * h, id=i, range=float(z[0]), bearing=float(z[1])))
                if estimator == "ekf":
                    qh, P, info = ekf_update(qh, P, z, lm[i], Rm, ek["gate"] if use_gate else None)
                    out["n_acc" if info["accepted"] else "n_rej"] += 1
        if estimator == "truth":
            qh = q.copy()
        out["q_true"].append(q.copy())
        out["q_est"].append(qh.copy())
        out["P"].append(P.copy())
        out["enc"].append(encq.copy())
    for key in ("q_true", "q_est", "P", "enc"):
        out[key] = np.array(out[key])
    return out


# ==========================================================================================
# Part A -- wheeled-robot kinematics and odometry (MR Secs. 13.2-13.4)
# ==========================================================================================
def wheel_row(r: float, gamma: float, beta: float, xw: float, yw: float) -> np.ndarray:
    """A1. Row h_i(0) of MR Eq. 13.6 (phi = 0) for a wheel of radius r at (xw, yw) in {b}, driving direction beta and
    free-sliding angle gamma:  h_i(0) = (1/(r cos gamma)) [xw sin(beta+gamma) - yw cos(beta+gamma),
    cos(beta+gamma), sin(beta+gamma)].  Returns a length-3 array (wheel driving speed per unit body twist)."""
    # TODO (A1)
    raise NotImplementedError("wheel_row (A1): write this function")


def H0_matrix(wheels) -> np.ndarray:
    """A2. H(0) of MR Eq. 13.8 (m x 3): stack wheel_row for each wheel; wheels = list of (r, gamma, beta, xw, yw)."""
    # TODO (A2)
    raise NotImplementedError("H0_matrix (A2): write this function")


def diffdrive_F(r: float, d: float) -> np.ndarray:
    """A3. F of the differential drive (MR Eq. 13.34), 3 x 2: V_b = (omega_bz, v_bx, v_by) = F (u_L, u_R)
    (or F (Delta theta_L, Delta theta_R) for one odometry interval)."""
    # TODO (A3)
    raise NotImplementedError("diffdrive_F (A3): write this function")


def wheel_speeds(v: float, omega: float, r: float, d: float):
    """A4. Wheel speeds of the diff-drive for the canonical controls (MR p. 528):
    u_L = (v - omega d)/r,  u_R = (v + omega d)/r.  Returns (u_L, u_R) in rad/s."""
    # TODO (A4)
    raise NotImplementedError("wheel_speeds (A4): write this function")


def odometry_step(q, dtheta, F) -> np.ndarray:
    """A5. One odometry update (MR Sec. 13.4): V_b = F dtheta (dtheta = wheel-angle increments, m of them),
    Delta_q_b from MR Eq. 13.35 (the two cases omega_bz = 0 and omega_bz != 0, including v_by),
    Delta_q = [[1, 0, 0], [0, cos phi_k, -sin phi_k], [0, sin phi_k, cos phi_k]] Delta_q_b (MR Eq. 13.36),
    returns q_{k+1} = q_k + Delta_q (phi not wrapped)."""
    # TODO (A5)
    raise NotImplementedError("odometry_step (A5): write this function")


def dead_reckoning(q0, enc, F) -> np.ndarray:
    """A6. Odometry over a whole run: enc = N x m cumulative wheel angles (rad); returns N x 3 poses with
    q[0] = q0 and q[k+1] = odometry_step(q[k], enc[k+1] - enc[k], F)."""
    # TODO (A6)
    raise NotImplementedError("dead_reckoning (A6): write this function")


# ==========================================================================================
# Part B -- EKF localisation with known landmarks (Corke Secs. 6.1-6.2; Week 10)
# ==========================================================================================
def odometry_jacobians(q, dtheta, r: float, d: float):
    """B1. Diff-drive odometry model q+ = f(q, dtheta) (= odometry_step with diffdrive_F) and its Jacobians
    F_q = df/dq (3 x 3) and F_u = df/d(dtheta) (3 x 2), evaluated at (q, dtheta).
    Hint: with omega = r (dth_R - dth_L)/(2d), v = r (dth_L + dth_R)/2, Delta_q_b = (omega, v sin(omega)/omega,
    v (1 - cos omega)/omega); use the series sin w/w ~ 1 - w^2/6, (1 - cos w)/w ~ w/2 - w^3/24 for |w| < 1e-4.
    Returns (q_next, F_q, F_u)."""
    # TODO (B1)
    raise NotImplementedError("odometry_jacobians (B1): write this function")


def wheel_noise_cov(dtheta, r: float, k_s: float) -> np.ndarray:
    """B2. Odometry noise Q_u (2 x 2) of the wheel-angle increments: each wheel's ground distance r dtheta_i has
    variance k_s |r dtheta_i| (independent wheels), so var(dtheta_i) = k_s |dtheta_i| / r."""
    # TODO (B2)
    raise NotImplementedError("wheel_noise_cov (B2): write this function")


def ekf_predict(q, P, dtheta, r: float, d: float, k_s: float):
    """B3. EKF prediction with the odometry increments: q- = f(q, dtheta), P- = F_q P F_q^T + F_u Q_u F_u^T
    (Corke Eqs. 6.6-6.7).  Returns (q-, P-)."""
    # TODO (B3)
    raise NotImplementedError("ekf_predict (B3): write this function")


def range_bearing(q, m) -> np.ndarray:
    """B4. Predicted reading of marker m = (m_x, m_y) from pose q = (phi, x, y), sensor at the origin of {b}:
    rho = sqrt(dx^2 + dy^2), beta = wrap(atan2(dy, dx) - phi), (dx, dy) = m - (x, y)  (Corke Eq. 6.9)."""
    # TODO (B4)
    raise NotImplementedError("range_bearing (B4): write this function")


def range_bearing_jacobian(q, m) -> np.ndarray:
    """B5. H = dh/dq (2 x 3) of range_bearing for the state ordering (phi, x, y):
    [[0, -dx/rho, -dy/rho], [-1, dy/rho^2, -dx/rho^2]]."""
    # TODO (B5)
    raise NotImplementedError("range_bearing_jacobian (B5): write this function")


def ekf_update(q, P, z, m, R, gate=None):
    """B6. EKF update with one range-bearing reading z of marker m (Corke Eqs. 6.13-6.16):
    nu = z - h(q) with the bearing component wrapped; S = H P H^T + R; d2 = nu^T S^-1 nu (squared Mahalanobis
    distance, 'NIS').  If gate is not None and d2 > gate the reading is rejected (q, P returned unchanged).
    Otherwise K = P H^T S^-1, q+ = q + K nu, P+ = (I - K H) P (I - K H)^T + K R K^T (Joseph form).
    Returns (q+, P+, info) with info = dict(nu=nu, S=S, K=K, d2=d2, accepted=bool)."""
    # TODO (B6)
    raise NotImplementedError("ekf_update (B6): write this function")


def ekf_localise(log: dict, meas: list, p: dict, landmark_ids=None, use_gate: bool = True, k_s=None) -> dict:
    """B7. Run the EKF over a recorded log: start from the true initial pose with P0 = diag(P0_sigma^2)
    (p['ekf']); for k = 0..N-2 predict with the encoder increments enc[k+1] - enc[k] (nominal r, d of p['chair']),
    then update with every reading whose k equals k+1 (in file order), skipping markers not in landmark_ids
    (default: all).  R = diag(sigma_range^2, sigma_bearing^2) of p['ekf']; gate = p['ekf']['gate'] if use_gate.
    Returns dict q (N x 3), P (N x 3 x 3), n_acc, n_rej, d2 (list of the squared Mahalanobis distances)."""
    # TODO (B7)
    raise NotImplementedError("ekf_localise (B7): write this function")


# ==========================================================================================
# Part C -- feedback control of the nonholonomic base (MR Sec. 13.3.4)
# ==========================================================================================
def error_coordinates(q, qd) -> np.ndarray:
    """C1. Error coordinates of MR Eq. 13.30: phi_e = wrap(phi - phi_d) and (x_e, y_e) = the error
    (x - x_d, y - y_d) expressed in the reference frame {d}.  Returns (phi_e, x_e, y_e)."""
    # TODO (C1)
    raise NotImplementedError("error_coordinates (C1): write this function")


def tracking_control(q, qd, vd: float, wd: float, k1: float, k2: float, k3: float):
    """C2. Nonlinear feed-forward + feedback tracking law of MR Eq. 13.31:
    v = (v_d - k1 |v_d| (x_e + y_e tan phi_e)) / cos phi_e,
    omega = omega_d - (k2 v_d y_e + k3 |v_d| tan phi_e) cos^2 phi_e.   Returns (v, omega)."""
    # TODO (C2)
    raise NotImplementedError("tracking_control (C2): write this function")


def point_controls(phi: float, xr: float, yr: float, pdot) -> tuple:
    """C3. Controls (v, omega) that give the reference point P = (xr, yr) in {b} the world velocity
    pdot = (xdot_P, ydot_P) (MR Eq. 13.21).  Singular for xr = 0."""
    # TODO (C3)
    raise NotImplementedError("point_controls (C3): write this function")


def point_tracking_control(q, qd, vd: float, wd: float, xr: float, kp: float):
    """C4. Reference-point tracking (MR Eq. 13.29 + feed-forward): P is the point (xr, 0) of {b}; its desired position
    is the same point of the reference chassis, p_d = (x_d + xr cos phi_d, y_d + xr sin phi_d), with velocity
    pdot_d = (v_d cos phi_d - xr omega_d sin phi_d, v_d sin phi_d + xr omega_d cos phi_d).
    Command pdot = pdot_d + kp (p_d - p) and convert with point_controls.  Returns (v, omega)."""
    # TODO (C4)
    raise NotImplementedError("point_tracking_control (C4): write this function")


def saturate_wheels(v: float, omega: float, r: float, d: float, u_max: float, v_max: float = None):
    """C5. Scale (v, omega) by ONE common factor s >= 1 so that |u_L|, |u_R| <= u_max and |v| <= v_max (if given):
    s = max(1, max(|u_L|, |u_R|)/u_max, |v|/v_max).  Keeps the path curvature omega/v.  Returns (v/s, omega/s)."""
    # TODO (C5)
    raise NotImplementedError("saturate_wheels (C5): write this function")


# ==========================================================================================
# Part D (optional) -- mobile manipulation: wheelchair + feeding arm (MR Sec. 13.5)
# ==========================================================================================
def base_jacobian(F, Tb0, T0e) -> np.ndarray:
    """D1. J_base = [Ad_{T_0e^-1 T_b0^-1}] F6 (MR Sec. 13.5), with F6 = F padded to 6 rows: two rows of zeros above
    F and one below, so that V_b6 = (0, 0, omega_bz, v_bx, v_by, 0) = F6 u.  Returns 6 x m."""
    # TODO (D1)
    raise NotImplementedError("base_jacobian (D1): write this function")


def mobile_manip_jacobian(F, Tb0, Blist, M, theta) -> np.ndarray:
    """D2. J_e = [J_base(theta)  J_arm(theta)] (6 x (m + n)) of MR Sec. 13.5: T_0e = M e^[B1 th1] ... e^[Bn thn]
    (body PoE), J_arm = body Jacobian of the arm (MR Eq. 5.18; you may call modern_robotics FKinBody/JacobianBody)."""
    # TODO (D2)
    raise NotImplementedError("mobile_manip_jacobian (D2): write this function")
