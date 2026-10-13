"""Generate the synthetic IMU data set imu_knee.csv for Lab 2 (illustrative values).

Scenario: two IMUs (thigh, shank) strapped to a subject walking on a treadmill, as in a gait
laboratory, or to the thigh and shank links of a load-carriage exoskeleton during a field trial.
Each IMU's y-axis is aligned with the knee flexion axis, so  R_shank = R_thigh Rot(y, kappa).

Columns (time in s, angles in rad, angular velocity in rad/s, quaternions scalar-first):
  t,
  thigh_q0..thigh_q3, shank_q0..shank_q3          ground-truth orientations in the lab frame {s}
  thigh_wx..thigh_wz, shank_wx..shank_wz          ideal body-frame gyro (exact for zero-order hold)
  thigh_nwx..thigh_nwz, shank_nwx..shank_nwz      the same + constant bias + white noise
  kappa                                           ground-truth knee flexion angle

The ideal gyro sample k is  omega_b,k = so3ToVec(log(R_k^T R_{k+1})) / dt,  so the exponential
update R_{k+1} = R_k exp([omega_b,k] dt) reproduces the truth to machine precision.
Run:  python make_data.py   (deterministic, seed 361)
"""
from pathlib import Path

import numpy as np
import modern_robotics as mr

FS = 100.0            # Hz
T_END = 5.0           # s
STRIDE = 1.1          # s per gait cycle (illustrative)


def rot(axis, ang):
    return mr.MatrixExp3(mr.VecToso3(np.asarray(axis, float) * ang))


def quat(R):
    """Scalar-first unit quaternion with q0 >= 0 (via scipy, independent of the lab code)."""
    from scipy.spatial.transform import Rotation
    x, y, z, w = Rotation.from_matrix(R).as_quat()
    q = np.array([w, x, y, z])
    return q if q[0] >= 0 else -q


def thigh_R(t):
    ph = 2 * np.pi * t / STRIDE
    yaw = 0.30 + 0.08 * np.sin(ph)                     # heading + pelvic rotation
    pitch = 0.10 + 0.40 * np.sin(ph)                   # hip flexion/extension about y
    roll = 0.05 * np.sin(2 * ph)                       # small ab/adduction
    return rot([0, 0, 1], yaw) @ rot([0, 1, 0], pitch) @ rot([1, 0, 0], roll)


def kappa(t):
    ph = 2 * np.pi * t / STRIDE
    # two-hump knee flexion profile, 0.09 .. ~1.0 rad (illustrative)
    return 0.09 + 0.90 * (1 - np.cos(ph)) / 2 + 0.20 * np.exp(-((ph % (2 * np.pi)) - 1.0) ** 2 / 0.15)


def main(out=Path(__file__).with_name("imu_knee.csv")):
    dt = 1.0 / FS
    t = np.arange(0.0, T_END + dt / 2, dt)
    Rth = np.array([thigh_R(ti) for ti in t])
    Rsh = np.array([Rth[i] @ rot([0, 1, 0], kappa(ti)) for i, ti in enumerate(t)])
    rng = np.random.default_rng(361)
    rows = []
    bias = {"th": np.array([0.010, -0.006, 0.004]), "sh": np.array([-0.005, 0.008, 0.006])}
    for i, ti in enumerate(t):
        if i < len(t) - 1:
            wth = mr.so3ToVec(mr.MatrixLog3(Rth[i].T @ Rth[i + 1])) / dt
            wsh = mr.so3ToVec(mr.MatrixLog3(Rsh[i].T @ Rsh[i + 1])) / dt
        else:
            wth = np.zeros(3)
            wsh = np.zeros(3)
        nth = wth + bias["th"] + rng.normal(0, 0.005, 3)
        nsh = wsh + bias["sh"] + rng.normal(0, 0.005, 3)
        rows.append(np.concatenate([[ti], quat(Rth[i]), quat(Rsh[i]), wth, wsh, nth, nsh, [kappa(ti)]]))
    hdr = ("t,thigh_q0,thigh_q1,thigh_q2,thigh_q3,shank_q0,shank_q1,shank_q2,shank_q3,"
           "thigh_wx,thigh_wy,thigh_wz,shank_wx,shank_wy,shank_wz,"
           "thigh_nwx,thigh_nwy,thigh_nwz,shank_nwx,shank_nwy,shank_nwz,kappa")
    np.savetxt(out, np.array(rows), delimiter=",", header=hdr, comments="", fmt="%.12f")
    print(f"wrote {out} ({len(rows)} samples)")


if __name__ == "__main__":
    main()
