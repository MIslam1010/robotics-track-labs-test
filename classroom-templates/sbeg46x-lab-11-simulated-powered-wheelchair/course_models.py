"""Robot model used in Lab 11 -- copied from the shared course robot models (Course resources) (numbers identical).

  srs_7r_feeding()   Wheelchair-mounted SRS 7R assistive (feeding) arm, SBEG 361 project scenario S2:
                     shoulder z-y-x at height 0.25 m above the arm base {0}, elbow y, wrist x-y-z;
                     upper arm 0.40 m, forearm 0.35 m, wrist-to-spoon 0.12 m.
                     Lab 11 (optional Part D) mounts it on the powered wheelchair of data/wheelchair.json.

The lab must be self-contained, so the two helpers the model needs are copied as well.
ILLUSTRATIVE VALUES: teaching values, not the data of a real product.
Notation: Modern Robotics (MR) -- screw axes S = (w, v) (MR Sec. 3.3.2), home pose M, body screws B = [Ad_{M^-1}] S.
"""
from __future__ import annotations

import modern_robotics as mr
import numpy as np

DEG = np.pi / 180.0


def screw_revolute(w, q, h: float = 0.0) -> np.ndarray:
    """Screw axis S = (w, -w x q + h w) of a joint about unit axis w through point q (MR Sec. 3.3.2)."""
    w = np.asarray(w, float)
    q = np.asarray(q, float)
    return np.r_[w, -np.cross(w, q) + h * w]


def trans(p=(0.0, 0.0, 0.0), R=None) -> np.ndarray:
    return mr.RpToTrans(np.eye(3) if R is None else np.asarray(R, float), np.asarray(p, float))


def srs_7r_feeding() -> dict:
    """Wheelchair-mounted SRS 7R arm (shoulder z-y-x, elbow y, wrist x-y-z); shoulder height 0.25,
    upper arm 0.40, forearm 0.35, wrist-to-spoon 0.12 m.  Returns Slist, Blist, M, limits, qdot_max."""
    h, Lu, Lf, Lt = 0.25, 0.40, 0.35, 0.12
    sh, el, wr = np.array([0, 0, h]), np.array([Lu, 0, h]), np.array([Lu + Lf, 0, h])
    w = [(0, 0, 1), (0, 1, 0), (1, 0, 0), (0, 1, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)]
    q = [sh, sh, sh, el, wr, wr, wr]
    S = np.column_stack([screw_revolute(wi, qi) for wi, qi in zip(w, q)])
    M = trans(wr + np.array([Lt, 0, 0]))
    lim = np.array([[-170, 170], [-120, 120], [-170, 170], [-150, 5], [-170, 170], [-115, 115],
                    [-170, 170]], float) * DEG
    return dict(key="srs_7r_feeding", name="Wheelchair-mounted SRS 7R arm", Slist=S,
                Blist=mr.Adjoint(mr.TransInv(M)) @ S, M=M, limits=lim, qdot_max=40 * DEG * np.ones(7),
                params=dict(h=h, Lu=Lu, Lf=Lf, Lt=Lt), illustrative=True)
