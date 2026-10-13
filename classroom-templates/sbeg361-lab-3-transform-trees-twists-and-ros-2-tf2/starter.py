"""SBEG 361 Lab 3 -- Homogeneous transforms, transform trees and twists (STARTER).

Fill in every function marked  # TODO.  Do not change names or signatures: test_lab.py
imports them.  Run the auto-checks with

    LAB_IMPL=starter python -m pytest -q test_lab.py

You may NOT call modern_robotics, scipy.spatial.transform or np.linalg.inv inside these
functions -- they are what the tests compare against.  The helpers that are already complete
(vec_to_so3, so3_to_vec, _ancestors, _common, path, the loaders and the analysis helpers) may
be used freely.

Notation follows Lynch & Park, *Modern Robotics* (MR), Sections 3.3.1-3.3.2:
  T = (R, p) in SE(3)          4x4 homogeneous transformation           (MR Eq. 3.62)
  T^-1 = (R^T, -R^T p)         inverse                                   (MR Eq. 3.64)
  T x = R x + p                action on a point (homogeneous coords)    (MR Eq. 3.65)
  V = (omega, v)               twist, angular part FIRST                 (MR Eqs. 3.70, 3.74)
  [V] = [[omega] v; 0 0]       4x4 se(3) matrix                          (MR Eq. 3.71)
  [V_b] = T^-1 Tdot, [V_s] = Tdot T^-1                                  (MR Eqs. 3.81-3.82)
  [Ad_T] = [[R 0]; [[p]R R]]   adjoint                                   (MR Def. 3.20)
  screw {q, s_hat, h}: V = (s_hat, -s_hat x q + h s_hat) thetadot       (MR Sec. 3.3.2.2)

Transform tree convention (identical to ROS 2 tf2): an edge parent -> child stores
T_parent_child, the pose of the child frame expressed in the parent frame.
lookup(target, source) returns T_target_source (tf2: buffer.lookup_transform(target, source, t)).
"""
import json
import math

import numpy as np

TOL = 1e-6  # same threshold as modern_robotics.NearZero


# ---------------------------------------------------------------- helpers reused from Lab 2
def vec_to_so3(omega):
    """3x3 skew-symmetric matrix [omega] (MR Eq. 3.30)."""
    w1, w2, w3 = np.asarray(omega, float)
    return np.array([[0.0, -w3, w2], [w3, 0.0, -w1], [-w2, w1, 0.0]])


def so3_to_vec(so3mat):
    """omega from [omega]."""
    m = np.asarray(so3mat, float)
    return np.array([m[2, 1], m[0, 2], m[1, 0]])


def quat_xyzw_to_rot(q_xyzw):
    """Rotation matrix from a tf2/ROS quaternion (x, y, z, w) -- SCALAR LAST.

    MR App. B.3 writes q = (q0, q1, q2, q3) with the scalar first; tf2 and SciPy store
    (x, y, z, w) = (q1, q2, q3, q0).  The quaternion is normalised first (MR Eq. B.12)."""
    # TODO: reorder to (q0,q1,q2,q3) = (w,x,y,z), normalise, then use MR Eq. B.12 (your Lab 2 quat_to_rot)
    raise NotImplementedError("quat_xyzw_to_rot")


# ---------------------------------------------------------------- SE(3) basics
def rp_to_trans(R, p):
    """Build T = [[R, p], [0, 1]] (MR Eq. 3.62)."""
    # TODO: start from np.eye(4) and fill the R block and the p column
    raise NotImplementedError("rp_to_trans")


def trans_to_rp(T):
    """Split T into (R, p)."""
    # TODO: return copies of the top-left 3x3 block and the top-right 3x1 column
    raise NotImplementedError("trans_to_rp")


def trans_inv(T):
    """Inverse of T in SE(3) without a general matrix inverse (MR Eq. 3.64)."""
    # TODO: use MR Eq. 3.64 -- do NOT call np.linalg.inv
    raise NotImplementedError("trans_inv")


def is_se3(T, tol=1e-6):
    """True if T is a 4x4 element of SE(3): R^T R = I, det R = +1, last row (0, 0, 0, 1)."""
    # TODO: check shape, R^T R = I, det R = +1 and the last row (np.allclose with atol=tol)
    raise NotImplementedError("is_se3")


def transform_point(T, x):
    """Return T x = R x + p for one point (shape (3,)) or many points (shape (N, 3)) (MR Eq. 3.65)."""
    # TODO: R x + p; for an (N,3) array use x @ R.T + p
    raise NotImplementedError("transform_point")


# ---------------------------------------------------------------- se(3) and the adjoint
def vec_to_se3(V):
    """4x4 matrix [V] of the twist V = (omega, v) (MR Eq. 3.71)."""
    # TODO: top-left block [omega] (vec_to_so3), top-right column v, bottom row zeros
    raise NotImplementedError("vec_to_se3")


def se3_to_vec(se3mat):
    """Twist V = (omega, v) from its 4x4 matrix [V]."""
    # TODO: inverse of vec_to_se3
    raise NotImplementedError("se3_to_vec")


def adjoint(T):
    """6x6 adjoint [Ad_T] = [[R, 0], [[p]R, R]] (MR Def. 3.20)."""
    # TODO: MR Def. 3.20: blocks R, 0, [p]R, R
    raise NotImplementedError("adjoint")


# ---------------------------------------------------------------- twists
def twists_from_T_Tdot(T, Tdot):
    """Return (V_s, V_b) from a configuration T_sb and its time derivative (MR Eqs. 3.81-3.82)."""
    # TODO: [V_s] = Tdot T^-1, [V_b] = T^-1 Tdot (MR Eqs. 3.81-3.82); use trans_inv and se3_to_vec
    raise NotImplementedError("twists_from_T_Tdot")


def twists_from_samples(Ts, dt):
    """Estimate (V_s, V_b) at every interior sample of a pose sequence by central differences.

    Ts: array (N, 4, 4) of T_sb(t_k) sampled every dt seconds.
    Returns two arrays of shape (N-2, 6): twist k corresponds to sample k+1.
    Tdot_k ~ (T_{k+1} - T_{k-1}) / (2 dt).  The numerical T^-1 Tdot is only approximately in
    se(3), so the angular part is read from its skew-symmetric part (M - M^T)/2."""
    # TODO: central difference Tdot_k = (T_{k+1}-T_{k-1})/(2 dt); read omega from the skew part (M - M^T)/2
    raise NotImplementedError("twists_from_samples")


def screw_to_twist(q, s_hat, h, thetadot=1.0):
    """Twist V = (s_hat thetadot, (-s_hat x q + h s_hat) thetadot) of a screw {q, s_hat, h}
    (MR Sec. 3.3.2.2).  Use h = np.inf for a pure translation along s_hat (then q is ignored)."""
    # TODO: normalise s_hat; h = inf -> (0, s_hat*thetadot); else (s_hat, -s_hat x q + h s_hat)*thetadot
    raise NotImplementedError("screw_to_twist")


def twist_to_screw(V):
    """Screw interpretation of a twist (MR Sec. 3.3.2.2, Def. 3.24).

    Returns a dict with keys
      'S'        normalised screw axis (MR Def. 3.24),
      'thetadot' speed about/along the axis (>= 0),
      'h'        pitch (np.inf for a pure translation),
      's_hat'    unit axis direction,
      'q'        the point on the axis closest to the frame origin (q = omega x v / |omega|^2),
                 or None for a pure translation.
    Raises ValueError for the zero twist."""
    # TODO: case |omega| > TOL: thetadot = |omega|, h = omega.v/|omega|^2, q = omega x v/|omega|^2; case omega = 0: h = inf, q = None; zero twist -> ValueError
    raise NotImplementedError("twist_to_screw")


# ---------------------------------------------------------------- transform tree (tf2-like)
class TransformTree:
    """A tree of named frames; each edge parent -> child stores T_parent_child.

    Like tf2, every frame has at most one parent, so there is exactly one path between any two
    connected frames.  lookup(target, source) composes transforms along that path:
        T_target_source = (T_root_target)^-1 T_root_source  (computed via the common ancestor).
    """

    def __init__(self):
        self.parent = {}   # child -> parent
        self.T = {}        # child -> T_parent_child

    @property
    def frames(self):
        return set(self.parent) | set(self.parent.values())

    def add(self, parent, child, T_parent_child):
        """Add (or update, if the same parent->child edge exists) a transform.

        Raise ValueError if `child` already has a different parent, if parent == child, if
        T is not in SE(3), or if the edge would create a cycle."""
        # TODO: validate (parent != child, is_se3, single parent, no cycle via self._ancestors(parent)), then store
        raise NotImplementedError("add")

    def _ancestors(self, frame):
        """[frame, parent, grandparent, ..., root]."""
        chain = [frame]
        while chain[-1] in self.parent:
            chain.append(self.parent[chain[-1]])
        return chain

    def _to_ancestor(self, frame, ancestor):
        """T_ancestor_frame, composed down the chain ancestor -> ... -> frame."""
        # TODO: walk up from frame to ancestor, left-multiplying by self.T[f] (subscript cancellation)
        raise NotImplementedError("_to_ancestor")

    def _common(self, target, source):
        """Lowest common ancestor of two frames (KeyError / LookupError if impossible)."""
        for f in (target, source):
            if f not in self.frames:
                raise KeyError(f"unknown frame '{f}'")
        up_s = self._ancestors(source)
        common = next((f for f in self._ancestors(target) if f in up_s), None)
        if common is None:
            raise LookupError(f"frames '{target}' and '{source}' are not connected")
        return common

    def path(self, target, source):
        """List of frames from target to source through their lowest common ancestor."""
        common = self._common(target, source)
        up_t, up_s = self._ancestors(target), self._ancestors(source)
        return up_t[:up_t.index(common) + 1] + up_s[:up_s.index(common)][::-1]

    def lookup(self, target, source):
        """T_target_source: pose of `source` expressed in `target` (tf2 lookup_transform order)."""
        # TODO: common = self._common(target, source); combine two _to_ancestor results (one inverted)
        raise NotImplementedError("lookup")


def transform_from_tf(translation, rotation_xyzw):
    """T from a tf2-style (translation, quaternion x,y,z,w) pair."""
    return rp_to_trans(quat_xyzw_to_rot(rotation_xyzw), translation)


def load_tree_json(path):
    """Build a TransformTree from a JSON file holding a list of tf2-style static transforms:
       {"transforms": [{"parent": .., "child": .., "translation": [x,y,z],
                        "rotation_xyzw": [qx,qy,qz,qw]}, ...]}"""
    with open(path) as fh:
        data = json.load(fh)
    tree = TransformTree()
    for e in data["transforms"]:
        tree.add(e["parent"], e["child"], transform_from_tf(e["translation"], e["rotation_xyzw"]))
    return tree


def load_track_csv(path):
    """Read a tracked pose sequence. Columns: t, r11..r33 (row-major), px, py, pz.
    Returns (t, Ts) with Ts of shape (N, 4, 4)."""
    d = np.loadtxt(path, delimiter=",", skiprows=1)
    t = d[:, 0]
    Ts = np.tile(np.eye(4), (len(t), 1, 1))
    Ts[:, :3, :3] = d[:, 1:10].reshape(-1, 3, 3)
    Ts[:, :3, 3] = d[:, 10:13]
    return t, Ts


# ---------------------------------------------------------------- analysis helpers
def axis_angle_between(a, b):
    """Angle (rad) between two direction vectors."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    c = a @ b / (np.linalg.norm(a) * np.linalg.norm(b))
    return math.acos(max(-1.0, min(1.0, c)))


def point_line_distance(x, q, s_hat):
    """Distance from point x to the line through q with direction s_hat."""
    s = np.asarray(s_hat, float) / np.linalg.norm(s_hat)
    d = np.asarray(x, float) - np.asarray(q, float)
    return float(np.linalg.norm(d - (d @ s) * s))
