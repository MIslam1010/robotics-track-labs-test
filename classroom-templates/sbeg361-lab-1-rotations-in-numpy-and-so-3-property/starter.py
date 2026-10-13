"""SBEG 361 Lab 1 -- Rotations in NumPy and SO(3) property tests (STARTER).

Fill in every function marked  # TODO.  Do not change function names or signatures:
the auto-checks in test_lab.py import this file with   LAB_IMPL=starter pytest -q

Notation follows Lynch & Park, *Modern Robotics* (MR), Sections 3.1-3.2.1:
  * R_sb is the orientation of frame {b} relative to frame {s} (columns = x_b, y_b, z_b written in {s}).
  * Rot(w_hat, theta) is the rotation operator about the unit axis w_hat by theta (MR p. 72).
  * R R_sb rotates {b} about an axis expressed in {s};  R_sb R rotates about an axis in {b} (MR Eqs. 3.23-3.24).
  * Planar frames are pairs (P, p) with P in SO(2) (MR Eqs. 3.5, 3.8-3.9).

Use NumPy only (no modern_robotics inside this file) -- the tests compare your answers with modern_robotics.
"""
import numpy as np

# ---------------------------------------------------------------------------
# Part A -- elementary rotations
# ---------------------------------------------------------------------------

def rot_x(theta):
    """Rot(x_hat, theta) as a 3x3 NumPy array (theta in radians).  (Given as an example.)"""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[1.0, 0.0, 0.0],
                     [0.0, c, -s],
                     [0.0, s, c]])


def rot_y(theta):
    """Rot(y_hat, theta) as a 3x3 NumPy array (theta in radians).  See MR p. 72."""
    # TODO: build the matrix; watch the sign of the sin terms (it differs from rot_x and rot_z)
    raise NotImplementedError("TODO: rot_y")


def rot_z(theta):
    """Rot(z_hat, theta) as a 3x3 NumPy array (theta in radians).  See MR p. 72."""
    # TODO
    raise NotImplementedError("TODO: rot_z")


def rot_axis_angle(omega_hat, theta):
    """Rot(omega_hat, theta) from the explicit formula on MR p. 72.

    omega_hat must be a unit 3-vector: raise ValueError if | ||omega_hat|| - 1 | > 1e-6.
    Hint: write c = cos(theta), s = sin(theta), v = 1 - c and fill the 9 entries.
    """
    # TODO
    raise NotImplementedError("TODO: rot_axis_angle")


# ---------------------------------------------------------------------------
# Part B -- SO(3) property tests
# ---------------------------------------------------------------------------

def so3_errors(R):
    """Return (orth_err, det) for a 3x3 matrix (raise ValueError if R is not 3x3).

    orth_err = Frobenius norm of R^T R - I   (MR Eq. 3.19)   -> np.linalg.norm
    det      = det(R)                         (MR Eq. 3.21 requires +1)
    Return plain Python floats.
    """
    # TODO
    raise NotImplementedError("TODO: so3_errors")


def is_rotation(R, tol=1e-3):
    """True if R is (numerically) in SO(3): shape 3x3, ||R^T R - I||_F < tol and |det R - 1| < tol.
    Return False (do not raise) for a wrong shape."""
    # TODO
    raise NotImplementedError("TODO: is_rotation")


def rot_inv(R):
    """Inverse of a rotation matrix (MR Proposition 3.3).  Do NOT call np.linalg.inv."""
    # TODO
    raise NotImplementedError("TODO: rot_inv")


def project_to_so3(M):
    """Closest rotation matrix to M in the Frobenius norm (SVD method).

    M = U S V^T  ->  R = U diag(1, 1, d) V^T with d = det(U V^T), so that det R = +1 always.
    """
    # TODO: np.linalg.svd returns U, S, Vt (Vt is already V transposed)
    raise NotImplementedError("TODO: project_to_so3")


# ---------------------------------------------------------------------------
# Part C -- the three uses of a rotation matrix (MR Section 3.2.1.2)
# ---------------------------------------------------------------------------

def change_frame(R_ab, p_b):
    """Change the reference frame of a vector: return p_a (subscript cancellation rule)."""
    # TODO
    raise NotImplementedError("TODO: change_frame")


def rotate_in_fixed(R_sb, R):
    """Rotate {b} by R about an axis expressed in the fixed frame {s} (MR Eq. 3.23)."""
    # TODO
    raise NotImplementedError("TODO: rotate_in_fixed")


def rotate_in_body(R_sb, R):
    """Rotate {b} by R about an axis expressed in the body frame {b} (MR Eq. 3.24)."""
    # TODO
    raise NotImplementedError("TODO: rotate_in_body")


def planar_compose(P, p, Q, q):
    """Compose planar frames: given {b} in {s} as (P, p) and {c} in {b} as (Q, q),
    return {c} in {s} as the tuple (R, r)  (MR Eqs. 3.8-3.9)."""
    # TODO
    raise NotImplementedError("TODO: planar_compose")


# ---------------------------------------------------------------------------
# Part D -- application: optical tracker data of a laparoscopic instrument
# ---------------------------------------------------------------------------

def load_tracker_csv(path):
    """Read the tracker file (given). Returns an (N, 3, 3) array; each row of the file holds
    sample_id, r11, r12, r13, r21, ..., r33 (row-major)."""
    data = np.loadtxt(path, delimiter=",", skiprows=1)
    return data[:, 1:10].reshape(-1, 3, 3)


def classify_samples(Rs, tol=1e-3):
    """Boolean NumPy array: True where sample k is a valid rotation matrix (use is_rotation)."""
    # TODO
    raise NotImplementedError("TODO: classify_samples")


def tool_orientations(R_sm_list, R_mt):
    """Orientation of the tool-tip frame {t} in the tracker frame {s} for every sample.
    Return an (N, 3, 3) array.  Which matrix goes first?  Use subscript cancellation."""
    # TODO
    raise NotImplementedError("TODO: tool_orientations")
