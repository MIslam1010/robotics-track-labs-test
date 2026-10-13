"""SBEG 361 Lab 2 -- Exponential coordinates, matrix log and quaternions (STARTER).

Fill in every function marked  # TODO.  Do not change the function names or signatures:
the auto-checks in test_lab.py import them.  Run the tests with

    LAB_IMPL=starter python -m pytest -q test_lab.py

Notation follows Lynch & Park, *Modern Robotics* (MR), Sections 3.2.2-3.2.3 and Appendix B.3:
  [w]        3x3 skew-symmetric matrix of w in R^3            (MR Eq. 3.30)
  e^[w]theta Rodrigues' formula I + sin(t)[w] + (1-cos t)[w]^2 (MR Eq. 3.51)
  log R      matrix logarithm algorithm                        (MR Sec. 3.2.3.3, Eqs. 3.58-3.61)
  q          unit quaternion (q0, q1, q2, q3), scalar first      (MR Eqs. B.9-B.12, B.15)

You may NOT call modern_robotics, scipy.spatial.transform or scipy.linalg.expm/logm inside
these functions -- they are what you compare against in the tests.
"""
import numpy as np

TOL = 1e-6  # same threshold as modern_robotics.NearZero


# ---------------------------------------------------------------- so(3) <-> R^3
def vec_to_so3(omega):
    """Return the 3x3 skew-symmetric matrix [omega] (MR Eq. 3.30).

    Example: vec_to_so3([1, 2, 3]) -> [[0, -3, 2], [3, 0, -1], [-2, 1, 0]]
    """
    # TODO: build the matrix from the three components of omega
    raise NotImplementedError("vec_to_so3")


def so3_to_vec(so3mat):
    """Return the 3-vector omega from a skew-symmetric matrix [omega]."""
    # TODO: read omega back from entries (3,2), (1,3), (2,1) (1-based)
    raise NotImplementedError("so3_to_vec")


def axis_ang3(expc3):
    """Split exponential coordinates  w_hat*theta  into (w_hat, theta), theta >= 0.

    Raise ValueError if theta < TOL (axis undefined)."""
    # TODO
    raise NotImplementedError("axis_ang3")


# ---------------------------------------------------------------- exp and log
def matrix_exp3(so3mat):
    """Matrix exponential of [w_hat]theta in so(3) by Rodrigues' formula (MR Eq. 3.51).

    Hint: theta = ||so3_to_vec(so3mat)||; if theta < TOL return I; otherwise [w_hat] = so3mat/theta.
    """
    # TODO
    raise NotImplementedError("matrix_exp3")


def matrix_log3(R):
    """Matrix logarithm [w_hat]theta of R in SO(3), theta in [0, pi] (MR Sec. 3.2.3.3).

    Implement all three cases of the MR algorithm:
      (a) R = I               -> zero matrix
      (b) tr R = -1 (theta=pi) -> Eqs. 3.58 / 3.59 / 3.60 (use the first feasible one)
      (c) otherwise           -> theta = arccos((tr R - 1)/2), [w_hat] = (R - R^T)/(2 sin theta)
    Use a small tolerance (e.g. 1e-12) when comparing (tr R - 1)/2 with +1 and -1.
    """
    # TODO
    raise NotImplementedError("matrix_log3")


# ---------------------------------------------------------------- quaternions (MR App. B.3)
def rot_to_quat(R):
    """Unit quaternion q = (q0, q1, q2, q3) of R, with q0 >= 0 (MR Eqs. B.10-B.11).

    Note: printed Eq. B.11 has a typo in its last entry; it should read r21 - r12.
    For q0 <= 0.5 (theta >= 120 deg) build q from the matrix log instead (Eq. B.9),
    because dividing by 4*q0 becomes ill-conditioned near theta = pi.
    """
    # TODO
    raise NotImplementedError("rot_to_quat")


def quat_to_rot(q):
    """Rotation matrix of a unit quaternion (MR Eq. B.12). Normalise q first."""
    # TODO
    raise NotImplementedError("quat_to_rot")


def quat_mul(q, p):
    """Quaternion of the product R_q R_p (MR Eq. B.15)."""
    # TODO
    raise NotImplementedError("quat_mul")


def quat_conj(q):
    """Conjugate (= inverse for a unit quaternion): the rotation R_q^T."""
    # TODO
    raise NotImplementedError("quat_conj")


# ---------------------------------------------------------------- IMU orientation
def integrate_gyro_exp(R0, omega_b, dt):
    """Integrate body-frame gyro samples with the exponential update
        R_{k+1} = R_k exp([omega_b,k] dt)
    omega_b has shape (N, 3). Return an array of shape (N+1, 3, 3) whose first entry is R0."""
    # TODO
    raise NotImplementedError("integrate_gyro_exp")


def integrate_gyro_euler(R0, omega_b, dt):
    """First-order (forward Euler) update R_{k+1} = R_k (I + [omega_b,k] dt) -- for comparison.
    Same input/output shapes as integrate_gyro_exp."""
    # TODO
    raise NotImplementedError("integrate_gyro_euler")


def orientation_error(R_cur, R_des):
    """Exponential coordinates w_hat*theta (in the current body frame) of the rotation that takes
    R_cur to R_des, i.e. R_des = R_cur exp([w_hat] theta)."""
    # TODO
    raise NotImplementedError("orientation_error")


def rotation_angle(R1, R2):
    """Angle (rad, in [0, pi]) of the relative rotation R1^T R2."""
    # TODO
    raise NotImplementedError("rotation_angle")


def knee_flexion(R_thigh, R_shank):
    """Signed knee flexion angle (rad): y-component of log(R_thigh^T R_shank).

    Convention for this lab: both IMUs have their y-axis along the knee flexion axis,
    so R_shank = R_thigh Rot(y_hat, kappa)."""
    # TODO
    raise NotImplementedError("knee_flexion")
