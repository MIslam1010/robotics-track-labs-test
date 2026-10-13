// SBEG 361 Lab 2 -- Exponential coordinates, matrix log and quaternions (C++ track): declarations.
//
// The same functions as starter.py / test_lab.py, in namespace `lab`, with Eigen types instead of NumPy arrays.
// Write your code in starter.cpp. Do not change these declarations: test_lab.cpp calls them.
//
// Notation follows Lynch & Park, *Modern Robotics* (MR), Sections 3.2.2-3.2.3 and Appendix B.3:
//   [w]        3x3 skew-symmetric matrix of w in R^3            (MR Eq. 3.30)
//   e^[w]theta Rodrigues' formula I + sin(t)[w] + (1-cos t)[w]^2 (MR Eq. 3.51)
//   log R      matrix logarithm algorithm                        (MR Sec. 3.2.3.3, Eqs. 3.58-3.61)
//   q          unit quaternion (q0, q1, q2, q3), scalar first    (MR Eqs. B.9-B.12, B.15)
//              stored as an Eigen::Vector4d in this order (NOT Eigen::Quaterniond, whose storage order differs).
//
// Python -> C++: ValueError -> std::invalid_argument; a tuple -> std::pair; an (N, 3) array of gyro samples ->
// Eigen::MatrixXd with N rows and 3 columns (row k = sample k); an (N+1, 3, 3) array -> std::vector<Eigen::Matrix3d>.
#ifndef SBEG361_LAB2_HPP
#define SBEG361_LAB2_HPP

#include <sbeg/eigen.hpp>  // always first: includes Eigen/Dense with the course settings

#include <utility>
#include <vector>

namespace lab {

/// Same threshold as modern_robotics NearZero.
constexpr double TOL = 1e-6;

// ---------------------------------------------------------------------------
// so(3) <-> R^3
// ---------------------------------------------------------------------------

/// Return the 3x3 skew-symmetric matrix [omega] (MR Eq. 3.30).
/// Example: vec_to_so3((1, 2, 3)) -> [[0, -3, 2], [3, 0, -1], [-2, 1, 0]]
Eigen::Matrix3d vec_to_so3(const Eigen::Vector3d& omega);

/// Return the 3-vector omega from a skew-symmetric matrix [omega].
Eigen::Vector3d so3_to_vec(const Eigen::Matrix3d& so3mat);

/// Split exponential coordinates w_hat*theta into (w_hat, theta), theta >= 0.
/// Throw std::invalid_argument if theta < TOL (axis undefined).
std::pair<Eigen::Vector3d, double> axis_ang3(const Eigen::Vector3d& expc3);

// ---------------------------------------------------------------------------
// exp and log
// ---------------------------------------------------------------------------

/// Matrix exponential of [w_hat]theta in so(3) by Rodrigues' formula (MR Eq. 3.51).
Eigen::Matrix3d matrix_exp3(const Eigen::Matrix3d& so3mat);

/// Matrix logarithm [w_hat]theta of R in SO(3), theta in [0, pi] (MR Sec. 3.2.3.3), all three cases:
///   (a) R = I               -> zero matrix
///   (b) tr R = -1 (theta=pi) -> Eqs. 3.58 / 3.59 / 3.60 (the first feasible one)
///   (c) otherwise           -> theta = arccos((tr R - 1)/2), [w_hat] = (R - R^T)/(2 sin theta)
Eigen::Matrix3d matrix_log3(const Eigen::Matrix3d& R);

// ---------------------------------------------------------------------------
// quaternions (MR App. B.3); q = (q0, q1, q2, q3), scalar first
// ---------------------------------------------------------------------------

/// Unit quaternion q of R, with q0 >= 0 (MR Eqs. B.10-B.11; printed B.11 has a typo: last entry is r21 - r12).
/// For q0 <= 0.5 (theta >= 120 deg) build q from the matrix log instead (Eq. B.9).
Eigen::Vector4d rot_to_quat(const Eigen::Matrix3d& R);

/// Rotation matrix of a unit quaternion (MR Eq. B.12). Normalise q first.
Eigen::Matrix3d quat_to_rot(const Eigen::Vector4d& q);

/// Quaternion of the product R_q R_p (MR Eq. B.15).
Eigen::Vector4d quat_mul(const Eigen::Vector4d& q, const Eigen::Vector4d& p);

/// Conjugate (= inverse for a unit quaternion): the rotation R_q^T.
Eigen::Vector4d quat_conj(const Eigen::Vector4d& q);

// ---------------------------------------------------------------------------
// IMU orientation
// ---------------------------------------------------------------------------

/// Integrate body-frame gyro samples with the exponential update R_{k+1} = R_k exp([omega_b,k] dt).
/// omega_b has N rows and 3 columns (row k = sample k). Return N+1 matrices; the first one is R0.
std::vector<Eigen::Matrix3d> integrate_gyro_exp(const Eigen::Matrix3d& R0, const Eigen::MatrixXd& omega_b,
                                                double dt);

/// First-order (forward Euler) update R_{k+1} = R_k (I + [omega_b,k] dt) -- for comparison.
/// Same input/output shapes as integrate_gyro_exp.
std::vector<Eigen::Matrix3d> integrate_gyro_euler(const Eigen::Matrix3d& R0, const Eigen::MatrixXd& omega_b,
                                                  double dt);

/// Exponential coordinates w_hat*theta (in the current body frame) of the rotation that takes
/// R_cur to R_des, i.e. R_des = R_cur exp([w_hat] theta).
Eigen::Vector3d orientation_error(const Eigen::Matrix3d& R_cur, const Eigen::Matrix3d& R_des);

/// Angle (rad, in [0, pi]) of the relative rotation R1^T R2.
double rotation_angle(const Eigen::Matrix3d& R1, const Eigen::Matrix3d& R2);

/// Signed knee flexion angle (rad): y-component of log(R_thigh^T R_shank).
/// Convention for this lab: both IMUs have their y-axis along the knee flexion axis,
/// so R_shank = R_thigh Rot(y_hat, kappa).
double knee_flexion(const Eigen::Matrix3d& R_thigh, const Eigen::Matrix3d& R_shank);

}  // namespace lab

#endif  // SBEG361_LAB2_HPP
