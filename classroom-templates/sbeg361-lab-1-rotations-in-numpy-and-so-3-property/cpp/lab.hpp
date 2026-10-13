// SBEG 361 Lab 1 -- Rotations with Eigen and SO(3) property tests (C++ track): declarations.
//
// The same functions as starter.py / test_lab.py, in namespace `lab`, with Eigen types instead of NumPy arrays.
// Write your code in starter.cpp. Do not change these declarations: test_lab.cpp calls them.
//
// Notation follows Lynch & Park, *Modern Robotics* (MR), Sections 3.1-3.2.1:
//   * R_sb is the orientation of frame {b} relative to frame {s} (columns = x_b, y_b, z_b written in {s}).
//   * Rot(w_hat, theta) is the rotation operator about the unit axis w_hat by theta (MR p. 72).
//   * R R_sb rotates {b} about an axis expressed in {s};  R_sb R rotates about an axis in {b} (MR Eqs. 3.23-3.24).
//   * Planar frames are pairs (P, p) with P in SO(2) (MR Eqs. 3.5, 3.8-3.9).
//
// Python -> C++: ValueError -> std::invalid_argument; a tuple -> std::pair; an (N, 3, 3) array ->
// std::vector<Eigen::Matrix3d>; a boolean array -> std::vector<bool>.
#ifndef SBEG361_LAB1_HPP
#define SBEG361_LAB1_HPP

#include <sbeg/eigen.hpp>  // always first: includes Eigen/Dense with the course settings

#include <string>
#include <utility>
#include <vector>

namespace lab {

// ---------------------------------------------------------------------------
// Part A -- elementary rotations
// ---------------------------------------------------------------------------

/// Rot(x_hat, theta) as a 3x3 matrix (theta in radians).  (Given as an example.)
Eigen::Matrix3d rot_x(double theta);

/// Rot(y_hat, theta) as a 3x3 matrix (theta in radians).  See MR p. 72.
Eigen::Matrix3d rot_y(double theta);

/// Rot(z_hat, theta) as a 3x3 matrix (theta in radians).  See MR p. 72.
Eigen::Matrix3d rot_z(double theta);

/// Rot(omega_hat, theta) from the explicit formula on MR p. 72.
/// omega_hat must be a unit 3-vector: throw std::invalid_argument if | ||omega_hat|| - 1 | > 1e-6.
Eigen::Matrix3d rot_axis_angle(const Eigen::Vector3d& omega_hat, double theta);

// ---------------------------------------------------------------------------
// Part B -- SO(3) property tests
// ---------------------------------------------------------------------------

/// Return (orth_err, det) for a 3x3 matrix (throw std::invalid_argument if R is not 3x3).
///   orth_err = Frobenius norm of R^T R - I   (MR Eq. 3.19)
///   det      = det(R)                         (MR Eq. 3.21 requires +1)
std::pair<double, double> so3_errors(const Eigen::MatrixXd& R);

/// True if R is (numerically) in SO(3): size 3x3, ||R^T R - I||_F < tol and |det R - 1| < tol.
/// Return false (do not throw) for a wrong size.
bool is_rotation(const Eigen::MatrixXd& R, double tol = 1e-3);

/// Inverse of a rotation matrix (MR Proposition 3.3).  Do NOT call .inverse().
Eigen::Matrix3d rot_inv(const Eigen::Matrix3d& R);

/// Closest rotation matrix to M in the Frobenius norm (SVD method).
/// M = U S V^T  ->  R = U diag(1, 1, d) V^T with d = det(U V^T), so that det R = +1 always.
Eigen::Matrix3d project_to_so3(const Eigen::Matrix3d& M);

// ---------------------------------------------------------------------------
// Part C -- the three uses of a rotation matrix (MR Section 3.2.1.2)
// ---------------------------------------------------------------------------

/// Change the reference frame of a vector: return p_a (subscript cancellation rule).
Eigen::Vector3d change_frame(const Eigen::Matrix3d& R_ab, const Eigen::Vector3d& p_b);

/// Rotate {b} by R about an axis expressed in the fixed frame {s} (MR Eq. 3.23).
Eigen::Matrix3d rotate_in_fixed(const Eigen::Matrix3d& R_sb, const Eigen::Matrix3d& R);

/// Rotate {b} by R about an axis expressed in the body frame {b} (MR Eq. 3.24).
Eigen::Matrix3d rotate_in_body(const Eigen::Matrix3d& R_sb, const Eigen::Matrix3d& R);

/// Compose planar frames: given {b} in {s} as (P, p) and {c} in {b} as (Q, q),
/// return {c} in {s} as the pair (R, r)  (MR Eqs. 3.8-3.9).
std::pair<Eigen::Matrix2d, Eigen::Vector2d> planar_compose(const Eigen::Matrix2d& P, const Eigen::Vector2d& p,
                                                           const Eigen::Matrix2d& Q, const Eigen::Vector2d& q);

// ---------------------------------------------------------------------------
// Part D -- application: optical tracker data of a laparoscopic instrument
// ---------------------------------------------------------------------------

/// Read the tracker file (given). Returns one 3x3 matrix per sample; each row of the file holds
/// sample_id, r11, r12, r13, r21, ..., r33 (row-major).
std::vector<Eigen::Matrix3d> load_tracker_csv(const std::string& path);

/// One flag per sample: true where sample k is a valid rotation matrix (use is_rotation).
std::vector<bool> classify_samples(const std::vector<Eigen::Matrix3d>& Rs, double tol = 1e-3);

/// Orientation of the tool-tip frame {t} in the tracker frame {s} for every sample.
/// Return one matrix per sample.  Which matrix goes first?  Use subscript cancellation.
std::vector<Eigen::Matrix3d> tool_orientations(const std::vector<Eigen::Matrix3d>& R_sm_list,
                                               const Eigen::Matrix3d& R_mt);

}  // namespace lab

#endif  // SBEG361_LAB1_HPP
