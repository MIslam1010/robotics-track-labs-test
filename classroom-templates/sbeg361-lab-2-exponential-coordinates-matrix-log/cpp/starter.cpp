// SBEG 361 Lab 2 -- Exponential coordinates, matrix log and quaternions (C++ STARTER).
//
// Fill in every function marked  // TODO  (replace the `throw std::logic_error("TODO: ...")` line).
// Do not change function names or signatures (they are declared in lab.hpp). Build and test from the cpp folder:
//     cmake -S . -B build -G Ninja && cmake --build build && ./build/lab_tests
// or from your Lab code folder:   python run_lab.py 2 --cpp
//
// Notation follows Lynch & Park, *Modern Robotics* (MR), Sections 3.2.2-3.2.3 and Appendix B.3 (see lab.hpp).
// You may NOT call mrcpp (mr::...) inside this file -- the tests compare your answers with mrcpp, the C++ port of
// modern_robotics, and with numbers computed by the Python reference (modern_robotics and SciPy).
//
// Eigen reminders:  Eigen::Matrix3d M;  M << a, b, c,  d, e, f,  g, h, i;   (comma initialiser, row by row)
//   M.transpose(), M.trace(), v.norm(), Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(), A * B (product),
//   M(i, j) (0-based), omega_b.rows(), omega_b.row(k).transpose() (row k as a column Vector3d),
//   Eigen::Vector4d q(q0, q1, q2, q3).  Use std::sqrt, std::sin, std::cos, std::acos, std::abs.
#include "lab.hpp"

#include <cmath>
#include <stdexcept>

namespace lab {

// ---------------------------------------------------------------------------
// so(3) <-> R^3
// ---------------------------------------------------------------------------

Eigen::Matrix3d vec_to_so3(const Eigen::Vector3d& omega) {
    // TODO: build the matrix from the three components of omega
    throw std::logic_error("TODO: vec_to_so3");
}

Eigen::Vector3d so3_to_vec(const Eigen::Matrix3d& so3mat) {
    // TODO: read omega back from entries (3,2), (1,3), (2,1) (1-based) -- in C++ indices start at 0
    throw std::logic_error("TODO: so3_to_vec");
}

std::pair<Eigen::Vector3d, double> axis_ang3(const Eigen::Vector3d& expc3) {
    // TODO: throw std::invalid_argument("rotation angle is (near) zero: axis undefined") if theta < TOL;
    //       return {w_hat, theta};
    throw std::logic_error("TODO: axis_ang3");
}

// ---------------------------------------------------------------------------
// exp and log
// ---------------------------------------------------------------------------

Eigen::Matrix3d matrix_exp3(const Eigen::Matrix3d& so3mat) {
    // TODO: Rodrigues' formula (MR Eq. 3.51).
    // Hint: theta = ||so3_to_vec(so3mat)||; if theta < TOL return I; otherwise [w_hat] = so3mat / theta.
    throw std::logic_error("TODO: matrix_exp3");
}

Eigen::Matrix3d matrix_log3(const Eigen::Matrix3d& R) {
    // TODO: implement all three cases of the MR algorithm:
    //   (a) R = I               -> zero matrix
    //   (b) tr R = -1 (theta=pi) -> Eqs. 3.58 / 3.59 / 3.60 (use the first feasible one)
    //   (c) otherwise           -> theta = arccos((tr R - 1)/2), [w_hat] = (R - R^T)/(2 sin theta)
    // Use a small tolerance (e.g. 1e-12) when comparing (tr R - 1)/2 with +1 and -1.
    throw std::logic_error("TODO: matrix_log3");
}

// ---------------------------------------------------------------------------
// quaternions (MR App. B.3)
// ---------------------------------------------------------------------------

Eigen::Vector4d rot_to_quat(const Eigen::Matrix3d& R) {
    // TODO: unit quaternion with q0 >= 0 (MR Eqs. B.10-B.11).
    // Note: printed Eq. B.11 has a typo in its last entry; it should read r21 - r12.
    // For q0 <= 0.5 (theta >= 120 deg) build q from the matrix log instead (Eq. B.9),
    // because dividing by 4*q0 becomes ill-conditioned near theta = pi.
    throw std::logic_error("TODO: rot_to_quat");
}

Eigen::Matrix3d quat_to_rot(const Eigen::Vector4d& q) {
    // TODO: MR Eq. B.12. Normalise q first.
    throw std::logic_error("TODO: quat_to_rot");
}

Eigen::Vector4d quat_mul(const Eigen::Vector4d& q, const Eigen::Vector4d& p) {
    // TODO: MR Eq. B.15
    throw std::logic_error("TODO: quat_mul");
}

Eigen::Vector4d quat_conj(const Eigen::Vector4d& q) {
    // TODO
    throw std::logic_error("TODO: quat_conj");
}

// ---------------------------------------------------------------------------
// IMU orientation
// ---------------------------------------------------------------------------

std::vector<Eigen::Matrix3d> integrate_gyro_exp(const Eigen::Matrix3d& R0, const Eigen::MatrixXd& omega_b,
                                                double dt) {
    // TODO: R_{k+1} = R_k exp([omega_b,k] dt) for every row k of omega_b.
    // Return a std::vector with omega_b.rows() + 1 matrices whose first entry is R0 (use push_back).
    throw std::logic_error("TODO: integrate_gyro_exp");
}

std::vector<Eigen::Matrix3d> integrate_gyro_euler(const Eigen::Matrix3d& R0, const Eigen::MatrixXd& omega_b,
                                                  double dt) {
    // TODO: R_{k+1} = R_k (I + [omega_b,k] dt); same shapes as integrate_gyro_exp
    throw std::logic_error("TODO: integrate_gyro_euler");
}

Eigen::Vector3d orientation_error(const Eigen::Matrix3d& R_cur, const Eigen::Matrix3d& R_des) {
    // TODO
    throw std::logic_error("TODO: orientation_error");
}

double rotation_angle(const Eigen::Matrix3d& R1, const Eigen::Matrix3d& R2) {
    // TODO
    throw std::logic_error("TODO: rotation_angle");
}

double knee_flexion(const Eigen::Matrix3d& R_thigh, const Eigen::Matrix3d& R_shank) {
    // TODO
    throw std::logic_error("TODO: knee_flexion");
}

}  // namespace lab
