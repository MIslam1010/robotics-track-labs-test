// SBEG 46X Lab 11 -- Simulated powered wheelchair: odometry, EKF landmark localisation, trajectory-tracking control
// (formal report 3) -- C++ STARTER. You edit this file.
//
// Fill in every function marked  // TODO  (replace the `throw std::logic_error("TODO: ...")` line), in the order
// A1 ... D2. Do not change function names or signatures (they are declared in lab.hpp, with the full description
// of each function, the same text as the docstrings of starter.py). Build and test from the cpp folder:
//     cmake -S . -B build -G Ninja && cmake --build build && ./build/lab_tests
// or from your Lab code folder:   python run_lab.py 11 --cpp
//
// The provided helpers are in helpers.cpp (do not edit): load_params, landmark_table, load_log, load_meas, wrap,
// rot2, reference_trajectory, pose_errors, distance_travelled, sim_noise, simulate, tracking_metrics.
// As in the Python lab you may use mrcpp, the C++ port of modern_robotics, in Part D (mr::fkin_body,
// mr::jacobian_body, mr::adjoint, mr::trans_inv).
//
// Eigen reminders:  Eigen::Matrix3d T;  T << a, b, c,  d, e, f,  g, h, i;   (comma initialiser, row by row)
//   A * B, A.transpose(), A.inverse() (small fixed size), S.partialPivLu().solve(nu), v.dot(w), v.cwiseAbs(),
//   v.asDiagonal(), Eigen::Matrix3d::Identity(), M.row(k) / M.col(i), M.middleRows(2, 3), M.topLeftCorner<3, 3>(),
//   T.topRightCorner<3, 1>() (= T[:3, 3]), J << A, B; (= np.hstack([A, B]) after J.resize(rows, cols)).
//   std::hypot, std::atan2, std::abs (never abs), std::optional: `if (gate && d2 > *gate)`.
//   Write the type of every Eigen result (Eigen::Matrix3d C = A * B;), never `auto`.
#include "lab.hpp"

#include <sbeg/mrcpp.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace lab {

// ==========================================================================================
// Part A -- wheeled-robot kinematics and odometry (MR Secs. 13.2-13.4)
// ==========================================================================================
Eigen::Vector3d wheel_row(double r, double gamma, double beta, double xw, double yw) {
    // TODO (A1): a = beta + gamma; return (xw sin a - yw cos a, cos a, sin a) / (r cos gamma).
    throw std::logic_error("TODO: wheel_row");
}

Eigen::MatrixXd H0_matrix(const std::vector<Wheel>& wheels) {
    // TODO (A2): one row per wheel: H.row(i) = wheel_row(w.r, w.gamma, w.beta, w.xw, w.yw).transpose().
    throw std::logic_error("TODO: H0_matrix");
}

Eigen::MatrixXd diffdrive_F(double r, double d) {
    // TODO (A3): MR Eq. 13.34, a 3 x 2 Eigen::MatrixXd.
    throw std::logic_error("TODO: diffdrive_F");
}

std::pair<double, double> wheel_speeds(double v, double omega, double r, double d) {
    // TODO (A4): return {u_L, u_R}.
    throw std::logic_error("TODO: wheel_speeds");
}

Eigen::Vector3d odometry_step(const Eigen::Vector3d& q, const Eigen::VectorXd& dtheta, const Eigen::MatrixXd& F) {
    // TODO (A5): V_b = F dtheta; Delta_q_b from MR Eq. 13.35 (two cases, |omega_bz| < 1e-12, keep v_by);
    //            rotate with phi_k (MR Eq. 13.36) and add to q.
    throw std::logic_error("TODO: odometry_step");
}

Eigen::MatrixXd dead_reckoning(const Eigen::Vector3d& q0, const Eigen::MatrixXd& enc, const Eigen::MatrixXd& F) {
    // TODO (A6): N x 3 result, row 0 = q0, row k+1 = odometry_step(row k, increment of the encoders, F).
    //            Hint: Eigen::VectorXd dth = (enc.row(k + 1) - enc.row(k)).transpose();
    throw std::logic_error("TODO: dead_reckoning");
}

// ==========================================================================================
// Part B -- EKF localisation with known landmarks (Corke Secs. 6.1-6.2; Week 10)
// ==========================================================================================
OdometryJacobians odometry_jacobians(const Eigen::Vector3d& q, const Eigen::Vector2d& dtheta, double r, double d) {
    // TODO (B1): q_next, F_q (3 x 3), F_u (3 x 2); series for |omega| < 1e-4 (lab.hpp). Return {q_next, F_q, F_u}.
    throw std::logic_error("TODO: odometry_jacobians");
}

Eigen::Matrix2d wheel_noise_cov(const Eigen::Vector2d& dtheta, double r, double k_s) {
    // TODO (B2): diag(k_s |dtheta_i| / r).
    throw std::logic_error("TODO: wheel_noise_cov");
}

std::pair<Eigen::Vector3d, Eigen::Matrix3d> ekf_predict(const Eigen::Vector3d& q, const Eigen::Matrix3d& P,
                                                        const Eigen::Vector2d& dtheta, double r, double d,
                                                        double k_s) {
    // TODO (B3): q- and P- = F_q P F_q^T + F_u Q_u F_u^T (symmetrise: 0.5 (P- + P-^T)).
    throw std::logic_error("TODO: ekf_predict");
}

Eigen::Vector2d range_bearing(const Eigen::Vector3d& q, const Eigen::Vector2d& m) {
    // TODO (B4): (rho, wrap(atan2(dy, dx) - phi)).
    throw std::logic_error("TODO: range_bearing");
}

Matrix23d range_bearing_jacobian(const Eigen::Vector3d& q, const Eigen::Vector2d& m) {
    // TODO (B5): H for the state order (phi, x, y).
    throw std::logic_error("TODO: range_bearing_jacobian");
}

EkfUpdate ekf_update(const Eigen::Vector3d& q, const Eigen::Matrix3d& P, const Eigen::Vector2d& z,
                     const Eigen::Vector2d& m, const Eigen::Matrix2d& R, const std::optional<double>& gate) {
    // TODO (B6): nu (bearing wrapped), S, d2; gate -> rejected (q, P unchanged, K = 0, accepted = false);
    //            otherwise K, q+, Joseph-form P+ (symmetrised). Fill every field of the EkfUpdate result.
    throw std::logic_error("TODO: ekf_update");
}

EkfRun ekf_localise(const Log& log, const std::vector<Reading>& meas, const Params& p,
                    const std::optional<std::vector<int>>& landmark_ids, bool use_gate,
                    const std::optional<double>& k_s) {
    // TODO (B7): predict with each encoder increment, then update with every reading of sample k + 1 (file order),
    //            skipping markers not in landmark_table(p, landmark_ids). Store q (N x 3) and P (N matrices).
    throw std::logic_error("TODO: ekf_localise");
}

// ==========================================================================================
// Part C -- feedback control of the nonholonomic base (MR Sec. 13.3.4)
// ==========================================================================================
Eigen::Vector3d error_coordinates(const Eigen::Vector3d& q, const Eigen::Vector3d& qd) {
    // TODO (C1): MR Eq. 13.30.
    throw std::logic_error("TODO: error_coordinates");
}

std::pair<double, double> tracking_control(const Eigen::Vector3d& q, const Eigen::Vector3d& qd, double vd, double wd,
                                           double k1, double k2, double k3) {
    // TODO (C2): MR Eq. 13.31.
    throw std::logic_error("TODO: tracking_control");
}

std::pair<double, double> point_controls(double phi, double xr, double yr, const Eigen::Vector2d& pdot) {
    // TODO (C3): invert MR Eq. 13.20 (MR Eq. 13.21).
    throw std::logic_error("TODO: point_controls");
}

std::pair<double, double> point_tracking_control(const Eigen::Vector3d& q, const Eigen::Vector3d& qd, double vd,
                                                 double wd, double xr, double kp) {
    // TODO (C4): p_d, pdot_d, p; pdot = pdot_d + kp (p_d - p); return point_controls(phi, xr, 0, pdot).
    throw std::logic_error("TODO: point_tracking_control");
}

std::pair<double, double> saturate_wheels(double v, double omega, double r, double d, double u_max,
                                          const std::optional<double>& v_max) {
    // TODO (C5): one common factor s >= 1.
    throw std::logic_error("TODO: saturate_wheels");
}

// ==========================================================================================
// Part D (optional) -- mobile manipulation: wheelchair + feeding arm (MR Sec. 13.5)
// ==========================================================================================
Eigen::MatrixXd base_jacobian(const Eigen::MatrixXd& F, const Eigen::Matrix4d& Tb0, const Eigen::Matrix4d& T0e) {
    // TODO (D1): F6 (6 x m), then [Ad_{T_0e^-1 T_b0^-1}] F6.
    throw std::logic_error("TODO: base_jacobian");
}

Eigen::MatrixXd mobile_manip_jacobian(const Eigen::MatrixXd& F, const Eigen::Matrix4d& Tb0,
                                      const Eigen::MatrixXd& Blist, const Eigen::Matrix4d& M,
                                      const Eigen::VectorXd& theta) {
    // TODO (D2): T_0e = mr::fkin_body(M, Blist, theta); J_e = [base_jacobian  mr::jacobian_body(Blist, theta)].
    throw std::logic_error("TODO: mobile_manip_jacobian");
}

}  // namespace lab
