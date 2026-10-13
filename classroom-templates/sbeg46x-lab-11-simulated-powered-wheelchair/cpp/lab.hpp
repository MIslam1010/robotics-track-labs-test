// SBEG 46X Lab 11 -- simulated powered wheelchair: odometry, EKF landmark localisation, trajectory-tracking control
// (formal report 3) -- C++ track: declarations.
//
// The same functions as starter.py / test_lab.py, in namespace `lab`, with Eigen types instead of NumPy arrays.
// Write your code in starter.cpp. Do not change these declarations: test_lab.cpp calls them. The provided helpers
// (parameters, data files, wrap, rot2, reference route, simulator, metrics) are given in helpers.cpp.
//
// Model (Modern Robotics Ch. 13; all values illustrative, data/wheelchair.json):
//   chassis configuration q = (phi, x, y) in the ward frame {s}; chassis frame {b} at the midpoint of the drive-wheel
//   axle, x_b forward; planar body twist V_b = (omega_bz, v_bx, v_by).
//   Differential drive (MR Eq. 13.15): drive-wheel radius r, half-track d, wheel speeds u = (u_L, u_R);
//   canonical controls (v, omega):  u_L = (v - omega d)/r,  u_R = (v + omega d)/r  (MR p. 528).
//   Odometry (MR Sec. 13.4): V_b = F Delta_theta (MR Eqs. 13.32-13.34), Delta_q_b from MR Eq. 13.35, rotated to {s}
//   with the previous heading phi_k (MR Eq. 13.36), q_{k+1} = q_k + Delta_q.
//   Range-bearing sensor at the origin of {b}: z = (rho, beta) to a marker m = (m_x, m_y) with a known ID
//   (Corke Sec. 6.2, Eq. 6.9, written here for the state ordering (phi, x, y) of MR).
//   EKF (Corke Secs. 6.1.2 and 6.2, Eqs. 6.4-6.7 and 6.10-6.16; Week 10 notation Q, R, except that
//   the Kalman gain is called K as in Corke -- the lecture notes and Week 10 call it L):
//       predict  q- = f(q, Dtheta),  P- = F_q P F_q^T + F_u Q_u F_u^T
//       update   nu = z - h(q-) (bearing wrapped),  S = H P- H^T + R,  K = P- H^T S^-1,
//                q = q- + K nu,  P = (I - K H) P- (I - K H)^T + K R K^T   (Joseph form)
//   Trajectory tracking (MR Sec. 13.3.4): error coordinates MR Eq. 13.30, nonlinear law MR Eq. 13.31; reference-point
//   law MR Eq. 13.29 (with the feed-forward of the reference-point velocity) converted to (v, omega) by MR Eq. 13.21.
//   Mobile manipulation (MR Sec. 13.5): J_base = [Ad_{T_0e^-1 T_b0^-1}] F6,  J_e = [J_base  J_arm].
// Angles are not wrapped in the state (phi grows continuously); innovations and errors are wrapped to (-pi, pi].
//
// Python -> C++: a pose q or 3-vector -> Eigen::Vector3d; a tuple of two numbers -> std::pair<double, double>;
// tuples / dicts of results -> small structs with the same field names; None -> std::nullopt (std::optional);
// the parameter dict p -> lab::Params (p["chair"]["r"] -> p.chair.r); an (N, 3, 3) stack -> std::vector<Matrix3d>.
#ifndef SBEG46X_LAB11_HPP
#define SBEG46X_LAB11_HPP

#include <sbeg/eigen.hpp>  // always first: includes Eigen/Dense with the course settings

#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace lab {

using Matrix23d = Eigen::Matrix<double, 2, 3>;
using Matrix32d = Eigen::Matrix<double, 3, 2>;

// ==========================================================================================
// Provided (given in helpers.cpp, do not edit): parameters, data, helpers, reference route, simulator, metrics
// ==========================================================================================

/// One segment of the ward route: kind "line" (a = length in m) or "arc" (a = radius in m, b = angle in deg, left).
struct RouteSegment {
    std::string kind;
    double a = 0.0;
    double b = 0.0;
};

/// Lab parameters (data/wheelchair.json, illustrative values), field names as the JSON keys.
struct Params {
    struct Chair {  ///< nominal model used by odometry, EKF and controller
        double r = 0, d = 0, u_max = 0, v_max = 0, h = 0;
        int enc_counts_per_rev = 0;
    } chair;
    struct Truth {  ///< the simulated chair (unknown to the filter)
        double r_L = 0, r_R = 0, d = 0, k_slip = 0;
    } truth;
    struct Sensor {
        int every = 1;
        double max_range = 0, sigma_range = 0, sigma_bearing_deg = 0, p_outlier = 0;
        Eigen::Vector2d outlier_short = Eigen::Vector2d::Zero();
    } sensor;
    Eigen::MatrixXd landmarks;  ///< one row [id, x, y] per ward marker
    struct Ekf {
        double k_s = 0, sigma_range = 0, sigma_bearing_deg = 0, gate = 0;
        Eigen::Vector3d P0_sigma = Eigen::Vector3d::Zero();
    } ekf;
    struct Route {
        double v = 0;
        int laps = 1;
        std::vector<RouteSegment> segments;
        Eigen::Vector3d start = Eigen::Vector3d::Zero();
    } route;
    struct Control {
        double k1 = 0, k2 = 0, k3 = 0, xr = 0, kp = 0;
        Eigen::Vector3d q0_offset = Eigen::Vector3d::Zero();
    } control;
    struct ArmMount {
        Eigen::Vector3d p_b0 = Eigen::Vector3d::Zero();
        Eigen::VectorXd theta_feed;
    } arm_mount;
};

/// Load data/wheelchair.json (or `path`, if not empty).
Params load_params(const std::string& path = "");

/// {id: (x, y)} of the ward markers (optionally only the listed IDs; std::nullopt = all).
std::map<int, Eigen::Vector2d> landmark_table(const Params& p,
                                              const std::optional<std::vector<int>>& ids = std::nullopt);

/// Recorded run (simulation): t (s), enc_L, enc_R (cumulative wheel angles from the encoders, rad, quantised to
/// 4096 counts/rev) and the ground truth phi, x, y (from the simulator -- 'motion capture').
struct Log {
    Eigen::VectorXd t, enc_L, enc_R, phi, x, y;
};
/// Load data/ward_log.csv (or `path`, if not empty).
Log load_log(const std::string& path = "");

/// One landmark reading; reading k belongs to sample k of the log (taken after the motion from sample k-1 to k).
struct Reading {
    int k = 0;
    double t = 0.0;
    int id = 0;
    double range = 0.0;
    double bearing = 0.0;
};
/// Load data/ward_meas.csv (or `path`, if not empty), in file order.
std::vector<Reading> load_meas(const std::string& path = "");

/// Wrap an angle to (-pi, pi]  (same numbers as Python's wrap).
double wrap(double a);
/// Wrap every element of a vector to (-pi, pi].
Eigen::VectorXd wrap(const Eigen::VectorXd& a);

/// 2 x 2 rotation matrix of angle phi.
Eigen::Matrix2d rot2(double phi);

/// Reference q_d(t) = (phi_d, x_d, y_d) and nominal controls (v_d, omega_d) of the ward loop of p.route, sampled
/// every h: constant speed v along straight lines and left arcs.  qd has one ROW per sample (N x 3).
struct Reference {
    Eigen::VectorXd t;
    Eigen::MatrixXd qd;
    Eigen::VectorXd vd, wd;
};
Reference reference_trajectory(const Params& p);

/// Errors of an estimate (N x 3 poses, one row each): e_pos (m, Euclidean in x-y) and e_phi (rad, wrapped).
struct PoseErrors {
    Eigen::VectorXd e_pos, e_phi;
};
PoseErrors pose_errors(const Eigen::MatrixXd& q_est, const Eigen::MatrixXd& q_true);

/// Cumulative path length of the chair (m) along a trajectory (N x 3).
Eigen::VectorXd distance_travelled(const Eigen::MatrixXd& q_true);

/// Noise sequences of the simulator (one set per seed): slip (N x 2) and, for sample index k = 0..N (rows) and
/// marker j in ID order (columns), the sensor noise n_range, n_bearing (standard normal), the outlier draw
/// u_outlier ~ U(0, 1) and the outlier size a_outlier ~ U(outlier_short).
struct SimNoise {
    Eigen::MatrixXd slip, n_range, n_bearing, u_outlier, a_outlier;
};
/// Noise for `seed`: for seed 1211 with the default route and 8 markers, the NumPy sequences of the Python
/// simulate() (stored in cpp/sim_noise_1211.json, so both tracks see the same noise); for any other seed, a C++
/// generator (std::mt19937_64 + Box-Muller): valid noise, but not the numbers of Python's default_rng(seed).
SimNoise sim_noise(const Params& p, int seed, Eigen::Index N);

/// Options of simulate() (the keyword arguments of the Python version).
struct SimOptions {
    int seed = 1211;
    std::optional<Eigen::Vector3d> offset;               ///< initial pose error; default p.control.q0_offset
    std::optional<std::vector<int>> landmark_ids;        ///< markers that exist; default all
    bool use_gate = true;                                ///< Mahalanobis gate on/off
    std::optional<double> k_s;                           ///< filter odometry-noise coefficient; default p.ekf.k_s
    std::map<std::string, double> gains;                 ///< overrides of "k1", "k2", "k3", "kp", "xr"
};

/// Result of simulate(): t, qd, vd, wd (N), q_true, q_est (N + 1 x 3), P (N + 1 matrices), v, w, uL, uR, sat (N),
/// enc (N + 1 x 2, quantised cumulative wheel angles), meas (the readings), n_acc, n_rej.
struct SimResult {
    Eigen::VectorXd t, vd, wd;
    Eigen::MatrixXd qd, q_true, q_est;
    std::vector<Eigen::Matrix3d> P;
    Eigen::VectorXd v, w, uL, uR;
    std::vector<bool> sat;
    Eigen::MatrixXd enc;
    std::vector<Reading> meas;
    int n_acc = 0;
    int n_rej = 0;
};

/// Closed-loop run of the wheelchair on the ward loop (see starter.py for the full description).
///   estimator  "truth" (the controller sees the true pose), "odometry" (dead reckoning, odometry_step) or "ekf"
///              (ekf_predict + ekf_update with the landmark readings)
///   controller "nonlinear" (tracking_control, MR Eq. 13.31) or "point" (point_tracking_control, MR Eqs. 13.29/13.21)
/// The estimator starts from the TRUE initial pose (docking station). The noise depends only on opt.seed.
/// Throws std::invalid_argument for an unknown estimator or controller.
SimResult simulate(const Params& p, const std::string& estimator = "ekf", const std::string& controller = "nonlinear",
                   const SimOptions& opt = SimOptions());

/// Tracking metrics of a closed-loop run (true pose vs reference) for t >= t_from: rms and max position error,
/// rms heading error, final position error, peak |u_L|, |u_R|, fraction of samples scaled by saturate_wheels.
struct TrackingMetrics {
    double rms_pos = 0, max_pos = 0, rms_phi = 0, final_pos = 0, peak_u = 0, frac_sat = 0;
};
TrackingMetrics tracking_metrics(const SimResult& sim, double t_from = 5.0);

// ==========================================================================================
// Part A -- wheeled-robot kinematics and odometry (MR Secs. 13.2-13.4)
// ==========================================================================================

/// A wheel of radius r at (xw, yw) in {b}, driving direction beta and free-sliding angle gamma
/// (the tuple (r, gamma, beta, xw, yw) of the Python version, same order).
struct Wheel {
    double r, gamma, beta, xw, yw;
};

/// A1. Row h_i(0) of MR Eq. 13.6 (phi = 0):  h_i(0) = (1/(r cos gamma)) [xw sin(beta+gamma) - yw cos(beta+gamma),
/// cos(beta+gamma), sin(beta+gamma)].  Returns a 3-vector (wheel driving speed per unit body twist).
Eigen::Vector3d wheel_row(double r, double gamma, double beta, double xw, double yw);

/// A2. H(0) of MR Eq. 13.8 (m x 3): stack wheel_row for each wheel (one ROW per wheel).
Eigen::MatrixXd H0_matrix(const std::vector<Wheel>& wheels);

/// A3. F of the differential drive (MR Eq. 13.34), 3 x 2: V_b = (omega_bz, v_bx, v_by) = F (u_L, u_R)
/// (or F (Delta theta_L, Delta theta_R) for one odometry interval).
Eigen::MatrixXd diffdrive_F(double r, double d);

/// A4. Wheel speeds of the diff-drive for the canonical controls (MR p. 528):
/// u_L = (v - omega d)/r,  u_R = (v + omega d)/r.  Returns (u_L, u_R) in rad/s.
std::pair<double, double> wheel_speeds(double v, double omega, double r, double d);

/// A5. One odometry update (MR Sec. 13.4): V_b = F dtheta (dtheta = wheel-angle increments, m of them; F is 3 x m),
/// Delta_q_b from MR Eq. 13.35 (the two cases omega_bz = 0 (|omega_bz| < 1e-12) and omega_bz != 0, including v_by),
/// Delta_q = [[1, 0, 0], [0, cos phi_k, -sin phi_k], [0, sin phi_k, cos phi_k]] Delta_q_b (MR Eq. 13.36),
/// returns q_{k+1} = q_k + Delta_q (phi not wrapped).
Eigen::Vector3d odometry_step(const Eigen::Vector3d& q, const Eigen::VectorXd& dtheta, const Eigen::MatrixXd& F);

/// A6. Odometry over a whole run: enc = N x m cumulative wheel angles (rad, one row per sample); returns N x 3 poses
/// (one row each) with row 0 = q0 and row k+1 = odometry_step(row k, enc.row(k+1) - enc.row(k), F).
Eigen::MatrixXd dead_reckoning(const Eigen::Vector3d& q0, const Eigen::MatrixXd& enc, const Eigen::MatrixXd& F);

// ==========================================================================================
// Part B -- EKF localisation with known landmarks (Corke Secs. 6.1-6.2; Week 10)
// ==========================================================================================

/// Result of odometry_jacobians (the Python tuple (q_next, F_q, F_u)).
struct OdometryJacobians {
    Eigen::Vector3d q_next;
    Eigen::Matrix3d F_q;
    Matrix32d F_u;
};

/// B1. Diff-drive odometry model q+ = f(q, dtheta) (= odometry_step with diffdrive_F) and its Jacobians
/// F_q = df/dq (3 x 3) and F_u = df/d(dtheta) (3 x 2), evaluated at (q, dtheta).
/// Hint: with omega = r (dth_R - dth_L)/(2d), v = r (dth_L + dth_R)/2, Delta_q_b = (omega, v sin(omega)/omega,
/// v (1 - cos omega)/omega); use the series sin w/w ~ 1 - w^2/6, (1 - cos w)/w ~ w/2 - w^3/24 for |w| < 1e-4.
OdometryJacobians odometry_jacobians(const Eigen::Vector3d& q, const Eigen::Vector2d& dtheta, double r, double d);

/// B2. Odometry noise Q_u (2 x 2) of the wheel-angle increments: each wheel's ground distance r dtheta_i has
/// variance k_s |r dtheta_i| (independent wheels), so var(dtheta_i) = k_s |dtheta_i| / r.
Eigen::Matrix2d wheel_noise_cov(const Eigen::Vector2d& dtheta, double r, double k_s);

/// B3. EKF prediction with the odometry increments: q- = f(q, dtheta), P- = F_q P F_q^T + F_u Q_u F_u^T
/// (Corke Eqs. 6.6-6.7).  Returns (q-, P-).
std::pair<Eigen::Vector3d, Eigen::Matrix3d> ekf_predict(const Eigen::Vector3d& q, const Eigen::Matrix3d& P,
                                                        const Eigen::Vector2d& dtheta, double r, double d,
                                                        double k_s);

/// B4. Predicted reading of marker m = (m_x, m_y) from pose q = (phi, x, y), sensor at the origin of {b}:
/// rho = sqrt(dx^2 + dy^2), beta = wrap(atan2(dy, dx) - phi), (dx, dy) = m - (x, y)  (Corke Eq. 6.9).
Eigen::Vector2d range_bearing(const Eigen::Vector3d& q, const Eigen::Vector2d& m);

/// B5. H = dh/dq (2 x 3) of range_bearing for the state ordering (phi, x, y):
/// [[0, -dx/rho, -dy/rho], [-1, dy/rho^2, -dx/rho^2]].
Matrix23d range_bearing_jacobian(const Eigen::Vector3d& q, const Eigen::Vector2d& m);

/// Result of ekf_update: the updated (q, P) and the Python info dict (nu, S, K, d2, accepted).
/// For a rejected reading q, P are returned unchanged and K is zero (Python: None).
struct EkfUpdate {
    Eigen::Vector3d q;
    Eigen::Matrix3d P;
    Eigen::Vector2d nu;
    Eigen::Matrix2d S;
    Matrix32d K;
    double d2 = 0.0;
    bool accepted = false;
};

/// B6. EKF update with one range-bearing reading z of marker m (Corke Eqs. 6.13-6.16):
/// nu = z - h(q) with the bearing component wrapped; S = H P H^T + R; d2 = nu^T S^-1 nu (squared Mahalanobis
/// distance, 'NIS').  If gate is given (not std::nullopt) and d2 > gate the reading is rejected (q, P returned
/// unchanged). Otherwise K = P H^T S^-1, q+ = q + K nu, P+ = (I - K H) P (I - K H)^T + K R K^T (Joseph form).
EkfUpdate ekf_update(const Eigen::Vector3d& q, const Eigen::Matrix3d& P, const Eigen::Vector2d& z,
                     const Eigen::Vector2d& m, const Eigen::Matrix2d& R,
                     const std::optional<double>& gate = std::nullopt);

/// Result of ekf_localise: q (N x 3, one row per sample), P (N matrices), n_acc, n_rej, d2 (the squared
/// Mahalanobis distances of all readings used, in order).
struct EkfRun {
    Eigen::MatrixXd q;
    std::vector<Eigen::Matrix3d> P;
    int n_acc = 0;
    int n_rej = 0;
    std::vector<double> d2;
};

/// B7. Run the EKF over a recorded log: start from the true initial pose with P0 = diag(P0_sigma^2) (p.ekf);
/// for k = 0..N-2 predict with the encoder increments enc[k+1] - enc[k] (nominal r, d of p.chair), then update with
/// every reading whose k equals k+1 (in file order), skipping markers not in landmark_ids (default: all).
/// R = diag(sigma_range^2, sigma_bearing^2) of p.ekf; gate = p.ekf.gate if use_gate; k_s default p.ekf.k_s.
EkfRun ekf_localise(const Log& log, const std::vector<Reading>& meas, const Params& p,
                    const std::optional<std::vector<int>>& landmark_ids = std::nullopt, bool use_gate = true,
                    const std::optional<double>& k_s = std::nullopt);

// ==========================================================================================
// Part C -- feedback control of the nonholonomic base (MR Sec. 13.3.4)
// ==========================================================================================

/// C1. Error coordinates of MR Eq. 13.30: phi_e = wrap(phi - phi_d) and (x_e, y_e) = the error
/// (x - x_d, y - y_d) expressed in the reference frame {d}.  Returns (phi_e, x_e, y_e).
Eigen::Vector3d error_coordinates(const Eigen::Vector3d& q, const Eigen::Vector3d& qd);

/// C2. Nonlinear feed-forward + feedback tracking law of MR Eq. 13.31:
/// v = (v_d - k1 |v_d| (x_e + y_e tan phi_e)) / cos phi_e,
/// omega = omega_d - (k2 v_d y_e + k3 |v_d| tan phi_e) cos^2 phi_e.   Returns (v, omega).
std::pair<double, double> tracking_control(const Eigen::Vector3d& q, const Eigen::Vector3d& qd, double vd, double wd,
                                           double k1, double k2, double k3);

/// C3. Controls (v, omega) that give the reference point P = (xr, yr) in {b} the world velocity
/// pdot = (xdot_P, ydot_P) (MR Eq. 13.21).  Singular for xr = 0.
std::pair<double, double> point_controls(double phi, double xr, double yr, const Eigen::Vector2d& pdot);

/// C4. Reference-point tracking (MR Eq. 13.29 + feed-forward): P is the point (xr, 0) of {b}; its desired position
/// is the same point of the reference chassis, p_d = (x_d + xr cos phi_d, y_d + xr sin phi_d), with velocity
/// pdot_d = (v_d cos phi_d - xr omega_d sin phi_d, v_d sin phi_d + xr omega_d cos phi_d).
/// Command pdot = pdot_d + kp (p_d - p) and convert with point_controls.  Returns (v, omega).
std::pair<double, double> point_tracking_control(const Eigen::Vector3d& q, const Eigen::Vector3d& qd, double vd,
                                                 double wd, double xr, double kp);

/// C5. Scale (v, omega) by ONE common factor s >= 1 so that |u_L|, |u_R| <= u_max and |v| <= v_max (if given):
/// s = max(1, max(|u_L|, |u_R|)/u_max, |v|/v_max).  Keeps the path curvature omega/v.  Returns (v/s, omega/s).
std::pair<double, double> saturate_wheels(double v, double omega, double r, double d, double u_max,
                                          const std::optional<double>& v_max = std::nullopt);

// ==========================================================================================
// Part D (optional) -- mobile manipulation: wheelchair + feeding arm (MR Sec. 13.5)
// ==========================================================================================

/// D1. J_base = [Ad_{T_0e^-1 T_b0^-1}] F6 (MR Sec. 13.5), with F6 = F padded to 6 rows: two rows of zeros above
/// F and one below, so that V_b6 = (0, 0, omega_bz, v_bx, v_by, 0) = F6 u.  Returns 6 x m.
Eigen::MatrixXd base_jacobian(const Eigen::MatrixXd& F, const Eigen::Matrix4d& Tb0, const Eigen::Matrix4d& T0e);

/// D2. J_e = [J_base(theta)  J_arm(theta)] (6 x (m + n)) of MR Sec. 13.5: T_0e = M e^[B1 th1] ... e^[Bn thn]
/// (body PoE), J_arm = body Jacobian of the arm (MR Eq. 5.18; you may call mr::fkin_body / mr::jacobian_body).
Eigen::MatrixXd mobile_manip_jacobian(const Eigen::MatrixXd& F, const Eigen::Matrix4d& Tb0,
                                      const Eigen::MatrixXd& Blist, const Eigen::Matrix4d& M,
                                      const Eigen::VectorXd& theta);

}  // namespace lab

#endif  // SBEG46X_LAB11_HPP
