// SBEG 461 Lab 5 -- Trajectory library and torque feasibility on the course arm (C++ track): declarations.
//
// The same functions as starter.py / test_lab.py, in namespace `lab`, with Eigen types instead of NumPy arrays.
// Write your code in starter.cpp. Do not change these declarations: test_lab.cpp calls them.
// The provided helpers (skew ... export_csv, and the scalar / vector overloads marked "given") are in helpers.cpp.
//
// Notation follows Modern Robotics (MR) Ch. 9: a trajectory theta(t) = theta(s(t)) is a path theta(s),
// s in [0, 1], composed with a time scaling s(t), t in [0, T] (MR Sec. 9.1, Eqs. 9.1-9.2).
// Dynamics in MR list form (MR Sec. 8.3.2): Mlist, Glist, Slist, gravity g.
//
// Parts
//   A. Point-to-point time scalings: cubic, quintic, trapezoid       MR Sec. 9.2.2, Eqs. 9.9-9.24
//      + straight-line joint trajectories                             MR Eqs. 9.3-9.5
//   B. Via points: cubic via-point interpolation                      MR Sec. 9.3, Eqs. 9.25-9.29
//      + uniform cubic B-spline                                       MR p. 338; Gotor Sec. 7.1.4
//   C. Torque feasibility on the course arm                           MR Eqs. 8.50-8.54 (RNEA), 9.30-9.31
//      + uniform time scaling (stretching a trajectory by c > 0)      MR Sec. 9.7 (Hollerbach)
//
// Python -> C++: an array of times t -> Eigen::VectorXd (a scalar t has its own overload, given);
// an (N, n) array -> Eigen::MatrixXd with one ROW per sample; the tuples (s, sdot, sddot), (T, t_a, v_peak),
// (v, a, T), (t, Q, Qd, Qdd), (B, Bd, Bdd), (p, dp, ddp) -> the small structs below (field names as in Python,
// so `auto [s, sd, sdd] = lab::cubic_time_scaling(T, t);` works); the robot dict -> lab::Robot; the tasks dict ->
// lab::Tasks; the result dicts of check_limits / min_time_scale -> lab::LimitCheck / lab::TimeScaleResult;
// method = 3, 5, "trap" -> lab::CUBIC, lab::QUINTIC, lab::TRAP; T = None -> any value (ignored for TRAP);
// ValueError -> std::invalid_argument.
#ifndef SBEG461_LAB5_HPP
#define SBEG461_LAB5_HPP

#include <sbeg/eigen.hpp>  // always first: includes Eigen/Dense with the course settings
#include <sbeg/mrcpp.hpp>  // mr::TransformList, mr::InertiaList (types only)

#include <string>
#include <utility>
#include <vector>

namespace lab {

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

/// The course arm (Python: the dict returned by load_course_arm).
struct Robot {
    Eigen::MatrixXd Slist;     ///< 6 x 6 space-frame screw axes (one per COLUMN)
    Eigen::Matrix4d M;         ///< home configuration of the tool frame
    mr::TransformList Mlist;   ///< 7 frames M_{i-1,i} (CoM frames, MR Sec. 8.3.2)
    mr::InertiaList Glist;     ///< 6 spatial inertias G_i (angular block first, MR Eq. 8.32)
    Eigen::Vector3d g;         ///< gravity in {s} [m/s^2]
    Eigen::VectorXd qdot_max;  ///< joint speed limits [rad/s] (6)
    Eigen::VectorXd tau_max;   ///< joint torque limits [N m] (6)
    Eigen::MatrixXd limits;    ///< joint position limits [rad], 6 x 2 (min, max)
    Eigen::VectorXd q_home;    ///< ready pose [rad] (6)
};

/// The task configurations of data/tasks.json (Python: the dict returned by load_tasks).
struct Tasks {
    Eigen::VectorXd A, B, C, D, W;  ///< task configurations [rad] (6 each)
    Eigen::VectorXd via_times;      ///< via times of A -> W -> B [s]
    Eigen::VectorXd qddot_lim;      ///< joint acceleration limits [rad/s^2] (6)
    double payload_kg = 0.0;
    std::string description;
    /// tasks[name] for name = "A", "B", "C", "D" or "W" (given); other names -> std::invalid_argument.
    const Eigen::VectorXd& config(const std::string& name) const;
};

/// (s, sdot, sddot) sampled at the times t (Python: a tuple of three arrays).
struct TimeScaling {
    Eigen::VectorXd s, sd, sdd;
};

/// (s, sdot, sddot) at one time (Python: a tuple of three scalars).
struct TimeScalingPoint {
    double s = 0.0, sd = 0.0, sdd = 0.0;
};

/// (T, t_a, v_peak) of a trapezoidal time scaling.
struct TrapezoidParams {
    double T = 0.0, t_a = 0.0, v_peak = 0.0;
};

/// (v, a, T) of the fastest trapezoidal straight-line motion.
struct TrapezoidLimits {
    double v = 0.0, a = 0.0, T = 0.0;
};

/// (t, Q, Qd, Qdd): t (N), Q/Qd/Qdd N x n (row k = sample k).
struct Trajectory {
    Eigen::VectorXd t;
    Eigen::MatrixXd Q, Qd, Qdd;
};

/// (B, Bd, Bdd) of a via-point trajectory, each len(t) x m.
struct ViaSamples {
    Eigen::MatrixXd B, Bd, Bdd;
};

/// (p, dp/du, d2p/du2) of a B-spline at one parameter value, each (m).
struct BSplinePoint {
    Eigen::VectorXd p, dp, ddp;
};

/// Result of check_limits (Python: dict(vel_ratio, tau_ratio, ok)).
struct LimitCheck {
    Eigen::VectorXd vel_ratio, tau_ratio;  ///< worst ratio per joint (n)
    bool ok = false;
};

/// Result of min_time_scale (Python: dict(c_vel, c_tau, c, binding)).
struct TimeScaleResult {
    double c_vel = 0.0, c_tau = 0.0, c = 0.0;
    std::pair<std::string, int> binding;  ///< ("velocity" or "torque", joint index 0-based)
};

/// Time-scaling methods of joint_trajectory (Python: method = 3, 5, "trap").
constexpr int CUBIC = 3;
constexpr int QUINTIC = 5;
constexpr int TRAP = 0;

// ---------------------------------------------------------------------------
// Provided helpers (given in helpers.cpp; do not edit)
// ---------------------------------------------------------------------------

/// [w] for a 3-vector (MR Eq. 3.30).
Eigen::Matrix3d skew(const Eigen::Vector3d& w);
/// T^{-1} for T in SE(3).
Eigen::Matrix4d trans_inv(const Eigen::Matrix4d& T);
/// [Ad_T] (MR Def. 3.20).
sbeg::Matrix6d adjoint(const Eigen::Matrix4d& T);
/// [ad_V] (MR Eq. 8.38).
sbeg::Matrix6d ad(const sbeg::Vector6d& V);
/// e^{[S] theta} for a screw axis with ||omega|| = 1 or (omega = 0, ||v|| = 1) (MR Prop. 3.25).
Eigen::Matrix4d matrix_exp6(const sbeg::Vector6d& S, double theta);

/// Recursive Newton-Euler inverse dynamics (MR Eqs. 8.50-8.54), same interface as mr::inverse_dynamics.
/// This is the Lab 2/Lab 4 function; you may paste your own.
Eigen::VectorXd rnea(const Eigen::VectorXd& thetalist, const Eigen::VectorXd& dthetalist,
                     const Eigen::VectorXd& ddthetalist, const Eigen::Vector3d& g, const sbeg::Vector6d& Ftip,
                     const mr::TransformList& Mlist, const mr::InertiaList& Glist, const Eigen::MatrixXd& Slist);

/// The course arm (from the course robot models in Course resources, numbers identical; data/course_arm.json).
/// payload = true uses the 0.25 kg maximum payload at the tool point. derated = true applies the hardware-run
/// derating (50 % of rated speed, 60 % of rated torque). All inertial data and ratings are ILLUSTRATIVE values.
Robot load_course_arm(bool payload = true, bool derated = true);

/// Task configurations A, B, C, D, W (rad) and via times (s) of the lab (data/tasks.json; illustrative).
Tasks load_tasks();

/// Write t, q1..qn, qd1..qdn (SI units) to a CSV file (the quintic baseline for Lab 6; not a hardware command).
void export_csv(const std::string& path, const Eigen::VectorXd& t, const Eigen::MatrixXd& Q,
                const Eigen::MatrixXd& Qd);

// ---------------------------------------------------------------------------
// Part A -- point-to-point time scalings (MR Sec. 9.2.2)
// ---------------------------------------------------------------------------

/// Cubic rest-to-rest time scaling s = 3(t/T)^2 - 2(t/T)^3 (MR Eq. 9.9 with a2 = 3/T^2, a3 = -2/T^3).
///
/// Every t is clipped to [0, T]. Returns (s, sdot, sddot), each with the length of t.
/// At t = 0 and t = T return the one-sided values (sddot = +-6/T^2).
TimeScaling cubic_time_scaling(double T, const Eigen::VectorXd& t);
/// Scalar t (given: calls the vector version).
TimeScalingPoint cubic_time_scaling(double T, double t);

/// Quintic rest-to-rest time scaling s = 10 r^3 - 15 r^4 + 6 r^5, r = t/T, with zero end velocities
/// and accelerations (MR Sec. 9.2.2.1, Exercise 9.5; Fig. 9.4). Returns (s, sdot, sddot).
TimeScaling quintic_time_scaling(double T, const Eigen::VectorXd& t);
/// Scalar t (given: calls the vector version).
TimeScalingPoint quintic_time_scaling(double T, double t);

/// Total time and acceleration time of a trapezoidal time scaling with s-units v (1/s) and a (1/s^2).
///
/// Three-stage profile if v^2/a <= 1:  T = (a + v^2)/(v a), t_a = v/a  (MR p. 334).
/// If v^2/a > 1 the cruise speed is never reached (MR Exercise 9.10): bang-bang profile with
/// t_a = T/2 = 1/sqrt(a) and peak speed sqrt(a). Returns (T, t_a, v_peak).
TrapezoidParams trapezoid_params(double v, double a);

/// Trapezoidal time scaling (MR Eqs. 9.16-9.24); bang-bang (triangle) if v^2/a > 1.
/// Returns (s, sdot, sddot) at the times t (clipped to [0, T]).
TimeScaling trapezoid_time_scaling(double v, double a, const Eigen::VectorXd& t);
/// Scalar t (given: calls the vector version).
TimeScalingPoint trapezoid_time_scaling(double v, double a, double t);

/// Fastest trapezoidal straight-line motion (MR Eqs. 9.14-9.15): the largest v and a (s-units) with
/// |(theta_end - theta_start) v| <= qdot_lim and |(theta_end - theta_start) a| <= qddot_lim, joint by
/// joint (joints that do not move impose no limit). The limits are vectors (one value per joint).
/// Returns (v, a, T).
TrapezoidLimits trapezoid_from_limits(const Eigen::VectorXd& theta_start, const Eigen::VectorXd& theta_end,
                                      const Eigen::VectorXd& qdot_lim, const Eigen::VectorXd& qddot_lim);

/// Straight line in joint space theta(s) = theta_start + s (theta_end - theta_start) (MR Eq. 9.3),
/// time-scaled with method = CUBIC (3), QUINTIC (5) or TRAP (trapezoid; needs v and a and ignores T).
///
/// Returns (t, Q, Qd, Qdd): t (N), Q/Qd/Qdd N x n, with Qd = sdot D and Qdd = sddot D (MR Eqs. 9.4-9.5;
/// the path is straight so d2theta/ds2 = 0). The rows of Q equal mr::joint_trajectory (modern_robotics).
Trajectory joint_trajectory(const Eigen::VectorXd& theta_start, const Eigen::VectorXd& theta_end, double T, int N,
                            int method = QUINTIC, double v = 0.0, double a = 0.0);

// ---------------------------------------------------------------------------
// Part B -- via points (MR Sec. 9.3) and B-splines
// ---------------------------------------------------------------------------

/// Cubic via-point interpolation of ONE coordinate (MR Eqs. 9.26-9.29).
///
/// beta, beta_dot: (k) via positions and velocities; times: (k) increasing via times T_1 = 0 .. T_k.
/// Returns a (k-1) x 4 matrix whose row j is (a_j0, a_j1, a_j2, a_j3) for segment j.
Eigen::MatrixXd via_cubic_coefficients(const Eigen::VectorXd& beta, const Eigen::VectorXd& beta_dot,
                                       const Eigen::VectorXd& times);

/// Via velocities from positions and times (MR p. 337: "a heuristic could be used").
/// Rule used in this course (as in Lecture B), per coordinate: zero at the first and last via; at an
/// interior via, the average of the slopes of the two adjacent segments if both slopes have the same
/// strict sign, otherwise zero (the via is a turning point, or one side is flat).
/// points: k x m (one via per row; one coordinate = a k x 1 matrix); returns k x m.
Eigen::MatrixXd heuristic_via_velocities(const Eigen::MatrixXd& points, const Eigen::VectorXd& times);

/// Evaluate the cubic via-point trajectory of all m coordinates at the times t (clipped to [T_1, T_k]).
/// points, velocities: k x m; times: (k). Returns (B, Bd, Bdd), each len(t) x m.
ViaSamples via_trajectory(const Eigen::MatrixXd& points, const Eigen::MatrixXd& velocities,
                          const Eigen::VectorXd& times, const Eigen::VectorXd& t);

/// The four uniform cubic B-spline blending functions at local parameter u in [0, 1]:
/// b0 = (1-u)^3/6, b1 = (3u^3 - 6u^2 + 4)/6, b2 = (-3u^3 + 3u^2 + 3u + 1)/6, b3 = u^3/6.
Eigen::Vector4d bspline_basis(double u);
/// Vector u (given: calls the scalar version for each entry). Returns 4 x len(u) (column k = basis at u(k)).
Eigen::MatrixXd bspline_basis(const Eigen::VectorXd& u);

/// Uniform cubic B-spline with control points ctrl (n x m, one point per row), n >= 4, at global parameter
/// u in [0, n-3] (segment i = floor(u) uses P_i..P_{i+3}, local parameter u - i).
/// Returns (p, dp/du, d2p/du2), each (m). The curve is C2 and lies in the convex hull of the
/// control points (MR p. 338).
BSplinePoint uniform_cubic_bspline(const Eigen::MatrixXd& ctrl, double u);

// ---------------------------------------------------------------------------
// Part C -- torque feasibility (MR Eqs. 9.30-9.31) and uniform time scaling
// ---------------------------------------------------------------------------

/// Joint torques tau_k = M(q_k) qdd_k + c(q_k, qd_k) + g(q_k) along a sampled trajectory, by the RNEA.
/// robot: from load_course_arm; Q, Qd, Qdd: N x n. Returns N x n.
Eigen::MatrixXd torque_profile(const Robot& robot, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& Qd,
                               const Eigen::MatrixXd& Qdd);

/// Worst |velocity|/limit and |torque|/limit per joint along a trajectory.
/// Returns (vel_ratio (n), tau_ratio (n), ok); ok if every ratio <= 1 + tol.
LimitCheck check_limits(const Robot& robot, const Eigen::MatrixXd& Qd, const Eigen::MatrixXd& Tau,
                        double tol = 1e-9);

/// Smallest uniform stretch factor c > 0 that makes a sampled trajectory feasible.
///
/// Stretching time by c (theta_c(t) = theta(t/c)) divides velocities by c and accelerations by c^2, and
/// because c(theta, thetadot) is quadratic in thetadot (MR Sec. 9.4, c = thetadot^T Gamma thetadot)
///     tau_c = g(theta) + d / c^2,   d = tau(Q, Qd, Qdd) - g(theta)     (MR Sec. 9.7, Hollerbach 1984).
/// Velocity limits give c >= max |qd| / qdot_max. Torque limits give, sample by sample and joint by joint,
///     d > 0:  c^2 >= d / (tau_max - g),      d < 0:  c^2 >= -d / (tau_max + g),
/// which needs |g| < tau_max (otherwise the robot cannot even hold the configuration: throw
/// std::invalid_argument).
///
/// Returns (c_vel, c_tau, c = max(c_vel, c_tau), binding = ("velocity" or "torque", joint index 0-based)).
/// For a rest-to-rest trajectory computed with T = 1, c is the minimum feasible duration in seconds.
TimeScaleResult min_time_scale(const Robot& robot, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& Qd,
                               const Eigen::MatrixXd& Qdd);

}  // namespace lab

#endif  // SBEG461_LAB5_HPP
