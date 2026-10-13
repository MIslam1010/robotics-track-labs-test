// SBEG 461 Lab 6 -- time-optimal time scaling (MR Sec. 9.4) (C++ track): declarations.
//
// The same functions as starter.py / test_lab.py, in namespace `lab`, with Eigen types instead of NumPy arrays.
// Write your code in starter.cpp. Do not change these declarations: test_lab.cpp calls them.
// These functions become `timescaling` in your cumulative sbeg461_dynamics library.
//
// Notation (MR Ch. 9): path theta(s), s in [0, 1]; time scaling s(t); theta' = d theta/ds,
// theta'' = d^2 theta/ds^2.  Along the path the dynamics are (MR Eqs. 9.32-9.33)
//
//     m(s) sdd + c(s) sd^2 + g(s) = tau,
//     m(s) = M(theta) theta',  c(s) = M(theta) theta'' + theta'^T Gamma(theta) theta',  g(s) = g(theta),
//
// and the actuator limits tau_min <= tau <= tau_max become L(s, sd) <= sdd <= U(s, sd) (MR Eqs. 9.36-9.37).
// Units: SI (m, kg, s, rad, N m).
//
// Python -> C++: a model dict -> lab::Model; a path object / trajectory callable -> lab::Path / lab::Trajectory
// (std::function, called as path(s, order) / traj(t, order)); a returned dict or tuple of 3+ values -> a small
// struct with the same field names; an optional argument `x=None` -> an EMPTY Eigen::VectorXd (the default);
// RuntimeError -> std::runtime_error; toppra -> toppralite (given, see toppralite.hpp).
#ifndef SBEG461_LAB6_HPP
#define SBEG461_LAB6_HPP

#include <sbeg/eigen.hpp>  // always first: includes Eigen/Dense with the course settings
#include <sbeg/mrcpp.hpp>

#include "course_arm.hpp"
#include "toppralite.hpp"

#include <string>
#include <utility>
#include <vector>

namespace lab {

constexpr double EPS_M = 1e-9;  ///< |m_i(s)| below this is treated as a zero-inertia point (MR Sec. 9.4.4)

using Path = toppralite::Path;              ///< path(s, 0) = theta, path(s, 1) = theta', path(s, 2) = theta''
using Trajectory = toppralite::Trajectory;  ///< traj(t, 0) = q, traj(t, 1) = qdot, traj(t, 2) = qddot
using Model = course_arm::Robot;            ///< name, n, Mlist, Glist, Slist, g (+ the course-arm fields)

// =============================================================================================
// Provided: paths and robot models
// =============================================================================================

/// theta(s) = q0 + s (q1 - q0), s in [0, 1] (MR Eq. 9.3).  (Python: StraightLinePath(q0, q1).)
/// order 0 -> theta, 1 -> theta' = q1 - q0, 2 -> theta'' = 0.
Path straight_line_path(const Eigen::VectorXd& q0, const Eigen::VectorXd& q1);

/// C2 cubic-spline path through the joint-space waypoints (K x n, one waypoint per ROW), s_k = k/(K-1), with
/// theta'' = 0 at the ends ("natural" spline, as scipy.interpolate.CubicSpline(bc_type="natural") used by toppra).
/// The robot still starts and ends at rest because sdot = 0 there.
Path waypoint_path(const Eigen::MatrixXd& waypoints);

/// MR Fig. 9.9: x = xc + R cos(pi s), y = yc - R sin(pi s), s in [0, 1].  (Python: SemicirclePath(R, xc, yc).)
Path semicircle_path(double R = 0.3, double xc = 0.0, double yc = 0.0);

/// The MR 2R arm (MR Fig. 8.1, Eqs. 8.9-8.10): point masses at the link ends, gravity g along -y
/// (g = 0: horizontal plane, as in MR Exercise 9.25).
Model mr_2r_model(double L1 = 1.0, double L2 = 1.0, double m1 = 1.0, double m2 = 1.0, double g = 0.0);

/// Workshop 6 Problem 2: elbow exoskeleton + forearm + 1 kg bottle, vertical plane, theta = 0 with the forearm
/// hanging down.  Lumped link: m = 3.0 kg, sum(m r) = 0.698 kg m, inertia about the elbow 0.19734 kg m^2.
Model elbow_exo_model();

/// Workshop 6 Problem 3: X-Y gantry (two prismatic joints, horizontal plane) of a humanitarian-demining sensor
/// head.  Joint 1 (x) moves Mx in total; joint 2 (y) moves my.  Illustrative values.
Model gantry_model(double Mx = 40.0, double my = 15.0);

/// The course arm (the local model file course_arm.hpp, numbers identical to the course robot models).
Model course_arm_model(double payload = 0.0);

/// modern_robotics RNEA (mr::inverse_dynamics) for a model; gravity = false sets g = 0.
Eigen::VectorXd inverse_dynamics(const Model& model, const Eigen::VectorXd& q, const Eigen::VectorXd& dq,
                                 const Eigen::VectorXd& ddq, bool gravity = true);

/// Task data for Part C (data/lab6_task.json).
struct Task {
    std::string description;
    Eigen::MatrixXd waypoints;  ///< K x n, one waypoint per row
    double payload_kg = 0, torque_fraction = 0, sim_speed_fraction = 0, hw_speed_fraction = 0,
           first_run_speed_fraction = 0, hw_qddot_max = 0, servo_period_s = 0, joint_margin_deg = 0;
    int grid_N = 0;
};
/// Task data for Part C (waypoints, limit fractions, payload, servo period), read from the lab's data folder.
Task load_task(const std::string& name = "lab6_task.json");

/// Path of a file in the lab's data folder (Python: DATA / name).
std::string data_path(const std::string& name);

// =============================================================================================
// Part A -- the path-constrained dynamics and the motion cone (MR Secs. 9.4, 9.4.1)
// =============================================================================================

/// (m, c, g) of MR Eq. 9.33, each of size n.
struct PathDynamics {
    Eigen::VectorXd m, c, g;
};

/// Return (m, c, g) of MR Eq. 9.33 at path parameter s.
/// Use three calls of the RNEA (no new dynamics code):
///   g(s) = ID(theta, 0, 0)            with gravity,
///   m(s) = ID(theta, 0, theta')       without gravity  (= M theta'),
///   c(s) = ID(theta, theta', theta'') without gravity  (= M theta'' + theta'^T Gamma theta').
PathDynamics path_dynamics(const Model& model, const Path& path, double s);

/// (L, U) of MR Eqs. 9.36-9.37 at one state: the smallest and largest sdd that satisfy every
/// tau_min_i <= m_i sdd + c_i sdot^2 + g_i <= tau_max_i.
/// If m_i < 0 the two bounds swap.  If |m_i| < EPS_M (zero-inertia point) joint i gives no bound on
/// sdd, but if c_i sdot^2 + g_i lies outside [tau_min_i, tau_max_i] the state is inadmissible: return
/// (L, U) = (+inf, -inf).  The state is admissible when L <= U.
std::pair<double, double> accel_limits(const Eigen::VectorXd& m, const Eigen::VectorXd& c, const Eigen::VectorXd& g,
                                       double sdot, const Eigen::VectorXd& tau_min, const Eigen::VectorXd& tau_max);

/// sdot_lim(s): the largest sdot with L(s, sdot) <= U(s, sdot) (the velocity limit curve, MR Sec. 9.4.1),
/// combined with the kinematic limit |theta'_i| sdot <= qdot_max_i if qdot_max is given (non-empty).
/// Bisection on sdot in [0, sdot_max] (assumes one admissible interval [0, sdot_lim], MR Sec. 9.4.4).
/// Returns sdot_max if no limit is found below it.
double velocity_limit(const Model& model, const Path& path, double s, const Eigen::VectorXd& tau_min,
                      const Eigen::VectorXd& tau_max, const Eigen::VectorXd& qdot_max = Eigen::VectorXd(),
                      double sdot_max = 100.0, double tol = 1e-9);

// =============================================================================================
// Part B -- bang-bang time scaling (MR Sec. 9.4.1, Fig. 9.12a) and TOPP with toppra (toppralite)
// =============================================================================================

/// (L, U) at (s, x = sdot^2): path_dynamics + accel_limits with sdot = sqrt(max(x, 0)).  (Given; Python _limits_x.)
std::pair<double, double> limits_x(const Model& model, const Path& path, double s, double x,
                                   const Eigen::VectorXd& tau_min, const Eigen::VectorXd& tau_max);

/// The result of bang_bang (the Python dict).  xA is NaN where curve A was not computed and +inf after the switch;
/// xF is +inf where curve F was not computed.
struct BangBang {
    Eigen::VectorXd s, sdot, xA, xF;
    double s_switch = 0, sdot_switch = 0, T = 0;
};

/// Single-switch time-optimal scaling (MR Sec. 9.4.1, Fig. 9.12a).
///
/// Work with x = sdot^2, for which dx/ds = 2 sdd (since d(sdot^2)/ds = 2 sdot dsdot/ds = 2 sdd).
/// On the grid s_k = k/N:
///   * curve A: integrate dx/ds = 2 U(s, sqrt x) forward from (0, 0)  (Heun / RK2);
///   * curve F: integrate dx/ds = 2 L(s, sqrt x) backward from (1, 0) until s = 0 or until it
///     penetrates the velocity limit curve (L > U), where F simply ends (MR Sec. 9.4.2, step 2);
///   * the switch s* is where A meets F (linear interpolation of A - F between grid points);
///   * the optimal profile is A before s* and F after it; the time is T = sum 2 ds / (sdot_k + sdot_k+1)
///     (constant sdd on each interval, exact for MR's piecewise-constant-acceleration model).
/// If A penetrates the velocity limit curve before it meets F, throw std::runtime_error: a multi-switch
/// solution (MR Sec. 9.4.2, steps 4-6) is needed -- use toppra.
BangBang bang_bang(const Model& model, const Path& path, const Eigen::VectorXd& tau_min,
                   const Eigen::VectorXd& tau_max, int N = 2000);

/// The result of topp (the Python dict): T, traj(t, order), s_grid and sdot_grid (the phase-plane curve).
struct ToppResult {
    double T = 0;
    Trajectory traj;
    Eigen::VectorXd s_grid, sdot_grid;
};

/// Time-optimal time scaling with toppra (TOPP-RA; C++: toppralite), the same problem as MR Sec. 9.4.
/// Constraints: joint torques tau_min <= tau <= tau_max (default, i.e. empty tau_min: -tau_max) via the model's
/// RNEA, and optionally |qdot| <= qdot_max, |qddot| <= qddot_max (non-empty vectors).  Start and end at rest.
/// Grid: N + 1 equally spaced points on [0, 1].  Throws std::runtime_error if no feasible time scaling exists.
ToppResult topp(const Model& model, const Path& path, const Eigen::VectorXd& tau_max,
                const Eigen::VectorXd& qdot_max = Eigen::VectorXd(), const Eigen::VectorXd& qddot_max = Eigen::VectorXd(),
                int N = 200, const Eigen::VectorXd& tau_min = Eigen::VectorXd());

/// Quintic time scaling s(t), sdot, sdd (MR Sec. 9.2.2.1, Fig. 9.4); t is clipped to [0, T].  (Given.)
struct QuinticS {
    double s, sd, sdd;
};
QuinticS quintic_s(double t, double T);

/// theta(s(t)) with a quintic s(t), as a trajectory traj(t, order) (order 0, 1, 2 via MR Eqs. 9.1-9.2).  (Given.)
Trajectory quintic_traj(const Path& path, double T);

/// Smallest T for which the quintic time scaling of `path` satisfies the same limits as `topp`
/// (checked at N+1 instants).  Bisection on T (feasibility is monotone in T when gravity can be held
/// statically: slowing down moves the torques toward g(theta)).  Throws std::runtime_error if no T <= 1e4 s works.
double quintic_min_time(const Model& model, const Path& path, const Eigen::VectorXd& tau_max,
                        const Eigen::VectorXd& qdot_max = Eigen::VectorXd(),
                        const Eigen::VectorXd& qddot_max = Eigen::VectorXd(), int N = 400, double tol = 1e-4,
                        const Eigen::VectorXd& tau_min = Eigen::VectorXd());

/// The result of peak_usage (the Python dict).
struct PeakUsage {
    Eigen::VectorXd vel_ratio, tau_ratio;
    double jerk_max = 0;
    Eigen::MatrixXd tau;  ///< (N+1) x n, row = sample
    Eigen::VectorXd t;
};

/// Provided.  Peak |qdot|/qdot_max and |tau|/tau_max per joint, and the peak |jerk| (finite differences of
/// qddot) along a trajectory traj(t, order) sampled at N+1 instants on [0, T].
PeakUsage peak_usage(const Model& model, const Trajectory& traj, double T, const Eigen::VectorXd& tau_max,
                     const Eigen::VectorXd& qdot_max, int N = 1000);

// =============================================================================================
// Part C -- hardware command file, safety check, and sim-vs-hardware metrics
// =============================================================================================

/// Sample a trajectory for the servo stream.  `slow` >= 1 stretches time uniformly
/// (q_cmd(t) = q(t / slow)), which divides all speeds by `slow` and accelerations by slow^2.
/// Returns (t, q): t = 0, dt, ..., slow*T (the last sample is exactly slow*T), q of size len(t) x n (row = sample).
std::pair<Eigen::VectorXd, Eigen::MatrixXd> command_table(const Trajectory& traj, double T, double dt = 0.02,
                                                          double slow = 1.0);

/// The result of check_command (the Python dict).
struct CommandCheck {
    bool ok = false, within_limits = false;
    double peak_speed_ratio = 0;
    bool speed_ok = false, starts_at_rest = false, ends_at_rest = false;
};

/// Safety check of a command table before it goes to the arm (course-arm hardware safety checklist,
/// rules 1 and 3; Course resources):
///   * within_limits: every q stays at least `margin_deg` (checklist: 10 deg) inside the joint limits;
///   * peak_speed_ratio: max over joints and samples of |dq/dt| / (speed_fraction * qdot_max)
///     (finite differences of the samples);  speed_ok = ratio <= 1 + tol;
///   * starts_at_rest / ends_at_rest: first and last steps are smaller than 1e-3 rad;
///   * ok = all of the above.
CommandCheck check_command(const Eigen::VectorXd& t, const Eigen::MatrixXd& q, const Model& model,
                           double speed_fraction, double margin_deg = 10.0, double tol = 0.01);

/// Provided.  Write a CSV in the project log format (project starter `hardware.py`):
/// t, cmd_1..cmd_n[, meas_1..meas_n][, cur_1..cur_n].  Empty meas / cur are left out.
void write_log(const std::string& path, const Eigen::VectorXd& t, const Eigen::MatrixXd& cmd,
               const Eigen::MatrixXd& meas = Eigen::MatrixXd(), const Eigen::MatrixXd& cur = Eigen::MatrixXd());

/// A log read by load_log: cmd, meas, cur are N x n (row = sample), empty if absent.
struct Log {
    Eigen::VectorXd t;
    Eigen::MatrixXd cmd, meas, cur;
};
/// Provided.  Read a log written by `write_log` (or by the station logger).
Log load_log(const std::string& path);

/// Provided (simulation fallback).  Model of a position-controlled smart servo: pure delay, then a
/// second-order loop  qdd = wn^2 (q_cmd(t - delay) - q) - 2 zeta wn qd  (semi-implicit Euler on a
/// 1 ms grid), then 12-bit encoder quantisation.  Illustrative values; identify wn, zeta and delay
/// from a step response on your unit (project brief Sec. 5.1).  Returns meas, N x n.
Eigen::MatrixXd simulate_servo(const Eigen::VectorXd& t, const Eigen::MatrixXd& cmd, double wn = 25.0,
                               double zeta = 0.9, double delay = 0.02,
                               double enc_res = 2.0 * 3.14159265358979323846 / 4096.0);

/// The result of tracking_metrics (the Python dict).
struct TrackingMetrics {
    Eigen::VectorXd rms, max_abs;  ///< per joint [rad]
    double lag = 0;                ///< [s]
    Eigen::VectorXd rms_after_lag; ///< per joint [rad]
};

/// Sim-vs-hardware metrics of one run (HWk-2 style):
///   * rms (n): RMS of meas - cmd per joint [rad];   * max_abs (n): max |meas - cmd| [rad];
///   * lag: the time shift d in [0, max_lag] (multiple of the sample time) that minimises the total
///     RMS of meas(t) - cmd(t - d), i.e. how far the measured motion runs behind the command [s];
///   * rms_after_lag (n): the RMS error after removing that lag.
/// Assumes a uniform time grid.  cmd and meas are N x n (row = sample).
TrackingMetrics tracking_metrics(const Eigen::VectorXd& t, const Eigen::MatrixXd& cmd, const Eigen::MatrixXd& meas,
                                 double max_lag = 0.2);

}  // namespace lab

#endif  // SBEG461_LAB6_HPP
