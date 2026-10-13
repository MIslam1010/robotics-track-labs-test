// SBEG 461 Hardware rotation 1 (Week 8) -- trajectory execution and logging on the course arm, servo
// identification and the simulation-versus-hardware comparison (C++ track): declarations.
//
// The same functions as starter.py (module hwrun.py of your sbeg461_dynamics library), in namespace `lab`, with
// Eigen types instead of NumPy arrays.  Write your code in starter.cpp.  Do not change these declarations:
// test_lab.cpp calls them.
//
// What runs on the arm.  The course arm has position-controlled smart servos (project brief Sec. 5.1).  A command
// file of joint angles is streamed every 20 ms; the servo's own loop tracks it.  We model one joint of that loop as
// a pure delay followed by a second-order system (Astrom-Murray Eq. 6.22)
//     q_dd = wn^2 (q_cmd(t - delay) - q) - 2 zeta wn q_d ,
// plus a static sag  q_meas = q - g(q) / K  caused by the finite stiffness K (N m/rad) of the position loop under
// the gravity torque g(q) (MR Eq. 8.1 with theta_dot = theta_dd = 0), plus 12-bit encoder quantisation.
// All device numbers are ILLUSTRATIVE VALUES; identify them on your own unit.
//
// Notation: Modern Robotics (MR).  Units: SI (s, rad, rad/s, N m, A) unless a name ends in _deg.
// Time series and logs are matrices with ONE ROW PER SAMPLE (N x n), as the NumPy arrays; t is a VectorXd (N).
// Python -> C++: a dict result -> a small struct with the same field names; joints=(1, 2, 3) (0-based) ->
// std::vector<int>{1, 2, 3}; np.inf -> std::numeric_limits<double>::infinity(); the session record (a JSON
// dict in Python) -> sbeg::json (nlohmann::json).
#ifndef SBEG461_HW1_HPP
#define SBEG461_HW1_HPP

#include <sbeg/eigen.hpp>  // always first: includes Eigen/Dense with the course settings

#include <sbeg/labio.hpp>  // sbeg::json (task file, session record)

#include <functional>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace lab {

// ===========================================================================================================
// Constants (as in starter.py)
// ===========================================================================================================
constexpr double PI = 3.14159265358979323846;
constexpr double DEG = PI / 180.0;
constexpr double DT = 0.02;                      ///< servo stream / logger period (s), 50 Hz
constexpr double ENC_RES = 2.0 * PI / 4096.0;    ///< 12-bit encoder (rad)
constexpr double QUINTIC_V = 15.0 / 8.0;         ///< peak of ds/dr for the quintic 10r^3 - 15r^4 + 6r^5
/// Peak of |d2s/dr2| for the quintic: 10 / sqrt(3).
constexpr double QUINTIC_A = 5.773502691896258;

/// Servo model parameters (wn rad/s, zeta, delay s).
struct ServoParams {
    double wn;
    double zeta;
    double delay;
};
/// The Lab 6 servo model (not identified).
constexpr ServoParams NOMINAL_SERVO{25.0, 0.9, 0.02};

/// Software joint limits >= 10 deg inside the hardware limits (shared checklist, rule A1).
constexpr double SOFT_LIMIT_MARGIN_DEG = 10.0;
/// 0.125 kg = 50 % of the rated 0.25 kg (brief Sec. 5.2, rule A4).
constexpr double PAYLOAD_MAX_KG = 0.5 * 0.25;

/// Labels = the rule labels of project brief Sec. 5.2 (A1-A7 course arm, B1-B6 session rules).
inline const std::vector<std::string> PREFLIGHT_RULES = {
    "A5: every member who touches the arm has a signed induction",
    "A7: a TA or the instructor is named as supervisor",
    "A7: the e-stop was tested at the start of the session",
    "B4: a named member holds the e-stop",
    "A2/A6: the keep-out zone is taped and clear",
    "B5: the workspace is ready (mat, payload fixed, cables tidy)",
    "A1/A3: the command passed check_command at the required speed fraction with >= 10 deg margin",
    "B2: the dry run predicts speed and torque ratios <= 1 and is written on the log",
    "A (current): the servo current limit is <= 60 % of rated",
    "A4: the payload is <= 0.125 kg",
};

// ===========================================================================================================
// Provided (complete, in course_arm.cpp and helpers.cpp): model, task, logs, servo simulator
// ===========================================================================================================

/// The course arm (the lab's own copy of the shared course-arm model, course_arm.cpp = course_arm.py).
/// Python dict keys -> fields.  Slist 6 x 6 (axes are the COLUMNS), limits 6 x 2 (rad, [min, max] per joint).
struct Model {
    std::string name;
    int n = 6;
    Eigen::MatrixXd Slist;
    Eigen::Matrix4d M;
    Eigen::MatrixXd limits;
    Eigen::VectorXd qdot_max;   ///< rated speed (rad/s)
    Eigen::VectorXd tau_max;    ///< rated torque (N m)
    Eigen::VectorXd q_home;
    std::vector<Eigen::Matrix4d> Mlist;
    std::vector<sbeg::Matrix6d> Glist;
    Eigen::Vector3d g;
    double payload_max = 0.0;
    double derated_speed_fraction = 0.0;
    double derated_current_fraction = 0.0;
};

/// THE course arm.  `payload` (kg) adds a point mass at the tool point (same rule as the shared model).
Model course_arm_model(double payload = 0.0);

/// modern_robotics RNEA (MR Eqs. 8.50-8.54) for a model; gravity=false sets g = 0.
Eigen::VectorXd inverse_dynamics(const Model& model, const Eigen::VectorXd& q, const Eigen::VectorXd& dq,
                                 const Eigen::VectorXd& ddq, bool gravity = true);

/// g(theta): RNEA with zero velocity and acceleration (MR Eq. 8.1), one configuration q (n).
Eigen::VectorXd gravity_torques(const Model& model, const Eigen::VectorXd& q);

/// g(theta) for every row of Q (N x n) -> N x n (Python: gravity_torques(model, q) with q of shape (N, n)).
Eigen::MatrixXd gravity_torques_rows(const Model& model, const Eigen::MatrixXd& Q);

/// Task data: waypoints, limit fractions, payload, step test, static poses, stop test (data/<name>).
sbeg::json load_task(const std::string& name = "rotation1_task.json");

/// Path of a file in the lab's data folder (Python: DATA / name).
std::string data_path(const std::string& name);

/// Write a CSV in the project log format (project starter `hardware.py`):
/// t, cmd_1..cmd_n[, meas_1..meas_n][, cur_1..cur_n]  (s, rad, A).  An empty meas / cur (0 x 0) is left out.
void write_log(const std::string& path, const Eigen::VectorXd& t, const Eigen::MatrixXd& cmd,
               const Eigen::MatrixXd& meas = Eigen::MatrixXd(), const Eigen::MatrixXd& cur = Eigen::MatrixXd());

/// A log read by `load_log`: t (N), cmd, meas, cur (N x n; 0 x 0 when absent) and any extra columns without
/// "_" in the name (e.g. "pose", "dir") in `extra`.
struct Log {
    Eigen::VectorXd t;
    Eigen::MatrixXd cmd, meas, cur;
    std::map<std::string, Eigen::VectorXd> extra;
};

/// Read a log written by `write_log` or by the station logger.
Log load_log(const std::string& path);

/// Simulated servo response to a streamed command cmd (N x n) on the time grid t (the simulation fallback and the
/// "sim" of the sim-vs-hardware comparison).
///  * The command is a zero-order hold: each set-point is held until the next one arrives.
///  * Pure delay `delay`, then  q_dd = wn^2 (q_cmd - q) - 2 zeta wn q_d  (semi-implicit Euler, 1 ms).
///  * If `stiffness` K (n) is not empty and `model` is given, the static sag g(q)/K is subtracted
///    (K in N m/rad; `model` provides g(q) through the RNEA).
///  * Encoder quantisation `enc_res` (rad); use enc_res = 0 for an ideal sensor.
/// Returns meas (N x n).  (Python also accepts per-joint wn, zeta, delay; the C++ version takes scalars.)
Eigen::MatrixXd simulate_servo(const Eigen::VectorXd& t, const Eigen::MatrixXd& cmd, double wn = 25.0,
                               double zeta = 0.9, double delay = 0.02,
                               const Eigen::VectorXd& stiffness = Eigen::VectorXd(), const Model* model = nullptr,
                               double enc_res = ENC_RES);

/// Result of `check_command`.
struct CommandCheck {
    bool ok = false;
    bool within_limits = false;
    double peak_speed_ratio = 0.0;
    bool speed_ok = false;
    bool starts_at_rest = false;
    bool ends_at_rest = false;
};

/// Lab 6 safety check of a command table (project brief Sec. 5.2, rules A1 and A3): joint limits with a margin,
/// peak finite-difference speed against speed_fraction * rated speed, rest at both ends.
CommandCheck check_command(const Eigen::VectorXd& t, const Eigen::MatrixXd& q, const Model& model,
                           double speed_fraction, double margin_deg = SOFT_LIMIT_MARGIN_DEG, double tol = 0.01);

/// Result of `tracking_metrics` (per joint, rad; lag in s).
struct TrackingMetrics {
    Eigen::VectorXd rms, max_abs;
    double lag = 0.0;
    Eigen::VectorXd rms_after_lag;
};

/// Lab 6 metrics: RMS and max |meas - cmd| per joint, the lag d (multiple of the sample time) that minimises the
/// RMS of meas(t) - cmd(t - d), and the RMS after removing it.
TrackingMetrics tracking_metrics(const Eigen::VectorXd& t, const Eigen::MatrixXd& cmd, const Eigen::MatrixXd& meas,
                                 double max_lag = 0.2);

/// Averaged static poses (P poses): pose ids and the averaged cmd, meas, cur (P x n).
struct StaticPoses {
    std::vector<int> pose;
    Eigen::MatrixXd cmd, meas, cur;
};

/// Static-pose log (columns pose, dir, cmd_j, meas_j, cur_j; each pose approached from below, dir = +1, and from
/// above, dir = -1): average the two approaches of every pose, so that static friction cancels to first order.
StaticPoses average_approaches(const Log& log);

/// np.median(v): the middle value of v (mean of the two middle values for an even length).
double median(const Eigen::VectorXd& v);

/// Unit step response of q_dd + 2 zeta wn q_d + wn^2 q = wn^2 u (Astrom-Murray Eq. 6.22, k = 1) at times
/// tau >= 0 after the start of motion (0 for tau < 0).
Eigen::VectorXd step_response(const Eigen::VectorXd& tau, double wn, double zeta);

/// Result of `least_squares`: the solution x and cost = 0.5 * |residual(x)|^2.
struct LeastSquaresResult {
    Eigen::VectorXd x;
    double cost = 0.0;
};

/// Bounded nonlinear least squares, the C++ stand-in for scipy.optimize.least_squares(fun, x0, bounds=(lb, ub),
/// x_scale=x_scale): minimises 0.5 |fun(x)|^2 with lb <= x <= ub (Levenberg-Marquardt, forward-difference
/// Jacobian, steps projected onto the bounds; at most max_iter iterations).
LeastSquaresResult least_squares(const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& fun,
                                 const Eigen::VectorXd& x0, const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
                                 const Eigen::VectorXd& x_scale, int max_iter = 200);

// ===========================================================================================================
// Part A -- pre-flight: units, the command file and the dry run
// ===========================================================================================================

/// Servo reading (degrees, servo zero) -> model angle (rad, PoE zero of the course robot models):
/// q_model = q_servo - offset, converted to radians.
Eigen::VectorXd servo_to_model(const Eigen::VectorXd& q_servo_deg, const Eigen::VectorXd& offset_deg);

/// Inverse of `servo_to_model`: model angle (rad) -> servo command (degrees).
Eigen::VectorXd model_to_servo(const Eigen::VectorXd& q_model, const Eigen::VectorXd& offset_deg);

/// Shortest duration T of a joint-synchronised rest-to-rest QUINTIC segment (MR Sec. 9.2.2.1, p. 332) that moves
/// the joints by `delta` (n) with |qdot_j| <= v_max_j and |qddot_j| <= a_max_j (v_max, a_max: n entries; Python
/// also accepts a scalar -- in C++ pass Eigen::VectorXd::Constant(n, a)).
/// For theta = theta_start + s(t/T) delta with the quintic s: peak speed (15/8)|delta_j|/T and peak acceleration
/// (10/sqrt 3)|delta_j|/T^2.  T is the largest of the 2n resulting bounds, rounded UP to a whole number of servo
/// periods dt (so that the segment ends on a sample).
double segment_time(const Eigen::VectorXd& delta, const Eigen::VectorXd& v_max, const Eigen::VectorXd& a_max,
                    double dt = DT);

/// Result of `waypoint_command`: t (N), q, qd, qdd (N x n), seg_T (K-1) segment durations, t_start (K-1) start
/// time of each segment.
struct WaypointCommand {
    Eigen::VectorXd t;
    Eigen::MatrixXd q, qd, qdd;
    Eigen::VectorXd seg_T, t_start;
};

/// Command table through `waypoints` (K x n, one waypoint per ROW): a rest-to-rest quintic segment between
/// consecutive waypoints (duration from `segment_time`), a dwell of `dwell` s at the start, at every intermediate
/// waypoint and at the end.  Sampled every dt from t = 0.  The last sample is the end of the final dwell.
WaypointCommand waypoint_command(const Eigen::MatrixXd& waypoints, const Eigen::VectorXd& v_max,
                                 const Eigen::VectorXd& a_max, double dt = DT, double dwell = 0.5);

/// Result of `dry_run`.
struct DryRun {
    Eigen::MatrixXd tau;          ///< N x n RNEA torques along the command
    Eigen::VectorXd peak_tau;     ///< max |tau_j|
    Eigen::VectorXd torque_ratio; ///< peak_tau / (torque_fraction * tau_max)
    Eigen::VectorXd speed_ratio;  ///< max |qd_j| / (speed_fraction * qdot_max)
    bool ok = false;              ///< every ratio <= 1
};

/// Dry-run prediction written on the session log before the arm is powered (brief Sec. 5.2, rule B2).
/// `cmd` is the result of `waypoint_command`.  Uses the ANALYTIC qd, qdd of the command.
///   * tau (N x n): RNEA torques along the command (MR Eqs. 8.50-8.54);
///   * peak_tau (n): max |tau_j|;  torque_ratio (n): peak_tau / (torque_fraction * tau_max);
///   * speed_ratio (n): max |qd_j| / (speed_fraction * qdot_max);
///   * ok: every ratio <= 1.
DryRun dry_run(const Model& model, const WaypointCommand& cmd, double speed_fraction = 0.25,
               double torque_fraction = 0.6);

// ===========================================================================================================
// Part B -- step test and servo identification
// ===========================================================================================================

/// Hand method (Astrom-Murray Table 6.1): from the fractional overshoot Mp = exp(-pi zeta/sqrt(1-zeta^2)) and the
/// peak time t_peak = pi/(wn sqrt(1-zeta^2)) (measured from the START OF MOTION, i.e. after the delay) return
/// (zeta, wn).  Requires 0 < overshoot < 1.
std::pair<double, double> zeta_wn_from_step(double overshoot, double t_peak);

/// Result of `step_features`.
struct StepFeatures {
    double t_step = 0.0, A = 0.0, y0 = 0.0, y_ss = 0.0, t_move = 0.0, overshoot = 0.0, t_peak_abs = 0.0;
};

/// Features of ONE step in one joint signal (t, cmd, meas: N entries each).
///   * t_step: first sample time where cmd differs from cmd[0];  A: cmd[-1] - cmd[0];
///   * y0, y_ss: mean of meas before t_step, and mean over the last 0.3 s;
///   * t_move: first time (linear interpolation between samples) at which (meas - y0)/(y_ss - y0) reaches
///     `threshold` ... used only as a rough delay estimate;
///   * overshoot: (max(meas) - y_ss)/(y_ss - y0) for a positive step (min for a negative one);
///   * t_peak_abs: time of that extreme sample.
StepFeatures step_features(const Eigen::VectorXd& t, const Eigen::VectorXd& cmd, const Eigen::VectorXd& meas,
                           double threshold = 0.1);

/// Result of `identify_servo`.
struct ServoFit {
    double wn = 0.0, zeta = 0.0, delay = 0.0, y0 = 0.0, A_fit = 0.0, rms = 0.0;
};

/// Least-squares fit of (wn, zeta, delay) of the servo model to ONE step (t, cmd, meas: N entries each) on
/// [t_step - 0.1, t_step + window]:  meas(t) ~ y0 + A_fit * step_response(t - t_step - delay).
/// y0 and A_fit (which absorbs a change of sag) are solved linearly for every trial (wn, zeta, delay).
/// Start from the hand method (`step_features` + `zeta_wn_from_step`) and refine with the given `least_squares`
/// (bounds wn in [2, 200], zeta in [0.05, 3], delay in [0, 0.2]).
/// Returns ServoFit{wn, zeta, delay, y0, A_fit, rms} with rms the fit residual (rad).
ServoFit identify_servo(const Eigen::VectorXd& t, const Eigen::VectorXd& cmd, const Eigen::VectorXd& meas,
                        double window = 1.0);

// ===========================================================================================================
// Part C -- static poses: gravity check, torque constant, loop stiffness
// ===========================================================================================================

/// Least-squares k (N m/A, gear ratio included) with tau_model ~ k * current (one joint, or all entries pooled
/// into one vector).  Same definition as project `hardware.torque_constant`.
double torque_constant(const Eigen::VectorXd& current, const Eigen::VectorXd& tau_model);

/// RMS(predicted - measured) / RMS(measured) (project `hardware.normalised_rms`; HWk-2 gravity check passes
/// at <= 0.3).
double normalised_rms(const Eigen::VectorXd& predicted, const Eigen::VectorXd& measured);

/// Least-squares stiffness K (N m/rad) of one joint's position loop from static poses:
/// sag = cmd - meas ~ tau_model / K, i.e. 1/K = sum(tau*sag)/sum(tau^2).
double loop_stiffness(const Eigen::VectorXd& sag, const Eigen::VectorXd& tau_model);

/// Result of `gravity_check`: k, nrms, K over `joints`; g (P x n); tau_cur (P x len(joints)).
struct GravityCheck {
    Eigen::VectorXd k, nrms, K;
    Eigen::MatrixXd g, tau_cur;
};

/// Gravity check of HW2-2(b)/HW3-2 type on the averaged static poses (`average_approaches`).
/// For each joint index j in `joints` (0-based; default joints 2, 3 and 4, the gravity-loaded ones):
///   * the model gravity torque at the MEASURED pose, g_j(q_meas);
///   * k_j = torque_constant(cur_j[fit], g_j[fit]);   tau_cur_j = k_j * cur_j;
///   * nrms_j = normalised_rms(g_j[check], tau_cur_j[check]) on the check poses only;
///   * K_j = loop_stiffness(cmd_j - meas_j, g_j) on all poses.
GravityCheck gravity_check(const StaticPoses& st, const Model& model, const std::vector<int>& joints = {1, 2, 3},
                           const std::vector<int>& fit = {0, 2, 4}, const std::vector<int>& check = {1, 3, 5});

// ===========================================================================================================
// Part D -- simulation versus hardware, safe stop, pre-flight record
// ===========================================================================================================

/// Result of `compare_runs` (per joint, rad; lag in s).
struct RunComparison {
    Eigen::VectorXd rms_cmd_meas, max_cmd_meas, rms_sim_meas, max_sim_meas;
    double lag = 0.0;
};

/// Sim-vs-hardware metrics after time `skip` (project brief Sec. 5.4), all per joint (rad): rms_cmd_meas,
/// max_cmd_meas (tracking error of the device), rms_sim_meas, max_sim_meas (model error: simulation of the SAME
/// command against the measurement), and lag (s) of meas behind cmd from `tracking_metrics`.
/// cmd, meas, sim are N x n on the same time grid t.
RunComparison compare_runs(const Eigen::VectorXd& t, const Eigen::MatrixXd& cmd, const Eigen::MatrixXd& meas,
                           const Eigen::MatrixXd& sim, double skip = 0.0);

/// Safe-stop time (HWk-4): time from `t_trigger` until the joint speed, estimated by a central difference over
/// `window` s (q(t + w/2) - q(t - w/2))/w to suppress encoder steps, stays below `threshold` (rad/s, all joints)
/// for the rest of the log.  Returns infinity if it never settles.  q is N x n.
/// The speed estimate is assigned to the centre sample; samples within w/2 of the ends are ignored.
double stop_time(const Eigen::VectorXd& t, const Eigen::MatrixXd& q, double t_trigger,
                 double threshold = 2.0 * DEG, double window = 0.1);

/// Checks a session record (see data/session_record_template.json; read it with sbeg::load_json) against the
/// hardware safety checklist of project brief Sec. 5.2 and returns the list of FAILED rules (strings of
/// PREFLIGHT_RULES, in that order); an empty list means the session may start.
///
/// Record keys: members_at_arm (list of names), inducted (list of names), supervisor (str), estop_tested (bool),
/// estop_holder (str, must be one of members_at_arm), keepout_clear (bool), workspace_ready (bool), first_run
/// (bool), check (object from check_command, with keys ok and peak_speed_ratio), check_speed_fraction (number),
/// margin_deg (number), dry_run_speed_ratio (number), dry_run_torque_ratio (number), current_limit_fraction
/// (number), payload_kg (number, weighed payload at the tool point, at most PAYLOAD_MAX_KG).
/// The required speed fraction is 0.25 for a first run of a motion and 0.5 otherwise; a check made at a HIGHER
/// fraction than required does not count.  A missing key (or null) counts as a failure.
std::vector<std::string> preflight(const sbeg::json& record);

}  // namespace lab

#endif  // SBEG461_HW1_HPP
