// SBEG 461 Lab 6 -- time-optimal time scaling (MR Sec. 9.4): phase plane, bang-bang, TOPP (toppra, C++:
// toppralite) versus quintic, and a hardware command file for the course arm.  C++ STARTER.
//
// Fill in every function marked  // TODO  (replace the `throw std::logic_error("TODO: ...")` line).
// Do not change function names or signatures (they are declared in lab.hpp, with their documentation).
// Build and test from the cpp folder:
//     cmake -S . -B build -G Ninja && cmake --build build && ./build/lab_tests
// or from your Lab code folder:   python run_lab.py 6 --cpp
//
// The provided parts (paths, models, inverse_dynamics, load_task, limits_x, quintic_s, quintic_traj, peak_usage,
// write_log, load_log, simulate_servo) are complete, as in starter.py.  As in Python, use the RNEA through
// inverse_dynamics (mr::inverse_dynamics) -- no new dynamics code -- and toppralite (toppralite.hpp) in topp.
// Eigen reminders: v.cwiseAbs(), v.maxCoeff(), A.row(k), A.col(j), A.topRows(k), A.bottomRows(k),
//   A.array().square().mean(), A.array().square().colwise().mean().sqrt(), std::numeric_limits<double>::infinity().
#include "lab.hpp"

#include <sbeg/labio.hpp>
#include <sbeg/mrcpp.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace lab {

namespace {
constexpr double PI = 3.14159265358979323846;
[[maybe_unused]] const double INF = std::numeric_limits<double>::infinity();  // for your code
[[maybe_unused]] const double NaN = std::numeric_limits<double>::quiet_NaN();

/// np.linspace(a, b, num): a + i * (b - a) / (num - 1), the last point exactly b.
Eigen::VectorXd linspace(double a, double b, Eigen::Index num) {
    Eigen::VectorXd y(num);
    const double step = num > 1 ? (b - a) / static_cast<double>(num - 1) : 0.0;
    for (Eigen::Index i = 0; i < num; ++i) y(i) = static_cast<double>(i) * step + a;
    if (num > 1) y(num - 1) = b;
    return y;
}

/// np.interp(x, xp, fp, left, right) for one value (xp increasing).
double interp(double x, const Eigen::VectorXd& xp, const Eigen::VectorXd& fp, double left, double right) {
    const Eigen::Index n = xp.size();
    if (x < xp(0)) return left;
    if (x > xp(n - 1)) return right;
    if (x == xp(n - 1)) return fp(n - 1);
    const Eigen::Index j =
        static_cast<Eigen::Index>(std::upper_bound(xp.data(), xp.data() + n, x) - xp.data()) - 1;
    const double slope = (fp(j + 1) - fp(j)) / (xp(j + 1) - xp(j));
    return slope * (x - xp(j)) + fp(j);
}

/// Natural cubic spline through (x_k, Y.row(k)) (scipy CubicSpline(bc_type="natural")): value and derivatives.
struct NaturalSpline {
    Eigen::VectorXd x;
    Eigen::MatrixXd Y, M2;  // M2: second derivatives at the knots (one row per knot)
    Eigen::VectorXd operator()(double s, int order) const {
        const Eigen::Index K = x.size();
        Eigen::Index k = static_cast<Eigen::Index>(std::upper_bound(x.data(), x.data() + K, s) - x.data()) - 1;
        k = std::min<Eigen::Index>(std::max<Eigen::Index>(k, 0), K - 2);
        const double h = x(k + 1) - x(k), a = x(k + 1) - s, b = s - x(k);
        const Eigen::VectorXd m0 = M2.row(k).transpose(), m1 = M2.row(k + 1).transpose();
        const Eigen::VectorXd y0 = Y.row(k).transpose(), y1 = Y.row(k + 1).transpose();
        if (order == 0)
            return m0 * (a * a * a / (6 * h)) + m1 * (b * b * b / (6 * h)) + (y0 / h - m0 * (h / 6)) * a +
                   (y1 / h - m1 * (h / 6)) * b;
        if (order == 1)
            return -m0 * (a * a / (2 * h)) + m1 * (b * b / (2 * h)) - (y0 / h - m0 * (h / 6)) +
                   (y1 / h - m1 * (h / 6));
        if (order == 2) return m0 * (a / h) + m1 * (b / h);
        throw std::invalid_argument("path order must be 0, 1 or 2");
    }
};
}  // namespace

// =============================================================================================
// Provided: paths and robot models
// =============================================================================================
Path straight_line_path(const Eigen::VectorXd& q0, const Eigen::VectorXd& q1) {
    if (q0.size() != q1.size()) throw std::invalid_argument("straight_line_path: q0 and q1 differ in size");
    const Eigen::VectorXd d = q1 - q0;
    return [q0, d](double s, int order) -> Eigen::VectorXd {
        if (order == 0) return q0 + s * d;
        if (order == 1) return d;
        return Eigen::VectorXd::Zero(d.size());
    };
}

Path waypoint_path(const Eigen::MatrixXd& waypoints) {
    const Eigen::Index K = waypoints.rows();
    if (K < 2) throw std::invalid_argument("waypoint_path: at least two waypoints are needed");
    NaturalSpline sp;
    sp.x = linspace(0.0, 1.0, K);
    sp.Y = waypoints;
    sp.M2 = Eigen::MatrixXd::Zero(K, waypoints.cols());
    if (K > 2) {  // tridiagonal system for the interior second derivatives (natural ends: M2 = 0)
        const Eigen::Index m = K - 2;
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(m, m), rhs(m, waypoints.cols());
        for (Eigen::Index i = 0; i < m; ++i) {
            const Eigen::Index k = i + 1;
            const double h0 = sp.x(k) - sp.x(k - 1), h1 = sp.x(k + 1) - sp.x(k);
            A(i, i) = 2.0 * (h0 + h1);
            if (i > 0) A(i, i - 1) = h0;
            if (i + 1 < m) A(i, i + 1) = h1;
            rhs.row(i) = 6.0 * ((waypoints.row(k + 1) - waypoints.row(k)) / h1 -
                                (waypoints.row(k) - waypoints.row(k - 1)) / h0);
        }
        sp.M2.middleRows(1, m) = A.partialPivLu().solve(rhs);
    }
    return sp;
}

Path semicircle_path(double R, double xc, double yc) {
    return [R, xc, yc](double s, int order) -> Eigen::VectorXd {
        const double a = PI * s;
        Eigen::VectorXd out(2);
        if (order == 0) out << xc + R * std::cos(a), yc - R * std::sin(a);
        else if (order == 1) out << -PI * R * std::sin(a), -PI * R * std::cos(a);
        else out << -PI * PI * R * std::cos(a), PI * PI * R * std::sin(a);
        return out;
    };
}

Model mr_2r_model(double L1, double L2, double m1, double m2, double g) {
    Model r;
    r.name = "MR 2R";
    r.n = 2;
    r.Mlist = {mr::rp_to_trans(Eigen::Matrix3d::Identity(), Eigen::Vector3d(L1, 0, 0)),
               mr::rp_to_trans(Eigen::Matrix3d::Identity(), Eigen::Vector3d(L2, 0, 0)), Eigen::Matrix4d::Identity()};
    mr::Vector6d d1, d2;
    d1 << 0, 0, 0, m1, m1, m1;
    d2 << 0, 0, 0, m2, m2, m2;
    r.Glist = {mr::Matrix6d(d1.asDiagonal()), mr::Matrix6d(d2.asDiagonal())};
    r.Slist.resize(6, 2);
    r.Slist << 0, 0, 0, 0, 1, 1, 0, 0, 0, -L1, 0, 0;  // columns (0,0,1,0,0,0) and (0,0,1,0,-L1,0)
    r.g = Eigen::Vector3d(0.0, -g, 0.0);
    return r;
}

Model elbow_exo_model() {
    const double m = 1.6 + 0.4 + 1.0;
    const double mr_sum = 1.6 * 0.18 + 0.4 * 0.15 + 1.0 * 0.35;
    const double I_elbow = 0.012 + 1.6 * 0.18 * 0.18 + 0.002 + 0.4 * 0.15 * 0.15 + 1.0 * 0.35 * 0.35;
    const double rc = mr_sum / m;
    const double Ic = I_elbow - m * rc * rc;
    Model r;
    r.name = "elbow exoskeleton";
    r.n = 1;
    r.Mlist = {mr::rp_to_trans(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, -rc, 0)),
               mr::rp_to_trans(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, -(0.35 - rc), 0))};
    mr::Vector6d d;
    d << Ic, Ic, Ic, m, m, m;
    r.Glist = {mr::Matrix6d(d.asDiagonal())};
    r.Slist.resize(6, 1);
    r.Slist << 0, 0, 1, 0, 0, 0;
    r.g = Eigen::Vector3d(0.0, -9.81, 0.0);
    return r;
}

Model gantry_model(double Mx, double my) {
    Model r;
    r.name = "gantry";
    r.n = 2;
    r.Mlist = {Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity()};
    mr::Vector6d d1, d2;
    d1 << 0, 0, 0, Mx - my, Mx - my, Mx - my;
    d2 << 0, 0, 0, my, my, my;
    r.Glist = {mr::Matrix6d(d1.asDiagonal()), mr::Matrix6d(d2.asDiagonal())};
    r.Slist = Eigen::MatrixXd::Zero(6, 2);
    r.Slist(3, 0) = 1.0;
    r.Slist(4, 1) = 1.0;
    r.g = Eigen::Vector3d::Zero();
    return r;
}

Model course_arm_model(double payload) { return course_arm::course_arm(0.0, payload); }

Eigen::VectorXd inverse_dynamics(const Model& model, const Eigen::VectorXd& q, const Eigen::VectorXd& dq,
                                 const Eigen::VectorXd& ddq, bool gravity) {
    const Eigen::Vector3d g = gravity ? model.g : Eigen::Vector3d::Zero();
    return mr::inverse_dynamics(q, dq, ddq, g, mr::Vector6d::Zero(), model.Mlist, model.Glist, model.Slist);
}

std::string data_path(const std::string& name) { return sbeg::lab_path("../data/" + name); }

Task load_task(const std::string& name) {
    const sbeg::json j = sbeg::load_json(data_path(name));
    Task t;
    t.description = j.value("description", std::string());
    t.waypoints = sbeg::as_mat(j.at("waypoints"));
    t.payload_kg = j.at("payload_kg").get<double>();
    t.torque_fraction = j.at("torque_fraction").get<double>();
    t.sim_speed_fraction = j.at("sim_speed_fraction").get<double>();
    t.hw_speed_fraction = j.at("hw_speed_fraction").get<double>();
    t.first_run_speed_fraction = j.at("first_run_speed_fraction").get<double>();
    t.hw_qddot_max = j.at("hw_qddot_max").get<double>();
    t.servo_period_s = j.at("servo_period_s").get<double>();
    t.joint_margin_deg = j.at("joint_margin_deg").get<double>();
    t.grid_N = j.at("grid_N").get<int>();
    return t;
}

// =============================================================================================
// Part A -- the path-constrained dynamics and the motion cone (MR Secs. 9.4, 9.4.1)
// =============================================================================================
PathDynamics path_dynamics(const Model& model, const Path& path, double s) {
    // TODO: three calls of inverse_dynamics(model, ...) as in the docstring (lab.hpp); return {m, c, g}:
    //       PathDynamics out; out.m = ...; out.c = ...; out.g = ...; return out;
    //       path(s, 0), path(s, 1), path(s, 2) give theta, theta', theta''; Eigen::VectorXd::Zero(n) is the zero vector.
    throw std::logic_error("TODO: path_dynamics");
}

std::pair<double, double> accel_limits(const Eigen::VectorXd& m, const Eigen::VectorXd& c, const Eigen::VectorXd& g,
                                       double sdot, const Eigen::VectorXd& tau_min, const Eigen::VectorXd& tau_max) {
    // TODO: loop over joints; MR Eq. 9.36 (swap the bounds if m_i < 0); handle |m_i| < EPS_M;
    //       L = max_i L_i, U = min_i U_i (MR Eq. 9.37).  Start from L = -INF, U = +INF; return {L, U};
    //       inadmissible zero-inertia state: return {INF, -INF};  (INF is defined at the top of this file)
    throw std::logic_error("TODO: accel_limits");
}

double velocity_limit(const Model& model, const Path& path, double s, const Eigen::VectorXd& tau_min,
                      const Eigen::VectorXd& tau_max, const Eigen::VectorXd& qdot_max, double sdot_max,
                      double tol) {
    // TODO: kinematic bound min_i qdot_max_i/|theta'_i| (only if qdot_max.size() > 0, skip |theta'_i| <= 1e-12),
    //       then bisection on sdot with accel_limits (see the docstring in lab.hpp)
    throw std::logic_error("TODO: velocity_limit");
}

// =============================================================================================
// Part B -- bang-bang time scaling (MR Sec. 9.4.1, Fig. 9.12a) and TOPP with toppra (toppralite)
// =============================================================================================
std::pair<double, double> limits_x(const Model& model, const Path& path, double s, double x,
                                   const Eigen::VectorXd& tau_min, const Eigen::VectorXd& tau_max) {
    const PathDynamics pd = path_dynamics(model, path, s);
    return accel_limits(pd.m, pd.c, pd.g, std::sqrt(std::max(x, 0.0)), tau_min, tau_max);
}

BangBang bang_bang(const Model& model, const Path& path, const Eigen::VectorXd& tau_min,
                   const Eigen::VectorXd& tau_max, int N) {
    // TODO: grid s_k = k/N (linspace(0.0, 1.0, N + 1) is defined at the top of this file);
    //       F: integrate dx/ds = 2L backward from x=0 at s=1 (stop where L > U: catch std::runtime_error only);
    //       A: integrate dx/ds = 2U forward from x=0 at s=0 until A >= F (throw std::runtime_error if L > U first);
    //       use limits_x; interpolate the crossing; T = sum 2h/(sd_k + sd_k+1)
    throw std::logic_error("TODO: bang_bang");
}

ToppResult topp(const Model& model, const Path& path, const Eigen::VectorXd& tau_max, const Eigen::VectorXd& qdot_max,
                const Eigen::VectorXd& qddot_max, int N, const Eigen::VectorXd& tau_min) {
    // TODO: build toppralite constraints (namespace ta = toppralite;):
    //         ta::joint_torque_constraint(inv_dyn, tau_lim, Eigen::VectorXd::Zero(n)) with a lambda
    //           inv_dyn = [&model](const Eigen::VectorXd& q, const Eigen::VectorXd& qd, const Eigen::VectorXd& qdd)
    //                     { return inverse_dynamics(model, q, qd, qdd); };
    //         ta::joint_velocity_constraint(vlim), ta::joint_acceleration_constraint(alim) if given;
    //         (limits are n x 2 matrices [lower, upper]:  Eigen::MatrixXd vlim(n, 2); vlim << -qdot_max, qdot_max;)
    //       ta::TOPPRA inst(cons, path, grid);  auto traj = inst.compute_trajectory(0.0, 0.0);
    //       nullptr -> throw std::runtime_error;  read traj->duration() and inst.problem_data().sd_vec
    throw std::logic_error("TODO: topp");
}

QuinticS quintic_s(double t, double T) {
    const double tau = std::min(std::max(t / T, 0.0), 1.0);
    QuinticS q;
    q.s = 10 * std::pow(tau, 3) - 15 * std::pow(tau, 4) + 6 * std::pow(tau, 5);
    q.sd = (30 * std::pow(tau, 2) - 60 * std::pow(tau, 3) + 30 * std::pow(tau, 4)) / T;
    q.sdd = (60 * tau - 180 * std::pow(tau, 2) + 120 * std::pow(tau, 3)) / (T * T);
    return q;
}

Trajectory quintic_traj(const Path& path, double T) {
    return [path, T](double t, int order) -> Eigen::VectorXd {
        const QuinticS q = quintic_s(t, T);
        if (order == 0) return path(q.s, 0);
        const Eigen::VectorXd d1 = path(q.s, 1);
        if (order == 1) return d1 * q.sd;
        return d1 * q.sdd + path(q.s, 2) * (q.sd * q.sd);
    };
}

double quintic_min_time(const Model& model, const Path& path, const Eigen::VectorXd& tau_max,
                        const Eigen::VectorXd& qdot_max, const Eigen::VectorXd& qddot_max, int N, double tol,
                        const Eigen::VectorXd& tau_min) {
    // TODO: precompute m(s), c(s), g(s) at the quintic samples; feasible(T) checks torque, speed and
    //       acceleration limits; bracket and bisect on T
    throw std::logic_error("TODO: quintic_min_time");
}

PeakUsage peak_usage(const Model& model, const Trajectory& traj, double T, const Eigen::VectorXd& tau_max,
                     const Eigen::VectorXd& qdot_max, int N) {
    PeakUsage out;
    out.t = linspace(0.0, T, N + 1);
    const Eigen::Index K = out.t.size();
    Eigen::MatrixXd qd, qdd;
    for (Eigen::Index k = 0; k < K; ++k) {
        const Eigen::VectorXd q = traj(out.t(k), 0), v = traj(out.t(k), 1), a = traj(out.t(k), 2);
        if (k == 0) {
            qd.resize(K, q.size());
            qdd.resize(K, q.size());
            out.tau.resize(K, q.size());
        }
        qd.row(k) = v.transpose();
        qdd.row(k) = a.transpose();
        out.tau.row(k) = inverse_dynamics(model, q, v, a).transpose();
    }
    out.vel_ratio = qd.cwiseAbs().colwise().maxCoeff().transpose().cwiseQuotient(qdot_max);
    out.tau_ratio = out.tau.cwiseAbs().colwise().maxCoeff().transpose().cwiseQuotient(tau_max);
    double jm = 0.0;
    for (Eigen::Index k = 0; k + 1 < K; ++k)
        jm = std::max(jm, ((qdd.row(k + 1) - qdd.row(k)) / (out.t(k + 1) - out.t(k))).cwiseAbs().maxCoeff());
    out.jerk_max = jm;
    return out;
}

// =============================================================================================
// Part C -- hardware command file, safety check, and sim-vs-hardware metrics
// =============================================================================================
std::pair<Eigen::VectorXd, Eigen::MatrixXd> command_table(const Trajectory& traj, double T, double dt, double slow) {
    // TODO: t = 0, dt, ..., slow*T (last sample exactly slow*T); q.row(k) = traj(t(k) / slow, 0).transpose()
    throw std::logic_error("TODO: command_table");
}

CommandCheck check_command(const Eigen::VectorXd& t, const Eigen::MatrixXd& q, const Model& model,
                           double speed_fraction, double margin_deg, double tol) {
    // TODO: joint-limit margin (model.limits, n x 2), finite-difference speeds against
    //       speed_fraction * model.qdot_max, rest at both ends
    throw std::logic_error("TODO: check_command");
}

void write_log(const std::string& path, const Eigen::VectorXd& t, const Eigen::MatrixXd& cmd,
               const Eigen::MatrixXd& meas, const Eigen::MatrixXd& cur) {
    const Eigen::Index N = t.size(), n = cmd.cols();
    std::vector<std::string> header = {"t"};
    Eigen::Index cols = 1 + n + (meas.size() ? meas.cols() : 0) + (cur.size() ? cur.cols() : 0);
    Eigen::MatrixXd data(N, cols);
    data.col(0) = t;
    data.middleCols(1, n) = cmd;
    for (Eigen::Index j = 0; j < n; ++j) header.push_back("cmd_" + std::to_string(j + 1));
    Eigen::Index c = 1 + n;
    if (meas.size()) {
        data.middleCols(c, meas.cols()) = meas;
        for (Eigen::Index j = 0; j < meas.cols(); ++j) header.push_back("meas_" + std::to_string(j + 1));
        c += meas.cols();
    }
    if (cur.size()) {
        data.middleCols(c, cur.cols()) = cur;
        for (Eigen::Index j = 0; j < cur.cols(); ++j) header.push_back("cur_" + std::to_string(j + 1));
    }
    sbeg::write_csv(path, header, data);
}

Log load_log(const std::string& path) {
    const sbeg::CsvTable tab = sbeg::read_csv(path);
    auto group = [&](const std::string& prefix) {
        std::vector<std::pair<int, Eigen::Index>> cols;  // (number, column)
        for (std::size_t i = 0; i < tab.header.size(); ++i) {
            const std::string& h = tab.header[i];
            if (h.rfind(prefix + "_", 0) == 0) cols.emplace_back(std::stoi(h.substr(prefix.size() + 1)),
                                                                 static_cast<Eigen::Index>(i));
        }
        std::sort(cols.begin(), cols.end());
        Eigen::MatrixXd out(tab.data.rows(), static_cast<Eigen::Index>(cols.size()));
        for (std::size_t j = 0; j < cols.size(); ++j) out.col(static_cast<Eigen::Index>(j)) = tab.data.col(cols[j].second);
        return cols.empty() ? Eigen::MatrixXd() : out;
    };
    Log log;
    log.t = tab.data.col(tab.col("t"));
    log.cmd = group("cmd");
    log.meas = group("meas");
    log.cur = group("cur");
    return log;
}

Eigen::MatrixXd simulate_servo(const Eigen::VectorXd& t, const Eigen::MatrixXd& cmd, double wn, double zeta,
                               double delay, double enc_res) {
    const double h = 1e-3;
    const Eigen::Index N = t.size(), n = cmd.cols();
    // tf = np.arange(t[0], t[-1] + h/2, h)
    const auto nf = static_cast<Eigen::Index>(std::ceil((t(N - 1) + h / 2 - t(0)) / h));
    Eigen::VectorXd tf(nf);
    for (Eigen::Index k = 0; k < nf; ++k) tf(k) = t(0) + static_cast<double>(k) * h;
    Eigen::MatrixXd cf(nf, n), out(nf, n), meas(N, n);
    for (Eigen::Index j = 0; j < n; ++j) {
        const Eigen::VectorXd cj = cmd.col(j);
        for (Eigen::Index k = 0; k < nf; ++k) cf(k, j) = interp(tf(k) - delay, t, cj, cj(0), cj(N - 1));
    }
    Eigen::RowVectorXd q = cmd.row(0), qd = Eigen::RowVectorXd::Zero(n);
    for (Eigen::Index k = 0; k < nf; ++k) {
        out.row(k) = q;
        const Eigen::RowVectorXd qdd = wn * wn * (cf.row(k) - q) - 2 * zeta * wn * qd;
        qd = qd + h * qdd;
        q = q + h * qd;
    }
    for (Eigen::Index j = 0; j < n; ++j) {
        const Eigen::VectorXd oj = out.col(j);
        for (Eigen::Index k = 0; k < N; ++k)
            meas(k, j) = std::nearbyint(interp(t(k), tf, oj, oj(0), oj(nf - 1)) / enc_res) * enc_res;
    }
    return meas;
}

TrackingMetrics tracking_metrics(const Eigen::VectorXd& t, const Eigen::MatrixXd& cmd, const Eigen::MatrixXd& meas,
                                 double max_lag) {
    // TODO: errors, RMS and max per joint; search the shift k*dt in [0, max_lag] minimising the RMS
    //       (dt = median of the time steps; meas.bottomRows(N - k) - cmd.topRows(N - k) is the shifted error)
    throw std::logic_error("TODO: tracking_metrics");
}

}  // namespace lab
