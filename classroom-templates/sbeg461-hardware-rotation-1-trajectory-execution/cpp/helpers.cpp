// SBEG 461 Hardware rotation 1 (Week 8) -- provided helpers (C++ track).  GIVEN code: do not edit; use these
// functions freely.
//
// The "Provided" part of starter.py, translated to C++:
//   inverse_dynamics, gravity_torques, gravity_torques_rows   RNEA through mrcpp (= modern_robotics)
//   load_task, data_path, write_log, load_log                 task file and station logs
//   simulate_servo                                            servo model: zero-order hold, delay, 2nd order, sag, encoder
//   check_command, tracking_metrics                           Lab 6, unchanged
//   average_approaches, step_response                         static poses; unit step response (Astrom-Murray Eq. 6.22)
//   median                                                    np.median
// and one replacement for SciPy:
//   least_squares                                             bounded Levenberg-Marquardt (scipy.optimize.least_squares)
#include "lab.hpp"

#include <sbeg/labio.hpp>
#include <sbeg/mrcpp.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace lab {
namespace {

using Eigen::Index;
using Eigen::MatrixXd;
using Eigen::VectorXd;

double median_step(const VectorXd& t) {
    if (t.size() < 2) throw std::invalid_argument("at least two samples are needed");
    return median((t.tail(t.size() - 1) - t.head(t.size() - 1)).eval());
}

/// np.interp(x, xp, fp): linear interpolation, constant beyond the ends (xp increasing).
VectorXd interp(const VectorXd& x, const VectorXd& xp, const VectorXd& fp) {
    VectorXd y(x.size());
    const Index n = xp.size();
    for (Index i = 0; i < x.size(); ++i) {
        const double xi = x(i);
        if (xi <= xp(0)) {
            y(i) = fp(0);
        } else if (xi >= xp(n - 1)) {
            y(i) = fp(n - 1);
        } else {
            const Index k = static_cast<Index>(std::upper_bound(xp.data(), xp.data() + n, xi) - xp.data()) - 1;
            const double s = (xi - xp(k)) / (xp(k + 1) - xp(k));
            y(i) = fp(k) + s * (fp(k + 1) - fp(k));
        }
    }
    return y;
}

}  // namespace

double median(const Eigen::VectorXd& v) {
    if (v.size() == 0) throw std::invalid_argument("median of an empty vector");
    std::vector<double> w(v.data(), v.data() + v.size());
    std::sort(w.begin(), w.end());
    const std::size_t m = w.size() / 2;
    return w.size() % 2 ? w[m] : 0.5 * (w[m - 1] + w[m]);
}

// ------------------------------------------------------------------ dynamics
Eigen::VectorXd inverse_dynamics(const Model& model, const Eigen::VectorXd& q, const Eigen::VectorXd& dq,
                                 const Eigen::VectorXd& ddq, bool gravity) {
    const Eigen::Vector3d g = gravity ? model.g : Eigen::Vector3d::Zero();
    return mr::inverse_dynamics(q, dq, ddq, g, mr::Vector6d::Zero(), model.Mlist, model.Glist, model.Slist);
}

Eigen::VectorXd gravity_torques(const Model& model, const Eigen::VectorXd& q) {
    const VectorXd z = VectorXd::Zero(q.size());
    return inverse_dynamics(model, q, z, z);
}

Eigen::MatrixXd gravity_torques_rows(const Model& model, const Eigen::MatrixXd& Q) {
    MatrixXd G(Q.rows(), Q.cols());
    for (Index k = 0; k < Q.rows(); ++k) G.row(k) = gravity_torques(model, Q.row(k).transpose()).transpose();
    return G;
}

// ------------------------------------------------------------------ task and logs
std::string data_path(const std::string& name) { return sbeg::lab_path("../data/" + name); }

sbeg::json load_task(const std::string& name) { return sbeg::load_json(data_path(name)); }

void write_log(const std::string& path, const Eigen::VectorXd& t, const Eigen::MatrixXd& cmd,
               const Eigen::MatrixXd& meas, const Eigen::MatrixXd& cur) {
    const Index N = t.size(), n = cmd.cols();
    if (cmd.rows() != N) throw std::invalid_argument("write_log: cmd must have one row per sample");
    std::vector<std::string> header = {"t"};
    std::vector<const MatrixXd*> blocks = {&cmd};
    for (Index j = 0; j < n; ++j) header.push_back("cmd_" + std::to_string(j + 1));
    if (meas.size() > 0) {
        if (meas.rows() != N || meas.cols() != n) throw std::invalid_argument("write_log: meas must be N x n");
        blocks.push_back(&meas);
        for (Index j = 0; j < n; ++j) header.push_back("meas_" + std::to_string(j + 1));
    }
    if (cur.size() > 0) {
        if (cur.rows() != N || cur.cols() != n) throw std::invalid_argument("write_log: cur must be N x n");
        blocks.push_back(&cur);
        for (Index j = 0; j < n; ++j) header.push_back("cur_" + std::to_string(j + 1));
    }
    std::ofstream out(sbeg::fs_path(path), std::ios::binary);
    if (!out) throw std::runtime_error("cannot write file: " + path);
    for (std::size_t k = 0; k < header.size(); ++k) out << (k ? "," : "") << header[k];
    out << "\n" << std::setprecision(15);
    auto r7 = [](double x) { return std::nearbyint(x * 1e7) / 1e7; };  // np.round(..., 7)
    for (Index i = 0; i < N; ++i) {
        out << r7(t(i));
        for (const MatrixXd* b : blocks)
            for (Index j = 0; j < n; ++j) out << "," << r7((*b)(i, j));
        out << "\n";
    }
}

Log load_log(const std::string& path) {
    const sbeg::CsvTable T = sbeg::read_csv(path);
    Log L;
    auto group = [&](const std::string& prefix) {
        std::vector<std::pair<int, Index>> cols;
        for (std::size_t c = 0; c < T.header.size(); ++c) {
            const std::string& h = T.header[c];
            if (h.rfind(prefix + "_", 0) == 0) cols.emplace_back(std::stoi(h.substr(prefix.size() + 1)), Index(c));
        }
        std::sort(cols.begin(), cols.end());
        MatrixXd M(T.data.rows(), static_cast<Index>(cols.size()));
        for (std::size_t k = 0; k < cols.size(); ++k) M.col(static_cast<Index>(k)) = T.data.col(cols[k].second);
        return cols.empty() ? MatrixXd() : M;
    };
    for (std::size_t c = 0; c < T.header.size(); ++c) {
        const std::string& h = T.header[c];
        if (h == "t") L.t = T.data.col(static_cast<Index>(c));
        else if (h.find('_') == std::string::npos) L.extra[h] = T.data.col(static_cast<Index>(c));
    }
    L.cmd = group("cmd");
    L.meas = group("meas");
    L.cur = group("cur");
    return L;
}

// ------------------------------------------------------------------ servo simulator
Eigen::MatrixXd simulate_servo(const Eigen::VectorXd& t, const Eigen::MatrixXd& cmd, double wn, double zeta,
                               double delay, const Eigen::VectorXd& stiffness, const Model* model, double enc_res) {
    const Index N = t.size(), n = cmd.cols();
    if (N < 1 || cmd.rows() != N) throw std::invalid_argument("simulate_servo: cmd must have one row per sample");
    const double h = 1e-3;
    const Index Nf = static_cast<Index>(std::ceil((t(N - 1) + h / 2 - t(0)) / h));  // np.arange(t0, t_end + h/2, h)
    VectorXd tf(Nf);
    for (Index k = 0; k < Nf; ++k) tf(k) = t(0) + static_cast<double>(k) * h;
    MatrixXd cf(Nf, n);
    for (Index k = 0; k < Nf; ++k) {  // zero-order hold of the delayed command
        const double x = tf(k) - delay + 1e-9;
        Index i = static_cast<Index>(std::upper_bound(t.data(), t.data() + N, x) - t.data()) - 1;
        i = std::min(std::max(i, Index(0)), N - 1);
        cf.row(k) = cmd.row(i);
    }
    VectorXd q = cmd.row(0).transpose();
    VectorXd qd = VectorXd::Zero(n);
    MatrixXd out(Nf, n);
    for (Index k = 0; k < Nf; ++k) {
        out.row(k) = q.transpose();
        const VectorXd qdd = wn * wn * (cf.row(k).transpose() - q) - 2.0 * zeta * wn * qd;
        qd = qd + h * qdd;
        q = q + h * qd;
    }
    MatrixXd meas(N, n);
    for (Index j = 0; j < n; ++j) meas.col(j) = interp(t, tf, out.col(j));
    if (stiffness.size() > 0) {
        if (model == nullptr) throw std::invalid_argument("simulate_servo: stiffness needs a model");
        if (stiffness.size() != n) throw std::invalid_argument("simulate_servo: stiffness must have n entries");
        const MatrixXd g = gravity_torques_rows(*model, meas);
        for (Index k = 0; k < N; ++k) meas.row(k) -= g.row(k).cwiseQuotient(stiffness.transpose());
    }
    if (enc_res != 0.0) meas = (meas / enc_res).unaryExpr([](double x) { return std::nearbyint(x); }) * enc_res;
    return meas;
}

// ------------------------------------------------------------------ Lab 6: command check and metrics
CommandCheck check_command(const Eigen::VectorXd& t, const Eigen::MatrixXd& q, const Model& model,
                           double speed_fraction, double margin_deg, double tol) {
    const Index N = t.size(), n = q.cols();
    if (N < 2 || q.rows() != N) throw std::invalid_argument("check_command: q must have one row per sample (N >= 2)");
    const double m = margin_deg * DEG;
    CommandCheck c;
    c.within_limits = true;
    for (Index j = 0; j < n; ++j)
        if (q.col(j).minCoeff() < model.limits(j, 0) + m || q.col(j).maxCoeff() > model.limits(j, 1) - m)
            c.within_limits = false;
    double ratio = 0.0;
    VectorXd steps(N - 1);
    for (Index k = 0; k + 1 < N; ++k) {
        const VectorXd dq = q.row(k + 1).transpose() - q.row(k).transpose();
        const double dt = t(k + 1) - t(k);
        for (Index j = 0; j < n; ++j)
            ratio = std::max(ratio, std::abs(dq(j) / dt) / (speed_fraction * model.qdot_max(j)));
        steps(k) = dq.cwiseAbs().maxCoeff();
    }
    c.peak_speed_ratio = ratio;
    c.starts_at_rest = steps(0) < 1e-3;
    c.ends_at_rest = steps(N - 2) < 1e-3;
    c.speed_ok = ratio <= 1.0 + tol;
    c.ok = c.within_limits && c.speed_ok && c.starts_at_rest && c.ends_at_rest;
    return c;
}

TrackingMetrics tracking_metrics(const Eigen::VectorXd& t, const Eigen::MatrixXd& cmd, const Eigen::MatrixXd& meas,
                                 double max_lag) {
    const Index N = t.size(), n = cmd.cols();
    if (cmd.rows() != N || meas.rows() != N || meas.cols() != n)
        throw std::invalid_argument("tracking_metrics: cmd and meas must be N x n");
    const MatrixXd e = meas - cmd;
    const double dt = median_step(t);
    double best_r = std::numeric_limits<double>::infinity();
    Index best_k = 0;
    const Index kmax = static_cast<Index>(std::nearbyint(max_lag / dt));
    for (Index k = 0; k <= kmax && k < N; ++k) {
        const MatrixXd ek = meas.bottomRows(N - k) - cmd.topRows(N - k);
        const double r = std::sqrt(ek.array().square().mean());
        if (r < best_r - 1e-15) {
            best_r = r;
            best_k = k;
        }
    }
    const MatrixXd ek = meas.bottomRows(N - best_k) - cmd.topRows(N - best_k);
    TrackingMetrics m;
    m.rms = e.array().square().colwise().mean().sqrt().transpose();
    m.max_abs = e.cwiseAbs().colwise().maxCoeff().transpose();
    m.lag = static_cast<double>(best_k) * dt;
    m.rms_after_lag = ek.array().square().colwise().mean().sqrt().transpose();
    return m;
}

// ------------------------------------------------------------------ static poses, step response
StaticPoses average_approaches(const Log& log) {
    const auto it = log.extra.find("pose");
    if (it == log.extra.end()) throw std::invalid_argument("average_approaches: the log has no 'pose' column");
    const VectorXd& pose = it->second;
    std::vector<int> ids;
    for (Index i = 0; i < pose.size(); ++i) ids.push_back(static_cast<int>(pose(i)));
    std::sort(ids.begin(), ids.end());
    ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
    StaticPoses s;
    s.pose = ids;
    const Index P = static_cast<Index>(ids.size()), n = log.cmd.cols();
    s.cmd = MatrixXd::Zero(P, n);
    s.meas = MatrixXd::Zero(P, n);
    s.cur = MatrixXd::Zero(P, n);
    for (Index p = 0; p < P; ++p) {
        int cnt = 0;
        for (Index i = 0; i < pose.size(); ++i) {
            if (static_cast<double>(ids[static_cast<std::size_t>(p)]) != pose(i)) continue;
            s.cmd.row(p) += log.cmd.row(i);
            s.meas.row(p) += log.meas.row(i);
            s.cur.row(p) += log.cur.row(i);
            ++cnt;
        }
        s.cmd.row(p) /= cnt;
        s.meas.row(p) /= cnt;
        s.cur.row(p) /= cnt;
    }
    return s;
}

Eigen::VectorXd step_response(const Eigen::VectorXd& tau, double wn, double zeta) {
    VectorXd y(tau.size());
    for (Index i = 0; i < tau.size(); ++i) {
        const double x = std::max(tau(i), 0.0);
        double v;
        if (zeta < 1.0 - 1e-6) {
            const double wd = wn * std::sqrt(1 - zeta * zeta);
            v = 1 - std::exp(-zeta * wn * x) * (std::cos(wd * x) + zeta / std::sqrt(1 - zeta * zeta) * std::sin(wd * x));
        } else if (zeta > 1.0 + 1e-6) {
            const double a = wn * (zeta + std::sqrt(zeta * zeta - 1)), b = wn * (zeta - std::sqrt(zeta * zeta - 1));
            v = 1 - (a * std::exp(-b * x) - b * std::exp(-a * x)) / (a - b);
        } else {
            v = 1 - std::exp(-wn * x) * (1 + wn * x);
        }
        y(i) = tau(i) > 0 ? v : 0.0;
    }
    return y;
}

// ------------------------------------------------------------------ SciPy replacement
LeastSquaresResult least_squares(const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& fun,
                                 const Eigen::VectorXd& x0, const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
                                 const Eigen::VectorXd& x_scale, int max_iter) {
    const Index p = x0.size();
    if (lb.size() != p || ub.size() != p || x_scale.size() != p)
        throw std::invalid_argument("least_squares: x0, lb, ub and x_scale must have the same length");
    auto clip = [&](const VectorXd& x) { return x.cwiseMax(lb).cwiseMin(ub).eval(); };
    VectorXd x = clip(x0);
    VectorXd r = fun(x);
    double cost = 0.5 * r.squaredNorm();
    double lambda = 1e-3;
    for (int it = 0; it < max_iter; ++it) {
        MatrixXd J(r.size(), p);  // forward differences (backward at an upper bound), in scaled variables
        for (Index k = 0; k < p; ++k) {
            double hk = 1e-7 * std::max(std::abs(x(k)), x_scale(k));
            VectorXd xk = x;
            if (xk(k) + hk > ub(k)) hk = -hk;
            xk(k) += hk;
            J.col(k) = (fun(xk) - r) / hk * x_scale(k);
        }
        const MatrixXd A = J.transpose() * J;
        const VectorXd g = J.transpose() * r;
        if (g.lpNorm<Eigen::Infinity>() < 1e-14) break;
        bool accepted = false;
        VectorXd xn, rn;
        double cn = cost;
        for (int inner = 0; inner < 30 && !accepted; ++inner) {
            const MatrixXd Al = A + lambda * MatrixXd::Identity(p, p);
            const VectorXd du = Al.ldlt().solve(-g);
            xn = clip(x + x_scale.cwiseProduct(du));
            rn = fun(xn);
            cn = 0.5 * rn.squaredNorm();
            if (std::isfinite(cn) && cn < cost) {
                accepted = true;
                lambda = std::max(lambda / 3.0, 1e-12);
            } else {
                lambda *= 4.0;
            }
        }
        if (!accepted) break;
        const double step = (xn - x).cwiseQuotient(x_scale).norm();
        const double drop = cost - cn;
        x = xn;
        r = rn;
        cost = cn;
        if (drop <= 1e-12 * cost || step <= 1e-10) break;
    }
    return LeastSquaresResult{x, cost};
}

}  // namespace lab
