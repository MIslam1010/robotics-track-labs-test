// SBEG 461 Lab 5 -- provided helpers (C++ track). Given code: do not edit; use these functions freely.
//
//   skew, trans_inv, adjoint, ad, matrix_exp6, rnea      the small SE(3) helpers and the RNEA of Labs 2/4
//   load_course_arm, load_tasks, Tasks::config            data/course_arm.json, data/tasks.json
//   export_csv                                            t, q1..qn, qd1..qdn to a CSV file
//   scalar overloads of the three time scalings and the vector overload of bspline_basis
//
// These are the "Provided helpers" of starter.py, translated to C++.
#include "lab.hpp"

#include <sbeg/labio.hpp>

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace lab {

// ---------------------------------------------------------------------------
// SE(3) helpers and the RNEA
// ---------------------------------------------------------------------------

Eigen::Matrix3d skew(const Eigen::Vector3d& w) {
    Eigen::Matrix3d m;
    m << 0.0, -w(2), w(1),
         w(2), 0.0, -w(0),
         -w(1), w(0), 0.0;
    return m;
}

Eigen::Matrix4d trans_inv(const Eigen::Matrix4d& T) {
    const Eigen::Matrix3d R = T.topLeftCorner<3, 3>();
    const Eigen::Vector3d p = T.block<3, 1>(0, 3);
    Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
    out.topLeftCorner<3, 3>() = R.transpose();
    out.block<3, 1>(0, 3) = -R.transpose() * p;
    return out;
}

sbeg::Matrix6d adjoint(const Eigen::Matrix4d& T) {
    const Eigen::Matrix3d R = T.topLeftCorner<3, 3>();
    const Eigen::Vector3d p = T.block<3, 1>(0, 3);
    sbeg::Matrix6d Ad = sbeg::Matrix6d::Zero();
    Ad.topLeftCorner<3, 3>() = R;
    Ad.bottomLeftCorner<3, 3>() = skew(p) * R;
    Ad.bottomRightCorner<3, 3>() = R;
    return Ad;
}

sbeg::Matrix6d ad(const sbeg::Vector6d& V) {
    sbeg::Matrix6d out = sbeg::Matrix6d::Zero();
    const Eigen::Matrix3d W = skew(V.head<3>());
    out.topLeftCorner<3, 3>() = W;
    out.bottomLeftCorner<3, 3>() = skew(V.tail<3>());
    out.bottomRightCorner<3, 3>() = W;
    return out;
}

Eigen::Matrix4d matrix_exp6(const sbeg::Vector6d& S, double theta) {
    const Eigen::Vector3d w = S.head<3>(), v = S.tail<3>();
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    if (w.norm() < 1e-12) {
        T.block<3, 1>(0, 3) = v * theta;
        return T;
    }
    const Eigen::Matrix3d W = skew(w);
    const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d R = I + std::sin(theta) * W + (1.0 - std::cos(theta)) * W * W;
    const Eigen::Matrix3d G = I * theta + (1.0 - std::cos(theta)) * W + (theta - std::sin(theta)) * W * W;
    T.topLeftCorner<3, 3>() = R;
    T.block<3, 1>(0, 3) = G * v;
    return T;
}

Eigen::VectorXd rnea(const Eigen::VectorXd& thetalist, const Eigen::VectorXd& dthetalist,
                     const Eigen::VectorXd& ddthetalist, const Eigen::Vector3d& g, const sbeg::Vector6d& Ftip,
                     const mr::TransformList& Mlist, const mr::InertiaList& Glist, const Eigen::MatrixXd& Slist) {
    const Eigen::Index n = thetalist.size();
    const std::size_t nn = static_cast<std::size_t>(n);
    if (dthetalist.size() != n || ddthetalist.size() != n || Slist.rows() != 6 || Slist.cols() != n ||
        Mlist.size() != nn + 1 || Glist.size() != nn)
        throw std::invalid_argument("rnea: inconsistent sizes of thetalist, dthetalist, ddthetalist, Mlist, Glist, Slist");
    Eigen::Matrix4d Mi = Eigen::Matrix4d::Identity();
    Eigen::MatrixXd Ai = Eigen::MatrixXd::Zero(6, n);
    std::vector<sbeg::Matrix6d> AdTi(nn + 1);
    Eigen::MatrixXd Vi = Eigen::MatrixXd::Zero(6, n + 1);
    Eigen::MatrixXd Vdi = Eigen::MatrixXd::Zero(6, n + 1);
    Vdi.block<3, 1>(3, 0) = -g;
    AdTi[nn] = adjoint(trans_inv(Mlist[nn]));
    sbeg::Vector6d Fi = Ftip;
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(n);
    for (Eigen::Index i = 0; i < n; ++i) {
        const std::size_t k = static_cast<std::size_t>(i);
        Mi = Mi * Mlist[k];
        Ai.col(i) = adjoint(trans_inv(Mi)) * Slist.col(i);
        const sbeg::Vector6d A = Ai.col(i);
        const Eigen::Matrix4d Ti = matrix_exp6(-A, thetalist(i)) * trans_inv(Mlist[k]);
        AdTi[k] = adjoint(Ti);
        Vi.col(i + 1) = AdTi[k] * Vi.col(i) + A * dthetalist(i);
        const sbeg::Vector6d V = Vi.col(i + 1);
        Vdi.col(i + 1) = AdTi[k] * Vdi.col(i) + A * ddthetalist(i) + ad(V) * A * dthetalist(i);
    }
    for (Eigen::Index i = n - 1; i >= 0; --i) {
        const std::size_t k = static_cast<std::size_t>(i);
        const sbeg::Vector6d V = Vi.col(i + 1), Vd = Vdi.col(i + 1);
        const sbeg::Vector6d F = AdTi[k + 1].transpose() * Fi + Glist[k] * Vd - ad(V).transpose() * (Glist[k] * V);
        Fi = F;
        tau(i) = Fi.dot(Ai.col(i));
    }
    return tau;
}

// ---------------------------------------------------------------------------
// Data
// ---------------------------------------------------------------------------

Robot load_course_arm(bool payload, bool derated) {
    const sbeg::json d = sbeg::load_json(sbeg::lab_path("../data/course_arm.json"));
    const double kf = derated ? d["derated_speed_fraction"].get<double>() : 1.0;
    const double tf = derated ? d["derated_current_fraction"].get<double>() : 1.0;
    Robot r;
    r.Slist = sbeg::as_mat(d["Slist"]);
    r.M = sbeg::as<Eigen::Matrix4d>(d["M"]);
    r.Mlist = sbeg::as_list<Eigen::Matrix4d>(d["Mlist"]);
    r.Glist = sbeg::as_list<sbeg::Matrix6d>(payload ? d["Glist_payload"] : d["Glist"]);
    r.g = sbeg::as<Eigen::Vector3d>(d["g"]);
    r.qdot_max = kf * sbeg::as_vec(d["qdot_max"]);
    r.tau_max = tf * sbeg::as_vec(d["tau_max"]);
    r.limits = sbeg::as_mat(d["limits"]);
    r.q_home = sbeg::as_vec(d["q_home"]);
    return r;
}

Tasks load_tasks() {
    const sbeg::json t = sbeg::load_json(sbeg::lab_path("../data/tasks.json"));
    Tasks out;
    out.A = sbeg::as_vec(t["A"]);
    out.B = sbeg::as_vec(t["B"]);
    out.C = sbeg::as_vec(t["C"]);
    out.D = sbeg::as_vec(t["D"]);
    out.W = sbeg::as_vec(t["W"]);
    out.via_times = sbeg::as_vec(t["via_times"]);
    out.qddot_lim = sbeg::as_vec(t["qddot_lim"]);
    out.payload_kg = t["payload_kg"].get<double>();
    out.description = t["description"].get<std::string>();
    return out;
}

const Eigen::VectorXd& Tasks::config(const std::string& name) const {
    if (name == "A") return A;
    if (name == "B") return B;
    if (name == "C") return C;
    if (name == "D") return D;
    if (name == "W") return W;
    throw std::invalid_argument("Tasks::config: no task configuration named '" + name + "'");
}

void export_csv(const std::string& path, const Eigen::VectorXd& t, const Eigen::MatrixXd& Q,
                const Eigen::MatrixXd& Qd) {
    if (Q.rows() != t.size() || Qd.rows() != t.size() || Qd.cols() != Q.cols())
        throw std::invalid_argument("export_csv: t, Q and Qd must have the same number of rows");
    const Eigen::Index n = Q.cols();
    std::vector<std::string> header{"t"};
    for (Eigen::Index i = 0; i < n; ++i) header.push_back("q" + std::to_string(i + 1));
    for (Eigen::Index i = 0; i < n; ++i) header.push_back("qd" + std::to_string(i + 1));
    Eigen::MatrixXd data(t.size(), 1 + 2 * n);
    data << t, Q, Qd;
    sbeg::write_csv(path, header, data);
}

// ---------------------------------------------------------------------------
// Scalar / vector overloads (they call the functions you write in starter.cpp)
// ---------------------------------------------------------------------------

namespace {
TimeScalingPoint first_sample(const TimeScaling& r) {
    if (r.s.size() != 1 || r.sd.size() != 1 || r.sdd.size() != 1)
        throw std::invalid_argument("time scaling: one time in, but not one (s, sdot, sddot) out");
    return {r.s(0), r.sd(0), r.sdd(0)};
}
Eigen::VectorXd one(double t) { return Eigen::VectorXd::Constant(1, t); }
}  // namespace

TimeScalingPoint cubic_time_scaling(double T, double t) { return first_sample(cubic_time_scaling(T, one(t))); }

TimeScalingPoint quintic_time_scaling(double T, double t) { return first_sample(quintic_time_scaling(T, one(t))); }

TimeScalingPoint trapezoid_time_scaling(double v, double a, double t) {
    return first_sample(trapezoid_time_scaling(v, a, one(t)));
}

Eigen::MatrixXd bspline_basis(const Eigen::VectorXd& u) {
    Eigen::MatrixXd b(4, u.size());
    for (Eigen::Index k = 0; k < u.size(); ++k) b.col(k) = bspline_basis(u(k));
    return b;
}

}  // namespace lab
