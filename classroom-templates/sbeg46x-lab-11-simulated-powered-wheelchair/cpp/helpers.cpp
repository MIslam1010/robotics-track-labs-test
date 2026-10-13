// SBEG 46X Lab 11 -- provided helpers (C++ track). Given code: do not edit; use these functions freely.
//
//   load_params, landmark_table, load_log, load_meas    data/wheelchair.json, data/ward_log.csv, data/ward_meas.csv
//   wrap, rot2                                          small helpers of starter.py
//   reference_trajectory                                the ward loop q_d(t), v_d, omega_d
//   pose_errors, distance_travelled, tracking_metrics   metrics
//   sim_noise, simulate                                 the closed-loop simulator (calls YOUR functions)
//
// These are the "Provided" functions of starter.py, translated to C++ line by line. The only addition is sim_noise:
// C++ cannot reproduce NumPy's random streams, so the noise of seed 1211 (the seed of the tests and experiments) is
// read from cpp/sim_noise_1211.json, which holds the NumPy draws of the Python simulate().
#include "lab.hpp"

#include <sbeg/labio.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include <set>
#include <stdexcept>
#include <string>

namespace lab {

namespace {
constexpr double PI = 3.14159265358979323846;
constexpr double DEG = PI / 180.0;  // math.radians(x) = x * (pi / 180)

/// Exact displacement of a constant planar body twist (w, vx, vy) applied for unit time from q = (phi, x, y).
/// Used by the simulator's 'true' chair (it does not call your code).
Eigen::Vector3d arc(const Eigen::Vector3d& q, double w, double vx, double vy) {
    Eigen::Vector3d dqb;
    if (std::abs(w) < 1e-12) {
        dqb << 0.0, vx, vy;
    } else {
        dqb << w, (vx * std::sin(w) + vy * (std::cos(w) - 1)) / w, (vy * std::sin(w) + vx * (1 - std::cos(w))) / w;
    }
    const Eigen::Vector2d xy = rot2(q(0)) * dqb.tail<2>();
    return q + Eigen::Vector3d(dqb(0), xy(0), xy(1));
}

/// Small portable generator for seeds other than 1211 (not NumPy's stream).
class Rng {
public:
    explicit Rng(std::uint64_t seed) : gen_(seed) {}
    double uniform(double lo = 0.0, double hi = 1.0) {
        constexpr double INV_2_53 = 1.0 / 9007199254740992.0;  // 2^-53
        return lo + (hi - lo) * (static_cast<double>(gen_() >> 11) * INV_2_53);
    }
    double normal() {
        if (has_spare_) {
            has_spare_ = false;
            return spare_;
        }
        constexpr double INV_2_53 = 1.0 / 9007199254740992.0;
        const double u1 = (static_cast<double>(gen_() >> 11) + 0.5) * INV_2_53;  // in (0, 1)
        const double u2 = (static_cast<double>(gen_() >> 11) + 0.5) * INV_2_53;
        const double rr = std::sqrt(-2.0 * std::log(u1));
        spare_ = rr * std::sin(2.0 * PI * u2);
        has_spare_ = true;
        return rr * std::cos(2.0 * PI * u2);
    }

private:
    std::mt19937_64 gen_;
    bool has_spare_ = false;
    double spare_ = 0.0;
};
}  // namespace

// ------------------------------------------------------------------------------------------------ data
Params load_params(const std::string& path) {
    const sbeg::json j = sbeg::load_json(path.empty() ? sbeg::lab_path("../data/wheelchair.json") : path);
    Params p;
    const sbeg::json& ch = j.at("chair");
    p.chair.r = ch.at("r").get<double>();
    p.chair.d = ch.at("d").get<double>();
    p.chair.u_max = ch.at("u_max").get<double>();
    p.chair.v_max = ch.at("v_max").get<double>();
    p.chair.enc_counts_per_rev = ch.at("enc_counts_per_rev").get<int>();
    p.chair.h = ch.at("h").get<double>();
    const sbeg::json& tr = j.at("truth");
    p.truth.r_L = tr.at("r_L").get<double>();
    p.truth.r_R = tr.at("r_R").get<double>();
    p.truth.d = tr.at("d").get<double>();
    p.truth.k_slip = tr.at("k_slip").get<double>();
    const sbeg::json& se = j.at("sensor");
    p.sensor.every = se.at("every").get<int>();
    p.sensor.max_range = se.at("max_range").get<double>();
    p.sensor.sigma_range = se.at("sigma_range").get<double>();
    p.sensor.sigma_bearing_deg = se.at("sigma_bearing_deg").get<double>();
    p.sensor.p_outlier = se.at("p_outlier").get<double>();
    p.sensor.outlier_short = sbeg::as<Eigen::Vector2d>(se.at("outlier_short"));
    p.landmarks = sbeg::as_mat(j.at("landmarks"));
    const sbeg::json& ek = j.at("ekf");
    p.ekf.k_s = ek.at("k_s").get<double>();
    p.ekf.sigma_range = ek.at("sigma_range").get<double>();
    p.ekf.sigma_bearing_deg = ek.at("sigma_bearing_deg").get<double>();
    p.ekf.P0_sigma = sbeg::as<Eigen::Vector3d>(ek.at("P0_sigma"));
    p.ekf.gate = ek.at("gate").get<double>();
    const sbeg::json& rt = j.at("route");
    p.route.v = rt.at("v").get<double>();
    p.route.laps = rt.at("laps").get<int>();
    for (const sbeg::json& s : rt.at("segments")) {
        RouteSegment seg;
        seg.kind = s.at(0).get<std::string>();
        seg.a = s.at(1).get<double>();
        if (seg.kind != "line") seg.b = s.at(2).get<double>();
        p.route.segments.push_back(seg);
    }
    p.route.start = sbeg::as<Eigen::Vector3d>(rt.at("start"));
    const sbeg::json& ct = j.at("control");
    p.control.k1 = ct.at("k1").get<double>();
    p.control.k2 = ct.at("k2").get<double>();
    p.control.k3 = ct.at("k3").get<double>();
    p.control.xr = ct.at("xr").get<double>();
    p.control.kp = ct.at("kp").get<double>();
    p.control.q0_offset = sbeg::as<Eigen::Vector3d>(ct.at("q0_offset"));
    p.arm_mount.p_b0 = sbeg::as<Eigen::Vector3d>(j.at("arm_mount").at("p_b0"));
    p.arm_mount.theta_feed = sbeg::as_vec(j.at("arm_mount").at("theta_feed"));
    return p;
}

std::map<int, Eigen::Vector2d> landmark_table(const Params& p, const std::optional<std::vector<int>>& ids) {
    std::map<int, Eigen::Vector2d> out;
    for (Eigen::Index i = 0; i < p.landmarks.rows(); ++i) {
        const int id = static_cast<int>(p.landmarks(i, 0));
        if (ids && std::find(ids->begin(), ids->end(), id) == ids->end()) continue;
        out[id] = Eigen::Vector2d(p.landmarks(i, 1), p.landmarks(i, 2));
    }
    return out;
}

Log load_log(const std::string& path) {
    const sbeg::CsvTable T = sbeg::read_csv(path.empty() ? sbeg::lab_path("../data/ward_log.csv") : path);
    Log log;
    log.t = T.data.col(T.col("t"));
    log.enc_L = T.data.col(T.col("enc_L"));
    log.enc_R = T.data.col(T.col("enc_R"));
    log.phi = T.data.col(T.col("phi"));
    log.x = T.data.col(T.col("x"));
    log.y = T.data.col(T.col("y"));
    return log;
}

std::vector<Reading> load_meas(const std::string& path) {
    const sbeg::CsvTable T = sbeg::read_csv(path.empty() ? sbeg::lab_path("../data/ward_meas.csv") : path);
    const Eigen::Index ck = T.col("k"), ct = T.col("t"), ci = T.col("id"), cr = T.col("range"), cb = T.col("bearing");
    std::vector<Reading> out;
    out.reserve(static_cast<std::size_t>(T.data.rows()));
    for (Eigen::Index i = 0; i < T.data.rows(); ++i) {
        Reading z;
        z.k = static_cast<int>(std::lround(T.data(i, ck)));
        z.t = T.data(i, ct);
        z.id = static_cast<int>(std::lround(T.data(i, ci)));
        z.range = T.data(i, cr);
        z.bearing = T.data(i, cb);
        out.push_back(z);
    }
    return out;
}

// ------------------------------------------------------------------------------------------------ small helpers
double wrap(double a) {
    // Python: -((-a + pi) % (2 pi) - pi), with Python's floored modulo
    const double two_pi = 2 * PI;
    double m = std::fmod(-a + PI, two_pi);
    if (m != 0.0) {
        if (m < 0) m += two_pi;
    } else {
        m = 0.0;
    }
    return -(m - PI);
}

Eigen::VectorXd wrap(const Eigen::VectorXd& a) {
    Eigen::VectorXd out(a.size());
    for (Eigen::Index i = 0; i < a.size(); ++i) out(i) = wrap(a(i));
    return out;
}

Eigen::Matrix2d rot2(double phi) {
    const double c = std::cos(phi), s = std::sin(phi);
    Eigen::Matrix2d R;
    R << c, -s, s, c;
    return R;
}

// ------------------------------------------------------------------------------------------------ reference route
Reference reference_trajectory(const Params& p) {
    const double h = p.chair.h, v = p.route.v;
    struct Seg {
        Eigen::Vector3d q0;
        double L, k;
    };
    std::vector<Seg> segs;
    Eigen::Vector3d q0 = p.route.start;
    for (int lap = 0; lap < p.route.laps; ++lap) {
        for (const RouteSegment& s : p.route.segments) {
            double L = 0.0, k = 0.0;
            if (s.kind == "line") {
                L = s.a;
            } else {
                L = s.a * (s.b * DEG);
                k = 1.0 / s.a;
            }
            segs.push_back({q0, L, k});
            if (k == 0.0) {
                q0 = q0 + Eigen::Vector3d(0.0, L * std::cos(q0(0)), L * std::sin(q0(0)));
            } else {
                const double ph = q0(0) + k * L;
                q0 = Eigen::Vector3d(ph, q0(1) + (std::sin(ph) - std::sin(q0(0))) / k,
                                     q0(2) - (std::cos(ph) - std::cos(q0(0))) / k);
            }
        }
    }
    double total = 0.0;
    for (const Seg& s : segs) total += s.L;
    const Eigen::Index N = static_cast<Eigen::Index>(std::floor(total / (v * h) + 1e-9)) + 1;
    Reference ref;
    ref.t.resize(N);
    ref.qd = Eigen::MatrixXd::Zero(N, 3);
    ref.vd = Eigen::VectorXd::Constant(N, v);
    ref.wd = Eigen::VectorXd::Zero(N);
    for (Eigen::Index i = 0; i < N; ++i) {
        const double ti = static_cast<double>(i) * h;
        ref.t(i) = ti;
        double s = v * ti;
        const Seg* cur = &segs.back();
        for (const Seg& sg : segs) {
            cur = &sg;
            if (s <= sg.L + 1e-12) break;
            s -= sg.L;
        }
        const Eigen::Vector3d& qs = cur->q0;
        const double k = cur->k;
        if (k == 0.0) {
            ref.qd.row(i) << qs(0), qs(1) + s * std::cos(qs(0)), qs(2) + s * std::sin(qs(0));
        } else {
            const double ph = qs(0) + k * s;
            ref.qd.row(i) << ph, qs(1) + (std::sin(ph) - std::sin(qs(0))) / k,
                qs(2) - (std::cos(ph) - std::cos(qs(0))) / k;
        }
        ref.wd(i) = v * k;
    }
    return ref;
}

// ------------------------------------------------------------------------------------------------ metrics
PoseErrors pose_errors(const Eigen::MatrixXd& q_est, const Eigen::MatrixXd& q_true) {
    if (q_est.rows() != q_true.rows() || q_est.cols() != 3 || q_true.cols() != 3)
        throw std::invalid_argument("pose_errors: q_est and q_true must both be N x 3");
    PoseErrors e;
    e.e_pos.resize(q_est.rows());
    e.e_phi.resize(q_est.rows());
    for (Eigen::Index i = 0; i < q_est.rows(); ++i) {
        e.e_pos(i) = std::hypot(q_est(i, 1) - q_true(i, 1), q_est(i, 2) - q_true(i, 2));
        e.e_phi(i) = wrap(q_est(i, 0) - q_true(i, 0));
    }
    return e;
}

Eigen::VectorXd distance_travelled(const Eigen::MatrixXd& q_true) {
    Eigen::VectorXd s = Eigen::VectorXd::Zero(q_true.rows());
    for (Eigen::Index i = 1; i < q_true.rows(); ++i)
        s(i) = s(i - 1) + std::hypot(q_true(i, 1) - q_true(i - 1, 1), q_true(i, 2) - q_true(i - 1, 2));
    return s;
}

TrackingMetrics tracking_metrics(const SimResult& sim, double t_from) {
    const Eigen::Index N = sim.t.size();
    const PoseErrors e = pose_errors(sim.q_true.topRows(N), sim.qd);
    double s2 = 0.0, sp = 0.0, mx = 0.0;
    Eigen::Index n = 0;
    for (Eigen::Index i = 0; i < N; ++i) {
        if (sim.t(i) >= t_from - 1e-12) {
            s2 += e.e_pos(i) * e.e_pos(i);
            sp += e.e_phi(i) * e.e_phi(i);
            mx = std::max(mx, e.e_pos(i));
            ++n;
        }
    }
    TrackingMetrics m;
    m.rms_pos = std::sqrt(s2 / static_cast<double>(n));
    m.max_pos = mx;
    m.rms_phi = std::sqrt(sp / static_cast<double>(n));
    m.final_pos = e.e_pos(N - 1);
    m.peak_u = std::max(sim.uL.cwiseAbs().maxCoeff(), sim.uR.cwiseAbs().maxCoeff());
    double ns = 0.0;
    for (bool b : sim.sat) ns += b ? 1.0 : 0.0;
    m.frac_sat = ns / static_cast<double>(sim.sat.size());
    return m;
}

// ------------------------------------------------------------------------------------------------ simulator
SimNoise sim_noise(const Params& p, int seed, Eigen::Index N) {
    const Eigen::Index nl = p.landmarks.rows();
    SimNoise z;
    z.slip = Eigen::MatrixXd::Zero(N, 2);
    z.n_range = Eigen::MatrixXd::Zero(N + 1, nl);
    z.n_bearing = Eigen::MatrixXd::Zero(N + 1, nl);
    z.u_outlier = Eigen::MatrixXd::Ones(N + 1, nl);
    z.a_outlier = Eigen::MatrixXd::Zero(N + 1, nl);
    if (seed == 1211) {
        const sbeg::json J = sbeg::load_json(sbeg::lab_path("sim_noise_1211.json"));
        if (J.at("N").get<Eigen::Index>() == N && J.at("n_landmarks").get<Eigen::Index>() == nl &&
            J.at("every").get<int>() == p.sensor.every) {
            z.slip = sbeg::as_mat(J.at("slip"));
            const Eigen::MatrixXd nr = sbeg::as_mat(J.at("n_range")), nb = sbeg::as_mat(J.at("n_bearing"));
            const Eigen::MatrixXd uo = sbeg::as_mat(J.at("u_outlier")), ao = sbeg::as_mat(J.at("a_outlier"));
            const std::vector<int> ks = J.at("scan_k").get<std::vector<int>>();
            for (std::size_t i = 0; i < ks.size(); ++i) {
                const Eigen::Index r = static_cast<Eigen::Index>(i);
                z.n_range.row(ks[i]) = nr.row(r);
                z.n_bearing.row(ks[i]) = nb.row(r);
                z.u_outlier.row(ks[i]) = uo.row(r);
                z.a_outlier.row(ks[i]) = ao.row(r);
            }
            return z;
        }
    }
    Rng rng(static_cast<std::uint64_t>(seed));  // same order of draws as the Python simulate()
    for (Eigen::Index k = 0; k < N; ++k)
        for (int c = 0; c < 2; ++c) z.slip(k, c) = rng.normal();
    for (Eigen::Index k = 0; k <= N; ++k)
        for (Eigen::Index j = 0; j < nl; ++j) {
            z.n_range(k, j) = rng.normal();
            z.n_bearing(k, j) = rng.normal();
        }
    for (Eigen::Index k = 0; k <= N; ++k)
        for (Eigen::Index j = 0; j < nl; ++j) z.u_outlier(k, j) = rng.uniform();
    for (Eigen::Index k = 0; k <= N; ++k)
        for (Eigen::Index j = 0; j < nl; ++j)
            z.a_outlier(k, j) = rng.uniform(p.sensor.outlier_short(0), p.sensor.outlier_short(1));
    return z;
}

SimResult simulate(const Params& p, const std::string& estimator, const std::string& controller,
                   const SimOptions& opt) {
    if (estimator != "truth" && estimator != "odometry" && estimator != "ekf")
        throw std::invalid_argument("simulate: unknown estimator '" + estimator + "'");
    if (controller != "nonlinear" && controller != "point")
        throw std::invalid_argument("simulate: unknown controller '" + controller + "'");
    const Params::Chair& ch = p.chair;
    const Params::Truth& tr = p.truth;
    const Params::Sensor& se = p.sensor;
    const Params::Ekf& ek = p.ekf;
    Params::Control ct = p.control;
    for (const auto& kv : opt.gains) {
        if (kv.first == "k1") ct.k1 = kv.second;
        else if (kv.first == "k2") ct.k2 = kv.second;
        else if (kv.first == "k3") ct.k3 = kv.second;
        else if (kv.first == "kp") ct.kp = kv.second;
        else if (kv.first == "xr") ct.xr = kv.second;
        else throw std::invalid_argument("simulate: unknown gain '" + kv.first + "'");
    }
    const double r = ch.r, d = ch.d, h = ch.h;
    const double res = 2 * PI / ch.enc_counts_per_rev;
    const Reference ref = reference_trajectory(p);
    const Eigen::Index N = ref.t.size();
    const std::map<int, Eigen::Vector2d> lm = landmark_table(p);
    std::vector<int> ids;
    for (const auto& kv : lm) ids.push_back(kv.first);  // sorted (std::map)
    const std::set<int> present = opt.landmark_ids ? std::set<int>(opt.landmark_ids->begin(), opt.landmark_ids->end())
                                                   : std::set<int>(ids.begin(), ids.end());
    const SimNoise nz = sim_noise(p, opt.seed, N);
    const double sb = se.sigma_bearing_deg * DEG;
    Eigen::Matrix2d Rm = Eigen::Matrix2d::Zero();
    Rm(0, 0) = ek.sigma_range * ek.sigma_range;
    Rm(1, 1) = std::pow(ek.sigma_bearing_deg * DEG, 2);
    const double ks = opt.k_s ? *opt.k_s : ek.k_s;
    Eigen::MatrixXd F;
    if (estimator != "truth") F = diffdrive_F(r, d);
    const Eigen::Vector3d off = opt.offset ? *opt.offset : ct.q0_offset;
    Eigen::Vector3d q = ref.qd.row(0).transpose() + off;
    Eigen::Vector3d qh = q;
    Eigen::Matrix3d P = ek.P0_sigma.cwiseProduct(ek.P0_sigma).asDiagonal();
    Eigen::Vector2d th = Eigen::Vector2d::Zero(), encq = Eigen::Vector2d::Zero();

    SimResult out;
    out.t = ref.t;
    out.qd = ref.qd;
    out.vd = ref.vd;
    out.wd = ref.wd;
    out.q_true.resize(N + 1, 3);
    out.q_est.resize(N + 1, 3);
    out.enc.resize(N + 1, 2);
    out.q_true.row(0) = q.transpose();
    out.q_est.row(0) = qh.transpose();
    out.enc.row(0) = encq.transpose();
    out.P.push_back(P);
    out.v = Eigen::VectorXd::Zero(N);
    out.w = Eigen::VectorXd::Zero(N);
    out.uL = Eigen::VectorXd::Zero(N);
    out.uR = Eigen::VectorXd::Zero(N);
    out.sat.assign(static_cast<std::size_t>(N), false);

    for (Eigen::Index k = 0; k < N; ++k) {
        const Eigen::Vector3d qc = estimator == "truth" ? q : qh;
        const Eigen::Vector3d qdk = ref.qd.row(k).transpose();
        std::pair<double, double> vw;
        if (controller == "nonlinear")
            vw = tracking_control(qc, qdk, ref.vd(k), ref.wd(k), ct.k1, ct.k2, ct.k3);
        else
            vw = point_tracking_control(qc, qdk, ref.vd(k), ref.wd(k), ct.xr, ct.kp);
        const double v = vw.first, w = vw.second;
        const std::pair<double, double> sw = saturate_wheels(v, w, r, d, ch.u_max, ch.v_max);
        const double vs = sw.first, ws = sw.second;
        out.sat[static_cast<std::size_t>(k)] = std::abs(vs - v) > 1e-12 || std::abs(ws - w) > 1e-12;
        const std::pair<double, double> u = wheel_speeds(vs, ws, r, d);
        out.v(k) = vs;
        out.w(k) = ws;
        out.uL(k) = u.first;
        out.uR(k) = u.second;
        const Eigen::Vector2d dth(u.first * h, u.second * h);
        // true chair: true radii and half-track, random slip on each wheel's ground distance
        const double sL = tr.r_L * dth(0) + std::sqrt(tr.k_slip * std::abs(r * dth(0))) * nz.slip(k, 0);
        const double sR = tr.r_R * dth(1) + std::sqrt(tr.k_slip * std::abs(r * dth(1))) * nz.slip(k, 1);
        q = arc(q, (sR - sL) / (2 * tr.d), (sL + sR) / 2, 0.0);
        th = th + dth;
        Eigen::Vector2d enc_new;
        for (int c = 0; c < 2; ++c) enc_new(c) = std::floor(th(c) / res + 1e-9) * res;
        const Eigen::Vector2d ddth = enc_new - encq;
        encq = enc_new;
        if (estimator == "odometry") {
            qh = odometry_step(qh, ddth, F);
        } else if (estimator == "ekf") {
            const std::pair<Eigen::Vector3d, Eigen::Matrix3d> pr = ekf_predict(qh, P, ddth, r, d, ks);
            qh = pr.first;
            P = pr.second;
        }
        if ((k + 1) % se.every == 0) {
            for (std::size_t jj = 0; jj < ids.size(); ++jj) {
                const int i = ids[jj];
                const Eigen::Index j = static_cast<Eigen::Index>(jj);
                if (!present.count(i)) continue;
                const Eigen::Vector2d& m = lm.at(i);
                const double dx = m(0) - q(1), dy = m(1) - q(2);
                const double rho = std::hypot(dx, dy);
                if (rho > se.max_range) continue;
                Eigen::Vector2d z(rho + se.sigma_range * nz.n_range(k + 1, j),
                                  wrap(std::atan2(dy, dx) - q(0) + sb * nz.n_bearing(k + 1, j)));
                if (nz.u_outlier(k + 1, j) < se.p_outlier) z(0) = std::max(0.05, z(0) - nz.a_outlier(k + 1, j));
                Reading rd;
                rd.k = static_cast<int>(k + 1);
                rd.t = static_cast<double>(k + 1) * h;
                rd.id = i;
                rd.range = z(0);
                rd.bearing = z(1);
                out.meas.push_back(rd);
                if (estimator == "ekf") {
                    const EkfUpdate upd =
                        ekf_update(qh, P, z, m, Rm, opt.use_gate ? std::optional<double>(ek.gate) : std::nullopt);
                    qh = upd.q;
                    P = upd.P;
                    (upd.accepted ? out.n_acc : out.n_rej) += 1;
                }
            }
        }
        if (estimator == "truth") qh = q;
        out.q_true.row(k + 1) = q.transpose();
        out.q_est.row(k + 1) = qh.transpose();
        out.P.push_back(P);
        out.enc.row(k + 1) = encq.transpose();
    }
    return out;
}

}  // namespace lab
