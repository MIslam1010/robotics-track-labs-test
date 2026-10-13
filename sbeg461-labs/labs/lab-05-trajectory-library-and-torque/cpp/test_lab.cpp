// Auto-checks for SBEG 461 Lab 5 (trajectory library; torque feasibility on the course arm) -- C++ track.
// Mirrors test_lab.py test by test (a parametrized Python test is ONE test case here that loops over the values).
//
// Build and run from this folder:
//     cmake -S . -B build -G Ninja && cmake --build build && ./build/lab_tests
//     ./build/lab_tests -tc="*cubic*,*quintic*"      (run only some tests, like pytest -k)
// Ground truth: mrcpp (the C++ port of modern_robotics: cubic/quintic_time_scaling, joint_trajectory,
// inverse_dynamics_trajectory), the numbers printed in MR Sec. 9.2-9.3, a brute-force bisection on the duration, and
// cpp_cases.json, which holds the numbers of the Python reference (including the scipy.interpolate.BSpline values),
// so both languages are checked against the same values.
//
// Plot data: when your functions return results of the right size, the tests also write CSV files into the folder
// you run lab_tests from (lab5_cpp_scaling_cubic.csv, lab5_cpp_scaling_quintic.csv, lab5_cpp_scaling_trapezoid.csv,
// lab5_cpp_fig_9_7a.csv, lab5_cpp_bsplines.csv, lab5_cpp_tau_AB.csv); plot them with  python cpp/plot_results.py
#include "lab.hpp"

#include <sbeg/labio.hpp>
#include <sbeg/mrcpp.hpp>

#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using sbeg::json;

namespace {
constexpr double PI = 3.14159265358979323846;

const json& cases() {
    static const json C = sbeg::load_json(sbeg::lab_path("cpp_cases.json"));
    return C;
}

/// The course arm with payload, derated (Python fixture `arm`), loaded with the given helper.
const lab::Robot& arm() {
    static const lab::Robot R = lab::load_course_arm(true, true);
    return R;
}

/// The task configurations (Python fixture `tasks`), loaded with the given helper.
const lab::Tasks& tasks() {
    static const lab::Tasks T = lab::load_tasks();
    return T;
}

VectorXd linspace(double a, double b, int n) { return VectorXd::LinSpaced(n, a, b); }

/// np.gradient(F, t, axis=0): second-order central differences inside, first-order differences at the ends.
MatrixXd gradient(const MatrixXd& F, const VectorXd& t) {
    const Eigen::Index n = F.rows();
    if (t.size() != n || n < 2) throw std::invalid_argument("gradient: need len(t) == rows >= 2");
    MatrixXd G(n, F.cols());
    G.row(0) = (F.row(1) - F.row(0)) / (t(1) - t(0));
    G.row(n - 1) = (F.row(n - 1) - F.row(n - 2)) / (t(n - 1) - t(n - 2));
    for (Eigen::Index i = 1; i + 1 < n; ++i) {
        const double hs = t(i) - t(i - 1), hd = t(i + 1) - t(i);
        G.row(i) = (hs * hs * F.row(i + 1) + (hd * hd - hs * hs) * F.row(i) - hd * hd * F.row(i - 1)) /
                   (hs * hd * (hd + hs));
    }
    return G;
}

/// np.allclose(got, expected, rtol, atol): |got - expected| <= atol + rtol |expected| entry by entry.
sbeg::NearResult allclose(const MatrixXd& got_in, const MatrixXd& exp_in, double atol, double rtol = 1e-5) {
    sbeg::NearResult r = sbeg::compare_near(got_in, exp_in, std::numeric_limits<double>::infinity());
    if (!r.ok) return r;  // shape mismatch or NaN/inf (message already set)
    MatrixXd got = got_in, exp = exp_in;
    if (got.rows() != exp.rows()) {  // a row and a column vector of the same length
        got = Eigen::Map<VectorXd>(got.data(), got.size()).eval();
        exp = Eigen::Map<VectorXd>(exp.data(), exp.size()).eval();
    }
    if (got.size() == 0) return r;
    const MatrixXd excess = (got - exp).cwiseAbs() - rtol * exp.cwiseAbs();
    Eigen::Index i = 0, k = 0;
    const double worst = excess.maxCoeff(&i, &k);
    r.ok = worst <= atol;
    std::ostringstream s;
    s << std::setprecision(12) << "worst entry (" << i << "," << k << "): got " << got(i, k) << ", expected "
      << exp(i, k) << " (atol " << atol << ", rtol " << rtol << ")";
    r.msg = s.str();
    return r;
}

/// np.isclose(got, expected, rtol, atol) for scalars.
sbeg::NearResult isclose(double got, double expected, double rtol = 1e-5, double atol = 1e-8) {
    return sbeg::compare_near(got, expected, atol + rtol * std::abs(expected));
}

/// Reference minimum duration by bisection (test_lab.py: _bisect_min_T), with mrcpp's inverse dynamics.
double bisect_min_T(const VectorXd& q0, const VectorXd& q1, int method, int N = 201) {
    const lab::Robot& R = arm();
    auto ok = [&](double T) {
        const lab::Trajectory tr = lab::joint_trajectory(q0, q1, T, N, method);
        const MatrixXd Tau = mr::inverse_dynamics_trajectory(tr.Q, tr.Qd, tr.Qdd, R.g, MatrixXd::Zero(N, 6), R.Mlist,
                                                             R.Glist, R.Slist);
        for (Eigen::Index k = 0; k < tr.Qd.rows(); ++k)
            for (Eigen::Index i = 0; i < 6; ++i)
                if (std::abs(tr.Qd(k, i)) > R.qdot_max(i) + 1e-12 || std::abs(Tau(k, i)) > R.tau_max(i) + 1e-12)
                    return false;
        return true;
    };
    double lo = 0.01, hi = 10.0;
    for (int it = 0; it < 40; ++it) {
        const double mid = 0.5 * (lo + hi);
        if (ok(mid)) hi = mid;
        else lo = mid;
    }
    return hi;
}

/// Plot data for plot_results.py: t, s, sdot, sddot.
void write_scaling_csv(const std::string& name, const VectorXd& t, const lab::TimeScaling& r) {
    if (r.s.size() != t.size() || r.sd.size() != t.size() || r.sdd.size() != t.size()) return;
    MatrixXd out(t.size(), 4);
    out << t, r.s, r.sd, r.sdd;
    sbeg::write_csv(name, {"t", "s", "sd", "sdd"}, out);
}
}  // namespace

#define CHECK_ALLCLOSE(got, expected, atol)                                                          \
    do {                                                                                             \
        const ::sbeg::NearResult lab_r_ = allclose((got), (expected), (atol));                        \
        CHECK_MESSAGE(lab_r_.ok, "allclose(" #got ", " #expected ")\n  " << lab_r_.msg);             \
    } while (false)

#define CHECK_ISCLOSE(got, expected)                                                                 \
    do {                                                                                             \
        const ::sbeg::NearResult lab_r_ = isclose((got), (expected));                                 \
        CHECK_MESSAGE(lab_r_.ok, "isclose(" #got ", " #expected ")\n  " << lab_r_.msg);              \
    } while (false)

#define CHECK_ISCLOSE_TOL(got, expected, rtol, atol)                                                 \
    do {                                                                                             \
        const ::sbeg::NearResult lab_r_ = isclose((got), (expected), (rtol), (atol));                 \
        CHECK_MESSAGE(lab_r_.ok, "isclose(" #got ", " #expected ")\n  " << lab_r_.msg);              \
    } while (false)

// ---------------------------------------------------------------- Part A: point-to-point time scalings

TEST_CASE("test_cubic_matches_modern_robotics_and_mr_fig_9_3") {
    for (const auto& c : cases()["cubic_matches_modern_robotics_and_mr_fig_9_3"]) {
        const double T = c["T"].get<double>();
        INFO("T = " << T);
        const VectorXd t = linspace(0.0, T, 41);
        const lab::TimeScaling r = lab::cubic_time_scaling(T, t);
        REQUIRE(r.s.size() == 41);
        REQUIRE(r.sd.size() == 41);
        REQUIRE(r.sdd.size() == 41);
        VectorXd s_mr(41);
        for (Eigen::Index k = 0; k < 41; ++k) s_mr(k) = mr::cubic_time_scaling(T, t(k));
        CHECK_ALLCLOSE(r.s, s_mr, 1e-12);
        CHECK_ALLCLOSE(r.s, sbeg::as_vec(c["s_mr"]), 1e-12);
        CHECK_ISCLOSE_TOL(r.sd.maxCoeff(), 3.0 / (2.0 * T), 1e-5, 1e-9);  // MR Fig. 9.3: peak 3/(2T) at T/2
        CHECK_ISCLOSE(r.sdd(0), 6.0 / (T * T));
        CHECK_ISCLOSE(r.sdd(40), -6.0 / (T * T));
        CHECK_ALLCLOSE(gradient(r.s, t).middleRows(1, 39), r.sd.segment(1, 39), 0.02 / T);
        if (T == 2.0) {
            const VectorXd t1 = linspace(0.0, 1.0, 201);
            write_scaling_csv("lab5_cpp_scaling_cubic.csv", t1, lab::cubic_time_scaling(1.0, t1));
        }
    }
}

TEST_CASE("test_quintic_matches_modern_robotics_and_mr_fig_9_4") {
    for (const auto& c : cases()["quintic_matches_modern_robotics_and_mr_fig_9_4"]) {
        const double T = c["T"].get<double>();
        INFO("T = " << T);
        const VectorXd t = linspace(0.0, T, 2001);
        const lab::TimeScaling r = lab::quintic_time_scaling(T, t);
        REQUIRE(r.s.size() == 2001);
        REQUIRE(r.sd.size() == 2001);
        REQUIRE(r.sdd.size() == 2001);
        VectorXd s_mr(2001);
        for (Eigen::Index k = 0; k < 2001; ++k) s_mr(k) = mr::quintic_time_scaling(T, t(k));
        CHECK_ALLCLOSE(r.s, s_mr, 1e-12);
        const VectorXd tc = sbeg::as_vec(c["t"]);
        CHECK_ALLCLOSE(lab::quintic_time_scaling(T, tc).s, sbeg::as_vec(c["s_mr"]), 1e-12);
        CHECK_ISCLOSE_TOL(r.sd.maxCoeff(), 15.0 / (8.0 * T), 1e-6, 1e-8);  // MR Fig. 9.4
        CHECK_ISCLOSE_TOL(r.sdd.maxCoeff(), 10.0 / (std::sqrt(3.0) * T * T), 1e-5, 1e-8);
        CHECK(std::abs(r.sd(0)) < 1e-12);
        CHECK(std::abs(r.sdd(0)) < 1e-12);
        CHECK(std::abs(r.sdd(2000)) < 1e-9);
        CHECK_ALLCLOSE(gradient(r.sd, t).middleRows(1, 1999), r.sdd.segment(1, 1999), 0.01 / (T * T));
        if (T == 2.0) {
            const VectorXd t1 = linspace(0.0, 1.0, 201);
            write_scaling_csv("lab5_cpp_scaling_quintic.csv", t1, lab::quintic_time_scaling(1.0, t1));
        }
    }
}

TEST_CASE("test_time_scalings_accept_scalars_and_clip") {
    const json& c = cases()["time_scalings_accept_scalars_and_clip"];
    {
        INFO("cubic_time_scaling");
        const lab::TimeScalingPoint p = lab::cubic_time_scaling(2.0, 1.0);
        CHECK_ISCLOSE(p.s, 0.5);
        CHECK_ISCLOSE(lab::cubic_time_scaling(2.0, 3.0).s, 1.0);
        CHECK_ISCLOSE(lab::cubic_time_scaling(2.0, -1.0).s, 0.0);
        CHECK_ALLCLOSE(sbeg::as_vec(c["cubic"]),
                       Eigen::Vector3d(p.s, lab::cubic_time_scaling(2.0, 3.0).s, lab::cubic_time_scaling(2.0, -1.0).s),
                       1e-12);
    }
    {
        INFO("quintic_time_scaling");
        const lab::TimeScalingPoint p = lab::quintic_time_scaling(2.0, 1.0);
        CHECK_ISCLOSE(p.s, 0.5);
        CHECK_ISCLOSE(lab::quintic_time_scaling(2.0, 3.0).s, 1.0);
        CHECK_ISCLOSE(lab::quintic_time_scaling(2.0, -1.0).s, 0.0);
        CHECK_ALLCLOSE(sbeg::as_vec(c["quintic"]),
                       Eigen::Vector3d(p.s, lab::quintic_time_scaling(2.0, 3.0).s,
                                       lab::quintic_time_scaling(2.0, -1.0).s),
                       1e-12);
    }
}

TEST_CASE("test_trapezoid_three_stage_mr_eqs_9_16_to_9_24") {
    const double v = 0.5, a = 1.25;  // v^2/a = 0.2 <= 1: three stages
    const lab::TrapezoidParams p = lab::trapezoid_params(v, a);
    CHECK_ISCLOSE(p.T, (a + v * v) / (v * a));
    CHECK_ISCLOSE(p.T, 2.4);
    CHECK_ISCLOSE(p.t_a, 0.4);
    CHECK_ISCLOSE(p.v_peak, 0.5);
    const json& c = cases()["trapezoid_three_stage_mr_eqs_9_16_to_9_24"];
    CHECK_ALLCLOSE(Eigen::Vector3d(p.T, p.t_a, p.v_peak),
                   Eigen::Vector3d(c["T"].get<double>(), c["t_a"].get<double>(), c["v_peak"].get<double>()), 1e-12);
    const VectorXd t = linspace(0.0, p.T, 4801);
    const lab::TimeScaling r = lab::trapezoid_time_scaling(v, a, t);
    REQUIRE(r.s.size() == 4801);
    REQUIRE(r.sd.size() == 4801);
    REQUIRE(r.sdd.size() == 4801);
    CHECK_ISCLOSE(r.s(4800), 1.0);
    CHECK_ISCLOSE(r.s(0), 0.0);
    CHECK_ISCLOSE(r.sd.maxCoeff(), v);
    CHECK_ISCLOSE(r.sdd.maxCoeff(), a);
    CHECK_ISCLOSE(r.sdd.minCoeff(), -a);
    const VectorXd ds = r.s.tail(4800) - r.s.head(4800);
    CHECK_MESSAGE(ds.minCoeff() >= -1e-12, "s must not decrease; min diff(s) = " << ds.minCoeff());
    CHECK_ALLCLOSE(gradient(r.s, t), r.sd, 2e-3);
    Eigen::Index k = 0;  // np.searchsorted(t, 1.0): cruise phase, MR Eq. 9.21
    while (k < t.size() && t(k) < 1.0) ++k;
    CHECK_ISCLOSE(r.s(k), v * t(k) - v * v / (2.0 * a));
    // plot data (Step 1): v = 1.5, a = 4.5 gives T = 1 s
    const VectorXd t1 = linspace(0.0, lab::trapezoid_params(1.5, 4.5).T, 201);
    write_scaling_csv("lab5_cpp_scaling_trapezoid.csv", t1, lab::trapezoid_time_scaling(1.5, 4.5, t1));
}

TEST_CASE("test_trapezoid_bang_bang_when_v2_over_a_exceeds_1") {
    const double v = 2.0, a = 1.0;  // v^2/a = 4 > 1 (MR Ex. 9.10, 9.13)
    const lab::TrapezoidParams p = lab::trapezoid_params(v, a);
    CHECK_ISCLOSE(p.T, 2.0);
    CHECK_ISCLOSE(p.t_a, 1.0);
    CHECK_ISCLOSE(p.v_peak, 1.0);
    const lab::TimeScaling r = lab::trapezoid_time_scaling(v, a, linspace(0.0, p.T, 1001));
    REQUIRE(r.s.size() == 1001);
    CHECK_ISCLOSE(r.s(1000), 1.0);
    CHECK_ISCLOSE(r.sd.maxCoeff(), 1.0);
}

TEST_CASE("test_trapezoid_from_limits_mr_exercise_9_4_data") {
    // MR Exercise 9.4 data: theta (0,0) -> (pi, pi/3), |qd| <= 2, |qdd| <= 0.5 -> bang-bang
    const lab::TrapezoidLimits r = lab::trapezoid_from_limits(Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(PI, PI / 3.0),
                                                              Eigen::Vector2d(2.0, 2.0), Eigen::Vector2d(0.5, 0.5));
    CHECK_ISCLOSE(r.v, 2.0 / PI);
    CHECK_ISCLOSE(r.a, 0.5 / PI);
    CHECK_ISCLOSE(r.T, 2.0 * std::sqrt(PI / 0.5));
    // a joint that does not move imposes no limit
    const lab::TrapezoidLimits r2 = lab::trapezoid_from_limits(
        Eigen::Vector2d(0.0, 1.0), Eigen::Vector2d(1.0, 1.0), Eigen::Vector2d(1.0, 1e-9), Eigen::Vector2d(1.0, 1e-9));
    CHECK_ISCLOSE(r2.v, 1.0);
    CHECK_ISCLOSE(r2.a, 1.0);
    const json& c = cases()["trapezoid_from_limits_mr_exercise_9_4_data"];
    CHECK_ISCLOSE(r.T, c["ex_9_4"]["T"].get<double>());
    CHECK_ISCLOSE(r2.T, c["still_joint"]["T"].get<double>());
}

TEST_CASE("test_joint_trajectory_matches_modern_robotics") {
    for (const auto& c : cases()["joint_trajectory_matches_modern_robotics"]) {
        const int method = c["method"].get<int>();
        INFO("method = " << method);
        const VectorXd q0 = sbeg::as_vec(c["q0"]), q1 = sbeg::as_vec(c["q1"]);
        const lab::Trajectory r = lab::joint_trajectory(q0, q1, 1.7, 35, method);
        REQUIRE(r.t.size() == 35);
        REQUIRE((r.Q.rows() == 35 && r.Q.cols() == 6));
        REQUIRE((r.Qd.rows() == 35 && r.Qd.cols() == 6));
        REQUIRE((r.Qdd.rows() == 35 && r.Qdd.cols() == 6));
        CHECK_ALLCLOSE(r.t, linspace(0.0, 1.7, 35), 1e-8);
        CHECK_ALLCLOSE(r.Q, mr::joint_trajectory(q0, q1, 1.7, 35, method), 1e-12);
        CHECK_ALLCLOSE(r.Q, sbeg::as_mat(c["Q_mr"]), 1e-12);
        const double dtm = r.t(1) - r.t(0);
        const VectorXd tt = VectorXd::LinSpaced(35, 0.0, 34.0) * dtm;  // np.gradient(Q, dtm, axis=0)
        CHECK_ALLCLOSE(gradient(r.Q, tt).middleRows(2, 31), r.Qd.middleRows(2, 31), 0.05);
    }
}

TEST_CASE("test_joint_trajectory_trapezoid_respects_limits") {
    const VectorXd& q0 = tasks().A;
    const VectorXd& q1 = tasks().B;
    const VectorXd vlim = VectorXd::Constant(6, 1.3962634), alim = tasks().qddot_lim;
    const lab::TrapezoidLimits L = lab::trapezoid_from_limits(q0, q1, vlim, alim);
    const lab::Trajectory r = lab::joint_trajectory(q0, q1, 0.0, 501, lab::TRAP, L.v, L.a);  // T is ignored
    REQUIRE(r.t.size() == 501);
    REQUIRE((r.Q.rows() == 501 && r.Q.cols() == 6));
    REQUIRE((r.Qd.rows() == 501 && r.Qd.cols() == 6));
    REQUIRE((r.Qdd.rows() == 501 && r.Qdd.cols() == 6));
    CHECK_ISCLOSE(r.t(500), L.T);
    CHECK_ALLCLOSE(r.Q.row(500), q1, 1e-8);
    CHECK_ALLCLOSE(r.Q.row(0), q0, 1e-8);
    for (Eigen::Index i = 0; i < 6; ++i) {
        INFO("joint " << i + 1);
        CHECK(r.Qd.col(i).cwiseAbs().maxCoeff() <= vlim(i) + 1e-9);
        CHECK(r.Qdd.col(i).cwiseAbs().maxCoeff() <= alim(i) + 1e-9);
    }
    CHECK_ISCLOSE(r.Qd.col(0).cwiseAbs().maxCoeff(), vlim(0));  // joint 1 saturates (largest move)
    const json& c = cases()["joint_trajectory_trapezoid_respects_limits"];
    CHECK_ALLCLOSE(Eigen::Vector3d(L.v, L.a, L.T),
                   Eigen::Vector3d(c["v"].get<double>(), c["a"].get<double>(), c["T"].get<double>()), 1e-12);
}

// ---------------------------------------------------------------- Part B: via points and B-splines

TEST_CASE("test_via_cubic_coefficients_formula_and_continuity") {
    const Eigen::Vector4d beta(0.0, 1.0, 0.5, 2.0), bdot(0.0, 0.8, -0.3, 0.0), Tm(0.0, 1.0, 2.5, 3.0);
    const MatrixXd A = lab::via_cubic_coefficients(beta, bdot, Tm);
    REQUIRE((A.rows() == 3 && A.cols() == 4));
    for (Eigen::Index j = 0; j < 3; ++j) {  // each segment meets its four constraints
        INFO("segment j = " << j);
        const double dT = Tm(j + 1) - Tm(j);
        const double a0 = A(j, 0), a1 = A(j, 1), a2 = A(j, 2), a3 = A(j, 3);
        CHECK_ISCLOSE(a0, beta(j));
        CHECK_ISCLOSE(a1, bdot(j));
        CHECK_ISCLOSE(a0 + a1 * dT + a2 * dT * dT + a3 * dT * dT * dT, beta(j + 1));
        CHECK_ISCLOSE(a1 + 2.0 * a2 * dT + 3.0 * a3 * dT * dT, bdot(j + 1));
    }
    CHECK_ALLCLOSE(A, sbeg::as_mat(cases()["via_cubic_coefficients_formula_and_continuity"]["A"]), 1e-12);
}

TEST_CASE("test_via_trajectory_mr_figure_9_7a") {
    // MR p. 337: vias (0,0), (0,1), (1,1), (1,0) at T = 0, 1, 2, 3, velocities (0,0), (1,0), (0,-1), (0,0)
    MatrixXd P(4, 2), V(4, 2);
    P << 0, 0,  0, 1,  1, 1,  1, 0;
    V << 0, 0,  1, 0,  0, -1,  0, 0;
    const Eigen::Vector4d Tm(0.0, 1.0, 2.0, 3.0);
    const lab::ViaSamples r = lab::via_trajectory(P, V, Tm, Tm);
    CHECK_ALLCLOSE(r.B, P, 1e-8);
    CHECK_ALLCLOSE(r.Bd, V, 1e-8);
    const VectorXd t = linspace(0.0, 3.0, 3001);
    const lab::ViaSamples f = lab::via_trajectory(P, V, Tm, t);
    REQUIRE((f.B.rows() == 3001 && f.B.cols() == 2));
    REQUIRE((f.Bd.rows() == 3001 && f.Bd.cols() == 2));
    CHECK_ALLCLOSE(gradient(f.B, t).middleRows(1, 2999), f.Bd.middleRows(1, 2999), 2e-3);
    CHECK(f.B.col(0).minCoeff() < -0.05);  // x dips below 0 in segment 1 (Fig. 9.7a): not in the convex hull
    CHECK_ISCLOSE(f.B.col(0).minCoeff(), cases()["via_trajectory_mr_figure_9_7a"]["x_min"].get<double>());
    MatrixXd out(3001, 3);  // plot data (Step 3)
    out << t, f.B;
    sbeg::write_csv("lab5_cpp_fig_9_7a.csv", {"t", "x", "y"}, out);
}

TEST_CASE("test_heuristic_via_velocities") {
    MatrixXd P(4, 2);
    P << 0.0, 0.0,  0.10, 0.20,  0.30, 0.20,  0.40, 0.0;
    const MatrixXd V = lab::heuristic_via_velocities(P, Eigen::Vector4d(0.0, 1.0, 2.0, 3.0));
    MatrixXd expected(4, 2);
    expected << 0, 0,  0.15, 0.0,  0.15, 0.0,  0, 0;  // flat side -> 0
    CHECK_ALLCLOSE(V, expected, 1e-8);
    const MatrixXd V2 = lab::heuristic_via_velocities(Eigen::Vector3d(0.0, 1.0, 0.0), Eigen::Vector3d(0.0, 1.0, 2.0));
    CHECK_ALLCLOSE(V2, MatrixXd::Zero(3, 1), 1e-8);  // turning point -> zero
}

TEST_CASE("test_bspline_basis_partition_of_unity") {
    const VectorXd u = linspace(0.0, 1.0, 11);
    const MatrixXd b = lab::bspline_basis(u);
    REQUIRE((b.rows() == 4 && b.cols() == 11));
    CHECK_ALLCLOSE(b.colwise().sum(), VectorXd::Ones(11), 1e-8);
    CHECK(b.minCoeff() >= -1e-15);
    CHECK_ALLCLOSE(lab::bspline_basis(0.0), Eigen::Vector4d(1.0 / 6.0, 4.0 / 6.0, 1.0 / 6.0, 0.0), 1e-8);
    CHECK_ALLCLOSE(b, sbeg::as_mat(cases()["bspline_basis_partition_of_unity"]["b"]), 1e-12);
}

TEST_CASE("test_uniform_cubic_bspline_matches_scipy_and_convex_hull") {
    const json& c = cases()["uniform_cubic_bspline_matches_scipy_and_convex_hull"];
    const MatrixXd P = sbeg::as_mat(c["P"]);  // drawn with the generator of test_lab.py
    const Eigen::Index n = P.rows();
    for (const auto& k : c["cases"]) {
        const double u = k["u"].get<double>();
        INFO("u = " << u);
        const lab::BSplinePoint r = lab::uniform_cubic_bspline(P, u);
        CHECK_ALLCLOSE(r.p, sbeg::as_vec(k["p"]), 1e-12);
        CHECK_ALLCLOSE(r.dp, sbeg::as_vec(k["dp"]), 1e-10);
        CHECK_ALLCLOSE(r.ddp, sbeg::as_vec(k["ddp"]), 1e-9);
        REQUIRE(r.p.size() == 2);
        const Eigen::Index i = std::min(static_cast<Eigen::Index>(std::floor(u)), n - 4);  // local convex hull
        const MatrixXd box = P.middleRows(i, 4);
        for (Eigen::Index d = 0; d < 2; ++d) {
            CHECK(r.p(d) >= box.col(d).minCoeff() - 1e-12);
            CHECK(r.p(d) <= box.col(d).maxCoeff() + 1e-12);
        }
    }
    // plot data (Step 3): the four vias of Fig. 9.7a as control points, open (one segment) and clamped
    MatrixXd ctrl(4, 2), clamped(10, 2);
    ctrl << 0, 0,  0, 1,  1, 1,  1, 0;
    clamped << 0, 0,  0, 0,  0, 0,  0, 0,  0, 1,  1, 1,  1, 0,  1, 0,  1, 0,  1, 0;
    MatrixXd out(201, 5);
    for (Eigen::Index k = 0; k < 201; ++k) {
        const double rr = static_cast<double>(k) / 200.0;
        const lab::BSplinePoint a = lab::uniform_cubic_bspline(ctrl, rr);
        const lab::BSplinePoint b = lab::uniform_cubic_bspline(clamped, 7.0 * rr);
        if (a.p.size() != 2 || b.p.size() != 2) return;
        out.row(k) << rr, a.p(0), a.p(1), b.p(0), b.p(1);
    }
    sbeg::write_csv("lab5_cpp_bsplines.csv", {"r", "x_open", "y_open", "x_clamped", "y_clamped"}, out);
}

// ---------------------------------------------------------------- Part C: torque feasibility on the course arm

TEST_CASE("test_course_arm_data") {
    const lab::Robot& R = arm();
    CHECK((R.Slist.rows() == 6 && R.Slist.cols() == 6));
    CHECK(R.Mlist.size() == 7);
    CHECK(R.Glist.size() == 6);
    VectorXd rated(6);
    rated << 1.8, 3.0, 1.5, 0.6, 0.6, 0.4;
    CHECK_ALLCLOSE(R.tau_max, 0.6 * rated, 1e-8);
    CHECK_ALLCLOSE(R.qdot_max, VectorXd::Constant(6, 0.5 * 160.0 * PI / 180.0), 1e-8);
    CHECK_ALLCLOSE(R.tau_max, sbeg::as_vec(cases()["course_arm_data"]["tau_max"]), 1e-12);
    // C++ only: the limits fit check_limits (one entry per joint; at rest every ratio is 0)
    const lab::LimitCheck out = lab::check_limits(R, MatrixXd::Zero(1, 6), MatrixXd::Zero(1, 6));
    CHECK(out.ok);
}

TEST_CASE("test_torque_profile_matches_modern_robotics") {
    const lab::Robot& R = arm();
    const lab::Trajectory tr = lab::joint_trajectory(tasks().A, tasks().B, 1.8, 25, 5);
    const MatrixXd Tau = lab::torque_profile(R, tr.Q, tr.Qd, tr.Qdd);
    const MatrixXd ref =
        mr::inverse_dynamics_trajectory(tr.Q, tr.Qd, tr.Qdd, R.g, MatrixXd::Zero(25, 6), R.Mlist, R.Glist, R.Slist);
    REQUIRE((Tau.rows() == 25 && Tau.cols() == 6));
    CHECK_ALLCLOSE(Tau, ref, 1e-10);
    CHECK_ALLCLOSE(Tau, sbeg::as_mat(cases()["torque_profile_matches_modern_robotics"]["Tau_mr"]), 1e-10);
}

TEST_CASE("test_check_limits") {
    const lab::Robot& R = arm();
    MatrixXd Qd = MatrixXd::Zero(3, 6), Tau = MatrixXd::Zero(3, 6);
    Qd(1, 2) = 0.5 * R.qdot_max(2);
    Tau(2, 3) = -1.5 * R.tau_max(3);
    const lab::LimitCheck out = lab::check_limits(R, Qd, Tau);
    REQUIRE(out.vel_ratio.size() == 6);
    REQUIRE(out.tau_ratio.size() == 6);
    CHECK_ISCLOSE(out.vel_ratio(2), 0.5);
    CHECK_ISCLOSE(out.tau_ratio(3), 1.5);
    CHECK_FALSE(out.ok);
    CHECK(lab::check_limits(R, Qd, MatrixXd::Zero(3, 6)).ok);
}

TEST_CASE("test_min_time_scale_equals_bisection") {
    for (const auto& c : cases()["min_time_scale_equals_bisection"]) {
        const std::string goal = c["goal"].get<std::string>(), binding = c["binding"][0].get<std::string>();
        const int method = c["method"].get<int>();
        INFO("goal = " << goal << ", method = " << method << ", expected binding = " << binding);
        const lab::Trajectory tr = lab::joint_trajectory(tasks().A, tasks().config(goal), 1.0, 201, method);
        const lab::TimeScaleResult res = lab::min_time_scale(arm(), tr.Q, tr.Qd, tr.Qdd);
        CHECK_ISCLOSE(res.c, std::max(res.c_vel, res.c_tau));
        CHECK(res.binding.first == binding);
        CHECK_ISCLOSE_TOL(res.c, bisect_min_T(tasks().A, tasks().config(goal), method), 1e-6, 1e-8);
        CHECK_ISCLOSE_TOL(res.c, c["c_bisect"].get<double>(), 1e-6, 1e-8);
    }
}

TEST_CASE("test_min_time_scale_checkpoint_values") {
    const lab::Trajectory ab = lab::joint_trajectory(tasks().A, tasks().B, 1.0, 401, 5);
    const lab::TimeScaleResult r = lab::min_time_scale(arm(), ab.Q, ab.Qd, ab.Qdd);
    CHECK_ISCLOSE_TOL(r.c_vel, 1.6114, 1e-5, 1e-3);
    CHECK(r.binding == std::make_pair(std::string("velocity"), 0));
    const lab::Trajectory ad = lab::joint_trajectory(tasks().A, tasks().D, 1.0, 401, 5);
    const lab::TimeScaleResult r2 = lab::min_time_scale(arm(), ad.Q, ad.Qd, ad.Qdd);
    CHECK_ISCLOSE_TOL(r2.c_tau, 0.2348, 1e-5, 1e-3);
    CHECK(r2.binding == std::make_pair(std::string("torque"), 3));  // joint 4!
    // plot data (Step 5, Q4): tau_i / tau_max_i along the A -> B quintic at T = T_min
    if (!(r.c > 0.0) || !std::isfinite(r.c)) return;
    const lab::Trajectory tm = lab::joint_trajectory(tasks().A, tasks().B, r.c, 401, 5);
    const MatrixXd Tau = lab::torque_profile(arm(), tm.Q, tm.Qd, tm.Qdd);
    if (Tau.rows() != 401 || Tau.cols() != 6 || tm.t.size() != 401) return;
    MatrixXd out(401, 7);
    out << tm.t, Tau.array().rowwise() / arm().tau_max.transpose().array();
    sbeg::write_csv("lab5_cpp_tau_AB.csv", {"t", "r1", "r2", "r3", "r4", "r5", "r6"}, out);
}

TEST_CASE("test_min_time_scale_rejects_static_overload") {
    lab::Robot heavy = arm();
    heavy.tau_max = 0.1 * arm().tau_max;  // cannot even hold the arm against gravity
    const lab::Trajectory tr = lab::joint_trajectory(tasks().A, tasks().B, 1.0, 21, 5);
    CHECK_THROWS_AS(lab::min_time_scale(heavy, tr.Q, tr.Qd, tr.Qdd), std::invalid_argument);
}

TEST_CASE("test_via_trajectory_on_course_arm_is_feasible_and_stretch_works") {
    const lab::Robot& R = arm();
    MatrixXd P(3, 6);
    P << tasks().A.transpose(), tasks().W.transpose(), tasks().B.transpose();
    const VectorXd& Tm = tasks().via_times;
    const MatrixXd V = lab::heuristic_via_velocities(P, Tm);
    CHECK_ALLCLOSE(V, sbeg::as_mat(cases()["via_trajectory_on_course_arm_is_feasible_and_stretch_works"]["V"]), 1e-12);
    const VectorXd t = linspace(Tm(0), Tm(Tm.size() - 1), 481);
    const lab::ViaSamples B = lab::via_trajectory(P, V, Tm, t);
    const lab::TimeScaleResult res = lab::min_time_scale(R, B.B, B.Bd, B.Bdd);
    CHECK(res.c < 1.0);  // the given timing is feasible
    const double c = res.c;  // stretched trajectory touches a limit
    const MatrixXd Qd = B.Bd / c, Qdd = B.Bdd / (c * c);
    const MatrixXd Tau = lab::torque_profile(R, B.B, Qd, Qdd);
    const lab::LimitCheck out = lab::check_limits(R, Qd, Tau);
    REQUIRE(out.vel_ratio.size() == 6);
    REQUIRE(out.tau_ratio.size() == 6);
    CHECK(out.ok);
    CHECK_ISCLOSE_TOL(std::max(out.vel_ratio.maxCoeff(), out.tau_ratio.maxCoeff()), 1.0, 1e-5, 1e-9);
}
