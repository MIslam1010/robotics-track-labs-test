// Auto-checks for SBEG 461 Lab 6 (time-optimal time scaling, TOPP vs quintic, hardware command file) -- C++ track.
// Mirrors test_lab.py test by test.
//
// Build and run from this folder:
//     cmake -S . -B build -G Ninja && cmake --build build && ./build/lab_tests
//     ./build/lab_tests -tc="*path_dynamics*,*accel_limits*"      (run only some tests, like pytest -k)
// Ground truth: mrcpp (the C++ port of modern_robotics: RNEA), the closed-form MR 2R dynamics (MR Eqs. 8.9-8.10),
// closed-form phase-plane results of Workshop 6 (elbow exoskeleton, demining gantry), and cpp_cases.json, which
// holds the numbers of the Python reference (modern_robotics + toppra), so both languages are checked against the
// same values.
#include "lab.hpp"

#include <sbeg/labio.hpp>
#include <sbeg/mrcpp.hpp>

#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

using Eigen::Matrix2d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::VectorXd;
using sbeg::as;
using sbeg::as_mat;
using sbeg::as_vec;
using sbeg::json;
namespace CA = course_arm;

namespace {
constexpr double PI = 3.14159265358979323846;

const json& cases() {
    static const json C = sbeg::load_json(sbeg::lab_path("cpp_cases.json"));
    return C;
}

VectorXd vec(std::initializer_list<double> v) {
    VectorXd out(static_cast<Eigen::Index>(v.size()));
    Eigen::Index i = 0;
    for (double x : v) out(i++) = x;
    return out;
}

const VectorXd TAU_2R = vec({20.0, 10.0});
const VectorXd F_GANTRY = vec({120.0, 60.0});

// The fixtures of test_lab.py (rebuilt in every test case: they are cheap in C++).
struct Setup {
    lab::Model model;
    lab::Path path;
};
Setup two_r() { return {lab::mr_2r_model(), lab::straight_line_path(vec({0.0, 0.0}), vec({PI / 2, PI / 2}))}; }
Setup gantry() { return {lab::gantry_model(), lab::semicircle_path(0.3)}; }
Setup elbow() { return {lab::elbow_exo_model(), lab::straight_line_path(vec({0.0}), vec({2 * PI / 3}))}; }

struct ArmTask {
    lab::Task task;
    lab::Model r;
    lab::Path path;
    VectorXd tl, vmax, amax;  // 60 % torque, 50 % speed, |qddot| <= 4 rad/s^2 (the hw limit set)
};
ArmTask arm_task() {
    ArmTask a;
    a.task = lab::load_task();
    a.r = lab::course_arm_model(a.task.payload_kg);
    a.path = lab::waypoint_path(a.task.waypoints);
    a.tl = a.task.torque_fraction * a.r.tau_max;
    a.vmax = a.task.hw_speed_fraction * a.r.qdot_max;
    a.amax = VectorXd::Constant(a.r.n, a.task.hw_qddot_max);
    return a;
}
lab::ToppResult hw_topp(const ArmTask& a) { return lab::topp(a.r, a.path, a.tl, a.vmax, a.amax, a.task.grid_N); }

Matrix2d M2(const Vector2d& th) {
    const double c2 = std::cos(th(1));
    Matrix2d M;
    M << 3 + 2 * c2, 1 + c2, 1 + c2, 1.0;
    return M;
}

Vector2d c2r(const Vector2d& th, const Vector2d& dth) {
    const double s2 = std::sin(th(1));
    return Vector2d(-s2 * (2 * dth(0) * dth(1) + dth(1) * dth(1)), s2 * dth(0) * dth(0));
}

/// np.interp for one value (xp increasing).
double interp1(double x, const VectorXd& xp, const VectorXd& fp) {
    if (x <= xp(0)) return fp(0);
    for (Eigen::Index i = 1; i < xp.size(); ++i)
        if (x <= xp(i)) return fp(i - 1) + (fp(i) - fp(i - 1)) * (x - xp(i - 1)) / (xp(i) - xp(i - 1));
    return fp(fp.size() - 1);
}

/// What f() throws, as text: "std::runtime_error" for the velocity-limit-curve signal of bang_bang; an Eigen size /
/// index error (sbeg::EigenError, itself a std::runtime_error) or any other exception is reported with its message.
/// (As test_lab.py, which lets NotImplementedError through and accepts only RuntimeError.)
template <class F>
std::string thrown_kind(F&& f) {
    try {
        f();
    } catch (const sbeg::EigenError& e) {
        return std::string("Eigen size/index error: ") + e.what();
    } catch (const std::runtime_error&) {
        return "std::runtime_error";
    } catch (const std::exception& e) {
        return std::string("other exception: ") + e.what();
    }
    return "no exception";
}

void check_command_dict(const lab::CommandCheck& got, const json& exp) {
    CHECK(got.ok == exp["ok"].get<bool>());
    CHECK(got.within_limits == exp["within_limits"].get<bool>());
    CHECK(got.speed_ok == exp["speed_ok"].get<bool>());
    CHECK(got.starts_at_rest == exp["starts_at_rest"].get<bool>());
    CHECK(got.ends_at_rest == exp["ends_at_rest"].get<bool>());
    CHECK_SCALAR_NEAR(got.peak_speed_ratio, exp["peak_speed_ratio"].get<double>(), 1e-6);
}
}  // namespace

// ------------------------------------------------------------------ Part A: path dynamics, cone, VLC

TEST_CASE("test_path_dynamics_2r_closed_form") {
    // Straight line: theta'' = 0, so m = M(theta) Delta and c = c(theta, Delta) (MR Eqs. 9.32, 8.9-8.10).
    const Setup t = two_r();
    for (const auto& c : cases()["path_dynamics_2r_closed_form"]) {
        const double s = c["s"].get<double>();
        INFO("s = " << s);
        const lab::PathDynamics pd = lab::path_dynamics(t.model, t.path, s);
        const Vector2d D(PI / 2, PI / 2);
        const Vector2d th = s * D;
        CHECK_MAT_NEAR(pd.m, M2(th) * D, 1e-9);
        CHECK_MAT_NEAR(pd.c, c2r(th, D), 1e-9);
        CHECK_MAT_NEAR(pd.g, Vector2d::Zero(), 1e-12);
        CHECK_MAT_NEAR(pd.m, as_vec(c["m"]), 1e-10);
        CHECK_MAT_NEAR(pd.c, as_vec(c["c"]), 1e-10);
    }
}

TEST_CASE("test_path_dynamics_reproduces_rnea_on_course_arm") {
    // m sdd + c sdot^2 + g = ID(theta, theta' sdot, theta' sdd + theta'' sdot^2) for any (sdot, sdd).
    const ArmTask a = arm_task();
    const json& C = cases()["path_dynamics_reproduces_rnea_on_course_arm"];
    // the given C++ course-arm model and spline path are the Python ones
    REQUIRE(a.r.Mlist.size() == 7);
    REQUIRE(a.r.Glist.size() == 6);
    CHECK_MAT_NEAR(a.r.Slist, as_mat(C["Slist"]), 1e-12);
    CHECK_MAT_NEAR(a.r.M, as_mat(C["M"]), 1e-12);
    for (std::size_t i = 0; i < 7; ++i) CHECK_MAT_NEAR(a.r.Mlist[i], as_mat(C["Mlist"][i]), 1e-12);
    for (std::size_t i = 0; i < 6; ++i) CHECK_MAT_NEAR(a.r.Glist[i], as_mat(C["Glist"][i]), 1e-12);
    CHECK_MAT_NEAR(a.r.limits, as_mat(C["limits"]), 1e-12);
    CHECK_MAT_NEAR(a.r.tau_max, as_vec(C["tau_max"]), 1e-12);
    CHECK_MAT_NEAR(a.r.qdot_max, as_vec(C["qdot_max"]), 1e-12);
    for (const auto& p : C["points"]) {
        const double s = p["s"].get<double>();
        INFO("s = " << s);
        const VectorXd th = a.path(s, 0), d1 = a.path(s, 1), d2 = a.path(s, 2);
        CHECK_MAT_NEAR(th, as_vec(p["theta"]), 1e-12);
        CHECK_MAT_NEAR(d1, as_vec(p["d1"]), 1e-12);
        CHECK_MAT_NEAR(d2, as_vec(p["d2"]), 1e-12);
        const lab::PathDynamics pd = lab::path_dynamics(a.r, a.path, s);
        for (const auto& d : p["draws"]) {
            const double sd = d["sd"].get<double>(), sdd = d["sdd"].get<double>();
            const VectorXd tau = lab::inverse_dynamics(a.r, th, d1 * sd, d1 * sdd + d2 * (sd * sd));
            CHECK_MAT_NEAR(pd.m * sdd + pd.c * (sd * sd) + pd.g, tau, 1e-10);
            CHECK_MAT_NEAR(tau, as_vec(d["tau"]), 1e-10);
        }
        CHECK_MAT_NEAR(pd.m, as_vec(p["m"]), 1e-10);
        CHECK_MAT_NEAR(pd.c, as_vec(p["c"]), 1e-10);
        CHECK_MAT_NEAR(pd.g, as_vec(p["g"]), 1e-10);
    }
}

TEST_CASE("test_accel_limits_quiz_state") {
    // Quiz 5 Q2: m = (2, -0.5), c = (0.4, 0.3), g = (1, 0), |tau| <= (5, 2), sdot = 2 -> L = -1.6, U = 1.2;
    // sdot = 3 is inadmissible (sdot_lim^2 = 7.5).
    const VectorXd m = vec({2.0, -0.5}), c = vec({0.4, 0.3}), g = vec({1.0, 0.0}), t = vec({5.0, 2.0});
    auto [L, U] = lab::accel_limits(m, c, g, 2.0, -t, t);
    CHECK_APPROX(L, -1.6);
    CHECK_APPROX(U, 1.2);
    std::tie(L, U) = lab::accel_limits(m, c, g, 3.0, -t, t);
    CHECK(L > U);
    std::tie(L, U) = lab::accel_limits(m, c, g, std::sqrt(7.5) - 1e-6, -t, t);
    CHECK(L <= U);
}

TEST_CASE("test_accel_limits_zero_inertia") {
    // A zero component of m(s) gives a pure velocity constraint (MR Sec. 9.4.4).
    const VectorXd m = vec({0.0, 2.0}), c = vec({1.0, 0.0}), g = vec({0.0, 0.0}), t = vec({1.0, 1.0});
    auto [L, U] = lab::accel_limits(m, c, g, 0.9, -t, t);
    CHECK_APPROX(L, -0.5);
    CHECK_APPROX(U, 0.5);
    std::tie(L, U) = lab::accel_limits(m, c, g, 1.1, -t, t);
    CHECK(L > U);
}

TEST_CASE("test_velocity_limit_2r_workshop") {
    // Workshop 6 Problem 4: at s = 0.5 the VLC is where the lower bound of joint 1 meets the upper bound
    // of joint 2: (-20 - c1 x)/m1 = (10 - c2 x)/m2, x = sdot^2.
    const Setup t = two_r();
    const lab::PathDynamics pd = lab::path_dynamics(t.model, t.path, 0.5);
    const double x = (10 / pd.m(1) + 20 / pd.m(0)) / (pd.c(1) / pd.m(1) - pd.c(0) / pd.m(0));
    const double vl = lab::velocity_limit(t.model, t.path, 0.5, -TAU_2R, TAU_2R);
    CHECK_SCALAR_NEAR(vl, std::sqrt(x), 1e-6);
    CHECK_SCALAR_NEAR(std::sqrt(x), 2.1546, 1e-4);
    CHECK_SCALAR_NEAR(vl, cases()["velocity_limit_2r_workshop"]["vl"].get<double>(), 1e-8);
}

TEST_CASE("test_velocity_limit_gantry_closed_form") {
    // Workshop 6 Problem 3 (MR Fig. 9.9 path): closed-form VLC values at s = 0, 0.25, 0.5.
    const Setup t = gantry();
    const double R = 0.3, Mx = 40.0, my = 15.0, fx = 120.0, fy = 60.0;
    const double a = std::sqrt(0.5);
    const double A1 = fx / (Mx * PI * R * a), A2 = fy / (my * PI * R * a);
    auto vl = [&](double s) { return lab::velocity_limit(t.model, t.path, s, -F_GANTRY, F_GANTRY); };
    CHECK_SCALAR_NEAR(vl(0.25), std::sqrt((A1 + A2) / (2 * PI)), 1e-6);
    CHECK_SCALAR_NEAR(vl(0.5), std::sqrt(fy / (my * PI * PI * R)), 1e-6);
    CHECK_SCALAR_NEAR(vl(0.0), std::sqrt(fx / (Mx * PI * PI * R)), 1e-6);
    const json& c = cases()["velocity_limit_gantry_closed_form"];
    for (std::size_t i = 0; i < c["s"].size(); ++i)
        CHECK_SCALAR_NEAR(vl(c["s"][i].get<double>()), c["vl"][i].get<double>(), 1e-8);
}

TEST_CASE("test_velocity_limit_kinematic") {
    const Setup t = two_r();
    const VectorXd big = vec({1e6, 1e6});
    const double v = lab::velocity_limit(t.model, t.path, 0.3, -big, big, vec({2.0, 2.0}));
    CHECK_SCALAR_NEAR(v, 2 / (PI / 2), 1e-6);
}

// ------------------------------------------------------------------ Part B: bang-bang and TOPP

TEST_CASE("test_bang_bang_elbow_closed_form") {
    // Workshop 6 Problem 2: s* = 1/2 + k(1 - cos D)/(2 tau D); T from the closed-form phase curves.
    const Setup t = elbow();
    const double I = 0.19734, k = 0.698 * 9.81, D = 2 * PI / 3, tm = 10.0;
    const lab::BangBang out = lab::bang_bang(t.model, t.path, vec({-tm}), vec({tm}), 2000);
    const double s_star = 0.5 + k * (1 - std::cos(D)) / (2 * tm * D);
    const double x_star = 2 / (I * D) * (tm * s_star - k / D * (1 - std::cos(D * s_star)));
    CHECK_SCALAR_NEAR(out.s_switch, s_star, 1e-4);
    CHECK_SCALAR_NEAR(out.sdot_switch, std::sqrt(x_star), 1e-4 * std::sqrt(x_star));  // rel = 1e-4
    CHECK_SCALAR_NEAR(out.T, 0.388528, 2e-3 * 0.388528);                              // rel = 2e-3
    const json& c = cases()["bang_bang_elbow_closed_form"];
    CHECK_SCALAR_NEAR(out.s_switch, c["s_switch"].get<double>(), 1e-7);
    CHECK_SCALAR_NEAR(out.sdot_switch, c["sdot_switch"].get<double>(), 1e-7);
    CHECK_SCALAR_NEAR(out.T, c["T"].get<double>(), 1e-7);
}

TEST_CASE("test_bang_bang_matches_toppra_2r") {
    const Setup t = two_r();
    const lab::BangBang bb = lab::bang_bang(t.model, t.path, -TAU_2R, TAU_2R, 2000);
    const lab::ToppResult tp = lab::topp(t.model, t.path, TAU_2R, VectorXd(), VectorXd(), 400);
    CHECK_SCALAR_NEAR(bb.T, 1.3683, 2e-3);
    CHECK_SCALAR_NEAR(tp.T, bb.T, 0.01 * bb.T);  // rel = 0.01
    const json& c = cases()["bang_bang_matches_toppra_2r"];
    CHECK_SCALAR_NEAR(bb.T, c["T_bang_bang"].get<double>(), 1e-7);
    CHECK_SCALAR_NEAR(tp.T, c["T_toppra"].get<double>(), 1e-6);  // the real toppra
}

TEST_CASE("test_bang_bang_detects_velocity_limit_curve") {
    // On the semicircle the forward curve hits the VLC: one switch is not enough (MR Fig. 9.12b).
    // bang_bang must throw std::runtime_error (an unfinished function throws std::logic_error: it fails).
    const Setup t = gantry();
    CHECK(thrown_kind([&] { lab::bang_bang(t.model, t.path, -F_GANTRY, F_GANTRY, 1000); }) == "std::runtime_error");
}

TEST_CASE("test_topp_gantry") {
    const Setup t = gantry();
    const lab::ToppResult tp = lab::topp(t.model, t.path, F_GANTRY, VectorXd(), VectorXd(), 400);
    CHECK_SCALAR_NEAR(tp.T, 1.1655, 3e-3);
    REQUIRE(tp.s_grid.size() == 401);
    REQUIRE(tp.sdot_grid.size() == 401);
    const double sd = interp1(0.5, tp.s_grid, tp.sdot_grid);
    CHECK(sd <= lab::velocity_limit(t.model, t.path, 0.5, -F_GANTRY, F_GANTRY) + 1e-3);
    const json& c = cases()["topp_gantry"];
    CHECK_SCALAR_NEAR(tp.T, c["T"].get<double>(), 1e-6);
    CHECK_MAT_NEAR(tp.sdot_grid, as_vec(c["sdot_grid"]), 1e-6);
}

TEST_CASE("test_topp_torque_scaling") {
    // g = 0 and pure torque limits: scaling the limits by k scales T by 1/sqrt(k) (Workshop 6 P5).
    const Setup t = two_r();
    const double T1 = lab::topp(t.model, t.path, TAU_2R, VectorXd(), VectorXd(), 300).T;
    const double T2 = lab::topp(t.model, t.path, 0.6 * TAU_2R, VectorXd(), VectorXd(), 300).T;
    CHECK_SCALAR_NEAR(T2, T1 / std::sqrt(0.6), 5e-3 * T1 / std::sqrt(0.6));  // rel = 5e-3
    const json& c = cases()["topp_torque_scaling"];
    CHECK_SCALAR_NEAR(T1, c["T1"].get<double>(), 1e-6);
    CHECK_SCALAR_NEAR(T2, c["T2"].get<double>(), 1e-6);
}

TEST_CASE("test_topp_course_arm_respects_limits") {
    const ArmTask a = arm_task();
    const lab::ToppResult hw = hw_topp(a);
    const lab::PeakUsage pu = lab::peak_usage(a.r, hw.traj, hw.T, a.tl, a.vmax, 2000);
    CHECK(pu.vel_ratio.maxCoeff() <= 1.01);
    CHECK(pu.tau_ratio.maxCoeff() <= 1.03);
    double qdd_peak = 0.0;
    for (int k = 0; k <= 2000; ++k) qdd_peak = std::max(qdd_peak, hw.traj(hw.T * k / 2000.0, 2).cwiseAbs().maxCoeff());
    CHECK(qdd_peak <= a.task.hw_qddot_max * 1.01);
    CHECK_SCALAR_NEAR(hw.T, 1.5656, 0.01);
    CHECK_MAT_NEAR(hw.traj(0.0, 0), a.task.waypoints.row(0), 1e-6);
    CHECK_MAT_NEAR(hw.traj(hw.T, 0), a.task.waypoints.row(a.task.waypoints.rows() - 1), 1e-6);
    const json& c = cases()["topp_course_arm_respects_limits"];
    CHECK_SCALAR_NEAR(hw.T, c["T"].get<double>(), 1e-6);
    CHECK_MAT_NEAR(hw.sdot_grid, as_vec(c["sdot_grid"]), 1e-6);
    CHECK_MAT_NEAR(pu.vel_ratio, as_vec(c["vel_ratio"]), 1e-5);
    CHECK_MAT_NEAR(pu.tau_ratio, as_vec(c["tau_ratio"]), 1e-5);
}

TEST_CASE("test_quintic_min_time_elbow") {
    // Torque-limited quintic: 0.5231 s; speed-limited (3 rad/s): T = (15/8) D / 3 = 1.3090 s.
    const Setup t = elbow();
    const double Tt = lab::quintic_min_time(t.model, t.path, vec({10.0}));
    CHECK_SCALAR_NEAR(Tt, 0.52312, 1e-3);
    const double Ts = lab::quintic_min_time(t.model, t.path, vec({10.0}), vec({3.0}));
    CHECK_SCALAR_NEAR(Ts, 15.0 / 8 * (2 * PI / 3) / 3, 1e-3);
    const json& c = cases()["quintic_min_time_elbow"];
    CHECK_SCALAR_NEAR(Tt, c["T_torque"].get<double>(), 2e-4);
    CHECK_SCALAR_NEAR(Ts, c["T_speed"].get<double>(), 2e-4);
}

TEST_CASE("test_topp_beats_quintic_on_course_arm") {
    const ArmTask a = arm_task();
    const lab::ToppResult hw = hw_topp(a);
    const double Tq = lab::quintic_min_time(a.r, a.path, a.tl, a.vmax, a.amax);
    CHECK_SCALAR_NEAR(Tq, 2.2964, 0.01);
    CHECK(1.2 < Tq / hw.T);
    CHECK(Tq / hw.T < 2.0);
    CHECK_SCALAR_NEAR(Tq, cases()["topp_beats_quintic_on_course_arm"]["T_quintic"].get<double>(), 2e-4);
}

// ------------------------------------------------------------------ Part C: hardware command file and metrics

TEST_CASE("test_command_table") {
    const ArmTask a = arm_task();
    const lab::ToppResult hw = hw_topp(a);
    const double dt = a.task.servo_period_s;
    const auto [t, q] = lab::command_table(hw.traj, hw.T, dt, 2.0);
    REQUIRE(t.size() >= 2);
    CHECK(t(0) == 0.0);
    CHECK_APPROX(t(t.size() - 1), 2 * hw.T);
    for (Eigen::Index k = 0; k + 1 < t.size(); ++k) {
        INFO("k = " << k);
        CHECK(t(k + 1) - t(k) <= dt + 1e-12);
        CHECK(t(k + 1) - t(k) > 0);
    }
    CHECK(q.rows() == t.size());
    CHECK(q.cols() == 6);
    REQUIRE(q.rows() == t.size());
    CHECK_MAT_NEAR(q.row(0), a.task.waypoints.row(0), 1e-9);
    CHECK_MAT_NEAR(q.row(q.rows() - 1), a.task.waypoints.row(a.task.waypoints.rows() - 1), 1e-9);
    const json& c = cases()["command_table"];
    CHECK(t.size() == c["n_samples"].get<Eigen::Index>());
    CHECK_SCALAR_NEAR(t(t.size() - 1), c["t_end"].get<double>(), 1e-6);
}

TEST_CASE("test_check_command") {
    const ArmTask a = arm_task();
    const lab::ToppResult hw = hw_topp(a);
    const double dt = a.task.servo_period_s;
    const auto [t, q] = lab::command_table(hw.traj, hw.T, dt, 2.0);
    const lab::CommandCheck ok = lab::check_command(t, q, a.r, a.task.first_run_speed_fraction, a.task.joint_margin_deg);
    CHECK(ok.ok);
    CHECK(ok.within_limits);
    CHECK(ok.starts_at_rest);
    CHECK(ok.ends_at_rest);
    CHECK(0.9 < ok.peak_speed_ratio);
    CHECK(ok.peak_speed_ratio <= 1.01);
    const auto [t1, q1] = lab::command_table(hw.traj, hw.T, dt, 1.0);
    const lab::CommandCheck bad =
        lab::check_command(t1, q1, a.r, a.task.first_run_speed_fraction, a.task.joint_margin_deg);
    CHECK_FALSE(bad.ok);
    CHECK(bad.peak_speed_ratio > 1.8);
    MatrixXd q2 = q;
    q2.col(1).array() += 157.0 * PI / 180.0 - q2.col(1).maxCoeff();  // push joint 2 to 8 deg from its end stop
    CHECK_FALSE(lab::check_command(t, q2, a.r, 1.0, a.task.joint_margin_deg).within_limits);
    CHECK_FALSE(lab::check_command(t, q2, a.r, 1.0).within_limits);  // default margin = 10 deg
    CHECK(lab::check_command(t, q2, a.r, 1.0, 5.0).within_limits);   // a 5 deg margin would let it pass
    check_command_dict(ok, cases()["check_command"]["first_run"]);
    check_command_dict(bad, cases()["check_command"]["unstretched"]);
}

TEST_CASE("test_task_follows_safety_checklist") {
    // The Week 6 hardware run uses the course-arm checklist values (Course resources).
    const lab::Task task = lab::load_task();
    const CA::Safety& s = CA::COURSE_ARM_SAFETY;
    CHECK(task.joint_margin_deg == s.soft_limit_margin_deg);
    CHECK(s.soft_limit_margin_deg == 10.0);
    CHECK(task.first_run_speed_fraction == s.speed_fraction_first_run);
    CHECK(s.speed_fraction_first_run == 0.25);
    CHECK(task.hw_speed_fraction == s.speed_fraction);
    CHECK(s.speed_fraction == 0.5);
    CHECK(task.payload_kg <= s.payload_max_kg);
    CHECK_APPROX(s.payload_max_kg, 0.125);
    CHECK(task.servo_period_s == 0.02);
    CHECK(task.hw_speed_fraction / task.first_run_speed_fraction == 2.0);  // stretch factor slow = 2
    // C++ only: check_command applies these values -- the arm resting at q_home for 1 s at the servo period passes,
    // and q_home pushed to 8 deg from an end stop fails with the checklist margin.
    const lab::Model r = lab::course_arm_model(task.payload_kg);
    VectorXd t(51);
    for (Eigen::Index k = 0; k < t.size(); ++k) t(k) = static_cast<double>(k) * task.servo_period_s;
    MatrixXd q = r.q_home.transpose().replicate(t.size(), 1);
    CHECK(lab::check_command(t, q, r, s.speed_fraction_first_run, s.soft_limit_margin_deg).ok);
    q.col(1).setConstant(-157.0 * PI / 180.0);
    CHECK_FALSE(lab::check_command(t, q, r, s.speed_fraction_first_run, s.soft_limit_margin_deg).ok);
}

TEST_CASE("test_tracking_metrics_synthetic") {
    const Eigen::Index N = 400;  // np.arange(0, 4, 0.01)
    VectorXd t(N);
    for (Eigen::Index k = 0; k < N; ++k) t(k) = static_cast<double>(k) * 0.01;
    MatrixXd cmd(N, 2), meas(N, 2);
    for (Eigen::Index k = 0; k < N; ++k) {
        cmd.row(k) << std::sin(t(k)), 0.5 * std::cos(2 * t(k));
        meas.row(k) << std::sin(t(k) - 0.06), 0.5 * std::cos(2 * (t(k) - 0.06));
    }
    const lab::TrackingMetrics out = lab::tracking_metrics(t, cmd, meas);
    CHECK_SCALAR_NEAR(out.lag, 0.06, 1e-9);
    const MatrixXd e = meas - cmd;
    const VectorXd rms = e.array().square().colwise().mean().sqrt().transpose();
    const VectorXd mx = e.cwiseAbs().colwise().maxCoeff().transpose();
    CHECK_MAT_NEAR(out.rms, rms, 1e-8);
    CHECK_MAT_NEAR(out.max_abs, mx, 1e-8);
    REQUIRE(out.rms_after_lag.size() == 2);
    CHECK(out.rms_after_lag.maxCoeff() < 1e-9);
    const json& c = cases()["tracking_metrics_synthetic"];
    CHECK_MAT_NEAR(out.rms, as_vec(c["rms"]), 1e-12);
    CHECK_MAT_NEAR(out.max_abs, as_vec(c["max_abs"]), 1e-12);
}

TEST_CASE("test_tracking_metrics_on_fallback_log") {
    const lab::Log log = lab::load_log(lab::data_path("sim_servo_log.csv"));
    const lab::TrackingMetrics out = lab::tracking_metrics(log.t, log.cmd, log.meas);
    CHECK(0.06 <= out.lag);
    CHECK(out.lag <= 0.12);  // ~ delay + 2 zeta / wn = 0.092 s
    REQUIRE(out.rms.size() == 6);
    REQUIRE(out.rms_after_lag.size() == 6);
    for (Eigen::Index j = 0; j < 6; ++j) CHECK(out.rms_after_lag(j) <= out.rms(j) + 1e-12);
    CHECK_SCALAR_NEAR(out.rms.maxCoeff(), 0.0419, 2e-3);
    const json& c = cases()["tracking_metrics_on_fallback_log"];
    CHECK_SCALAR_NEAR(out.lag, c["lag"].get<double>(), 1e-12);
    CHECK_MAT_NEAR(out.rms, as_vec(c["rms"]), 1e-12);
    CHECK_MAT_NEAR(out.max_abs, as_vec(c["max_abs"]), 1e-12);
    CHECK_MAT_NEAR(out.rms_after_lag, as_vec(c["rms_after_lag"]), 1e-12);
}
