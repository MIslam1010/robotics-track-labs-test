// Auto-checks for SBEG 461 Hardware rotation 1 (Week 8) -- C++ track: command file, dry run, servo identification,
// gravity check, sim-vs-hardware comparison, safe stop and the pre-flight record.  Mirrors test_lab.py test by
// test (16 test cases for the 18 Python tests: the two Python tests that check only the provided helpers --
// step_response against the servo simulator, and the joint-limit margin of check_command -- are checked inside
// test_step_features_synthetic and test_command_passes_first_run_check).
//
// Build and run from this folder:
//     cmake -S . -B build -G Ninja && cmake --build build && ./build/lab_tests
//     ./build/lab_tests -tc="*segment*,*waypoint*"      (run only some tests, like pytest -k)
// Ground truth: closed forms (quintic peaks, second-order step response, Astrom-Murray Table 6.1), mrcpp (the C++
// port of modern_robotics: RNEA), synthetic signals with known answers, the synthetic station logs in ../data/, and
// cpp_cases.json, which holds the numbers of the Python reference, so both languages are checked against the same
// values.
#include "lab.hpp"

#include <sbeg/labio.hpp>
#include <sbeg/mrcpp.hpp>

#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

using Eigen::Index;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using sbeg::as_mat;
using sbeg::as_vec;
using sbeg::json;

namespace {

const json& cases() {  // loaded once
    static const json C = sbeg::load_json(sbeg::lab_path("cpp_cases.json"));
    return C;
}

const json& task() {
    static const json T = lab::load_task();
    return T;
}

const lab::Model& model() {
    static const lab::Model M = lab::course_arm_model(task()["payload_kg"].get<double>());
    return M;
}

/// The `cmd` fixture of test_lab.py: the first-run command through the task waypoints (your waypoint_command).
lab::WaypointCommand make_cmd() {
    const VectorXd v = task()["first_run_speed_fraction"].get<double>() * model().qdot_max;
    const VectorXd a = VectorXd::Constant(6, task()["qddot_max"].get<double>());
    return lab::waypoint_command(as_mat(task()["waypoints"]), v, a, task()["servo_period_s"].get<double>(),
                                 task()["dwell_s"].get<double>());
}

/// One step of the step test: (t, cmd, meas) of one joint inside its window.
struct Step {
    VectorXd t, cmd, meas;
};

/// The `steps` fixture of test_lab.py: joints 2 and 3 of data/step_log.csv cut to the windows of the task file.
std::vector<Step> steps() {
    const lab::Log L = lab::load_log(lab::data_path("step_log.csv"));
    std::vector<Step> out;
    const json& st = task()["step_test"];
    for (std::size_t s = 0; s < st["joints"].size(); ++s) {
        const int j = st["joints"][s].get<int>() - 1;
        const double a = st["windows_s"][s][0].get<double>(), b = st["windows_s"][s][1].get<double>();
        std::vector<Index> rows;
        for (Index i = 0; i < L.t.size(); ++i)
            if (L.t(i) >= a - 1e-9 && L.t(i) <= b + 1e-9) rows.push_back(i);
        Step x;
        x.t.resize(static_cast<Index>(rows.size()));
        x.cmd.resize(x.t.size());
        x.meas.resize(x.t.size());
        for (std::size_t r = 0; r < rows.size(); ++r) {
            const Index k = static_cast<Index>(r);
            x.t(k) = L.t(rows[r]);
            x.cmd(k) = L.cmd(rows[r], j);
            x.meas(k) = L.meas(rows[r], j);
        }
        out.push_back(x);
    }
    return out;
}

/// np.arange(start, stop, step).
VectorXd arange(double start, double stop, double step) {
    const Index n = static_cast<Index>(std::ceil((stop - start) / step));
    VectorXd x(n);
    for (Index i = 0; i < n; ++i) x(i) = start + static_cast<double>(i) * step;
    return x;
}

/// np.linspace(start, stop, num).
VectorXd linspace(double start, double stop, Index num) {
    VectorXd x(num);
    const double step = (stop - start) / static_cast<double>(num - 1);
    for (Index i = 0; i < num; ++i) x(i) = start + static_cast<double>(i) * step;
    x(num - 1) = stop;
    return x;
}

/// np.gradient(F, t, axis=0): second-order central differences inside, one-sided at the ends.
MatrixXd gradient(const MatrixXd& F, const VectorXd& t) {
    const Index N = F.rows();
    MatrixXd G(N, F.cols());
    G.row(0) = (F.row(1) - F.row(0)) / (t(1) - t(0));
    G.row(N - 1) = (F.row(N - 1) - F.row(N - 2)) / (t(N - 1) - t(N - 2));
    for (Index i = 1; i + 1 < N; ++i) {
        const double hd = t(i) - t(i - 1), hs = t(i + 1) - t(i);
        G.row(i) = (hd * hd * F.row(i + 1) + (hs * hs - hd * hd) * F.row(i) - hs * hs * F.row(i - 1)) /
                   (hs * hd * (hd + hs));
    }
    return G;
}

/// np.allclose(got, expected, atol=atol) (rtol = 1e-5, element by element), with a readable message.
template <typename A, typename B>
sbeg::NearResult allclose(const Eigen::MatrixBase<A>& got, const Eigen::MatrixBase<B>& expected, double atol = 1e-8) {
    const MatrixXd e = expected.template cast<double>();
    sbeg::NearResult r = sbeg::compare_near(got, expected, std::numeric_limits<double>::infinity());
    if (!r.ok) return r;  // shape mismatch or NaN
    MatrixXd g = got.template cast<double>();
    if (g.rows() != e.rows()) g.transposeInPlace();  // row vs column vector of equal length
    bool ok = true;
    for (Index i = 0; i < e.rows(); ++i)
        for (Index k = 0; k < e.cols(); ++k)
            ok = ok && std::abs(g(i, k) - e(i, k)) <= atol + 1e-5 * std::abs(e(i, k));
    r = sbeg::compare_near(got, expected, atol);
    r.ok = ok;
    return r;
}

#define CHECK_ALLCLOSE(got, expected, atol)                                                    \
    do {                                                                                       \
        const ::sbeg::NearResult ac_ = allclose((got), (expected), (atol));                    \
        CHECK_MESSAGE(ac_.ok, "np.allclose(" #got ", " #expected ")\n  " << ac_.msg);          \
    } while (false)

/// math.isclose(got, expected, abs_tol=abs_tol) (rel_tol = 1e-9).
#define CHECK_ISCLOSE(got, expected, abs_tol)                                                                  \
    do {                                                                                                       \
        const double ic_g_ = static_cast<double>(got), ic_e_ = static_cast<double>(expected);                  \
        const double ic_t_ = std::max(1e-9 * std::max(std::abs(ic_g_), std::abs(ic_e_)), (abs_tol));           \
        CHECK_SCALAR_NEAR(ic_g_, ic_e_, ic_t_);                                                                \
    } while (false)

VectorXd vec(std::initializer_list<double> v) {
    VectorXd x(static_cast<Index>(v.size()));
    Index i = 0;
    for (double e : v) x(i++) = e;
    return x;
}

std::string join(const std::vector<std::string>& v) {
    std::ostringstream s;
    s << "[";
    for (std::size_t i = 0; i < v.size(); ++i) s << (i ? ", " : "") << "\"" << v[i] << "\"";
    s << "]";
    return s.str();
}

json good_record() {
    return json{{"members_at_arm", json::array({"Salma", "Omar"})}, {"inducted", json::array({"Salma", "Omar", "Youssef"})},
                {"supervisor", "TA K."}, {"estop_tested", true}, {"estop_holder", "Omar"}, {"keepout_clear", true},
                {"workspace_ready", true}, {"first_run", true}, {"check", {{"ok", true}, {"peak_speed_ratio", 0.999}}},
                {"check_speed_fraction", 0.25}, {"margin_deg", 10.0}, {"dry_run_speed_ratio", 0.999},
                {"dry_run_torque_ratio", 0.50}, {"current_limit_fraction", 0.6}, {"payload_kg", 0.1}};
}

}  // namespace

// ------------------------------------------------------------------ Part A: pre-flight
TEST_CASE("test_servo_model_conversion") {
    const VectorXd w0 = as_mat(task()["waypoints"]).row(0).transpose();
    const VectorXd off = as_vec(task()["servo_zero_offset_deg"]);
    const VectorXd s = lab::model_to_servo(w0, off);
    CHECK_ALLCLOSE(s, vec({1.5, -36.3775, 80.7141, -137.5099, -90.9544, 0.0}), 1e-4);
    CHECK_MAT_NEAR(s, as_vec(cases()["servo_model_conversion"]["servo"]), 1e-9);
    CHECK_ALLCLOSE(lab::servo_to_model(s, off), w0, 1e-12);
}

TEST_CASE("test_segment_time_hand_values") {
    const MatrixXd W = as_mat(task()["waypoints"]);
    const VectorXd v = 0.25 * model().qdot_max;
    VectorXd T(3);
    for (Index k = 0; k < 3; ++k)
        T(k) = lab::segment_time((W.row(k + 1) - W.row(k)).transpose(), v, VectorXd::Constant(6, 2.0), 0.02);
    CHECK_ALLCLOSE(T, vec({2.42, 1.62, 4.04}), 1e-9);
    // first segment: speed-limited, 15/8 * 0.9 / 0.6981 = 2.417 s, rounded up to 2.42 s
    CHECK_ISCLOSE(15.0 / 8.0 * 0.9 / v(0), 2.4172, 1e-4);
    // acceleration-limited case: tiny speed limit removed
    CHECK_ISCLOSE(lab::segment_time(vec({1.0}), vec({100.0}), vec({1.0}), 1e-6), std::sqrt(10.0 / std::sqrt(3.0)),
                  1e-5);
}

TEST_CASE("test_waypoint_command_structure") {
    const lab::WaypointCommand c = make_cmd();
    const json& ref = cases()["waypoint_command_structure"];
    REQUIRE(c.t.size() == 505);
    REQUIRE(c.q.rows() == 505);
    REQUIRE(c.qd.rows() == 505);
    REQUIRE(c.qdd.rows() == 505);
    CHECK_ISCLOSE(c.t(504), 10.08, 1e-9);
    CHECK_ALLCLOSE((c.t.tail(504) - c.t.head(504)).eval(), VectorXd::Constant(504, 0.02), 1e-8);
    CHECK_ALLCLOSE(c.seg_T, vec({2.42, 1.62, 4.04}), 1e-8);
    CHECK_ALLCLOSE(c.t_start, vec({0.5, 3.42, 5.54}), 1e-8);
    const MatrixXd W = as_mat(task()["waypoints"]);
    REQUIRE(c.t_start.size() == 3);
    for (Index k = 0; k < c.t_start.size(); ++k) {  // at rest at every waypoint
        const Index i = static_cast<Index>(std::llround(c.t_start(k) / 0.02));
        INFO("waypoint " << k << ", sample " << i);
        REQUIRE(i < 505);
        CHECK_ALLCLOSE(c.q.row(i), W.row(k), 1e-8);
        CHECK_ALLCLOSE(c.qd.row(i), VectorXd::Zero(6), 1e-8);
    }
    CHECK_ALLCLOSE(c.q.row(504), W.row(W.rows() - 1), 1e-8);
    const MatrixXd fd = gradient(c.q, c.t);  // analytic qd agrees with the samples
    CHECK((fd - c.qd).cwiseAbs().maxCoeff() < 0.02);
    // the same samples as the Python reference
    const std::vector<int> rows = ref["rows"].get<std::vector<int>>();
    for (std::size_t r = 0; r < rows.size(); ++r) {
        INFO("sample " << rows[r]);
        CHECK_MAT_NEAR(c.q.row(rows[r]), as_vec(ref["q"][r]), 1e-9);
        CHECK_MAT_NEAR(c.qd.row(rows[r]), as_vec(ref["qd"][r]), 1e-9);
        CHECK_MAT_NEAR(c.qdd.row(rows[r]), as_vec(ref["qdd"][r]), 1e-9);
    }
}

TEST_CASE("test_command_passes_first_run_check") {
    const lab::WaypointCommand cmd = make_cmd();
    const lab::CommandCheck c = lab::check_command(cmd.t, cmd.q, model(), 0.25, 10.0);
    CHECK(c.ok);
    CHECK(c.peak_speed_ratio > 0.99);
    CHECK(c.peak_speed_ratio <= 1.0);
    CHECK_SCALAR_NEAR(c.peak_speed_ratio, cases()["command_passes_first_run_check"]["peak_speed_ratio"].get<double>(),
                      1e-9);
    CHECK_FALSE(lab::check_command(cmd.t, cmd.q, model(), 0.2, 10.0).ok);

    // (Python: test_joint_limit_margin_is_checklist_value) Rule A1 (brief Sec. 5.2, shared checklist): software limits
    // >= 10 deg inside the end stops (<= +-155 deg, J6 <= +-165 deg); check_command uses this margin by default.
    CHECK(lab::SOFT_LIMIT_MARGIN_DEG == 10.0);
    CHECK_ALLCLOSE(((model().limits.col(1) / lab::DEG).array() - lab::SOFT_LIMIT_MARGIN_DEG).matrix(),
                   vec({155, 155, 155, 155, 155, 165}), 1e-8);
    const VectorXd t = arange(0, 1.02, 0.02);
    for (const std::pair<double, bool>& lim : {std::pair<double, bool>{150.0, true}, {157.0, false}}) {
        const double deg = lim.first;
        const bool ok = lim.second;
        INFO("joint 1 at " << deg << " deg");  // 157 deg: inside a 5 deg margin, outside 10 deg
        MatrixXd q = MatrixXd::Zero(t.size(), 6);
        q.col(0).setConstant(deg * lab::DEG);
        CHECK(lab::check_command(t, q, model(), 0.25).within_limits == ok);
    }
}

TEST_CASE("test_dry_run") {
    const lab::WaypointCommand cmd = make_cmd();
    const lab::DryRun d = lab::dry_run(model(), cmd, 0.25, 0.6);
    const json& ref = cases()["dry_run"];
    REQUIRE(d.tau.rows() == cmd.q.rows());
    REQUIRE(d.peak_tau.size() == 6);
    REQUIRE(d.torque_ratio.size() == 6);
    const Index k = 150;  // one sample checked directly with mrcpp (= modern_robotics)
    const VectorXd tau = mr::inverse_dynamics(cmd.q.row(k).transpose(), cmd.qd.row(k).transpose(),
                                              cmd.qdd.row(k).transpose(), model().g, mr::Vector6d::Zero(),
                                              model().Mlist, model().Glist, model().Slist);
    CHECK_ALLCLOSE(d.tau.row(k), tau, 1e-10);
    CHECK_ALLCLOSE(d.peak_tau.head(4), vec({0.0257, 0.8916, 0.3828, 0.1302}), 2e-4);
    CHECK_ISCLOSE(d.torque_ratio(1), 0.4953, 2e-4);
    CHECK(d.ok);
    CHECK(d.speed_ratio.maxCoeff() <= 1.0);
    CHECK_FALSE(lab::dry_run(model(), cmd, 0.25, 0.25).ok);  // 25 % torque: joint 2 would exceed
    CHECK_MAT_NEAR(d.peak_tau, as_vec(ref["peak_tau"]), 1e-9);
    CHECK_MAT_NEAR(d.speed_ratio, as_vec(ref["speed_ratio"]), 1e-9);
}

// ------------------------------------------------------------------ Part B: step test, identification
TEST_CASE("test_zeta_wn_from_step_closed_form") {
    const double zeta = 0.6, wn = 20.0;
    const double Mp = std::exp(-lab::PI * zeta / std::sqrt(1 - zeta * zeta));
    const double tp = lab::PI / (wn * std::sqrt(1 - zeta * zeta));
    const auto [z, w] = lab::zeta_wn_from_step(Mp, tp);
    CHECK_ISCLOSE(z, 0.6, 1e-12);
    CHECK_ISCLOSE(w, 20.0, 1e-9);
    CHECK_ISCLOSE(lab::zeta_wn_from_step(0.16, 0.2).first, 0.5038, 1e-4);  // A-M Table 6.1: 16 % <-> 0.5
}

TEST_CASE("test_step_features_synthetic") {
    // (Python: test_step_response_matches_servo_ode) the provided step_response agrees with the servo simulator
    {
        const VectorXd t = arange(0, 1.0, 0.002);
        MatrixXd c(t.size(), 1);
        for (Index i = 0; i < t.size(); ++i) c(i, 0) = t(i) >= 0.1 ? 1.0 : 0.0;
        const VectorXd y = lab::simulate_servo(t, c, 15.0, 0.4, 0.0, VectorXd(), nullptr, 0.0).col(0);
        CHECK((y - lab::step_response((t.array() - 0.1).matrix(), 15.0, 0.4)).cwiseAbs().maxCoeff() < 0.01);
    }
    const VectorXd t = arange(0, 2.0, 0.001);
    VectorXd c(t.size());
    for (Index i = 0; i < t.size(); ++i) c(i) = t(i) >= 0.5 ? 0.05 : 0.0;
    const VectorXd y = 0.05 * lab::step_response((t.array() - 0.53).matrix(), 20.0, 0.6);
    const lab::StepFeatures f = lab::step_features(t, c, y);
    CHECK_ISCLOSE(f.t_step, 0.5, 1e-9);
    CHECK_ISCLOSE(f.A, 0.05, 0.0);
    CHECK_ISCLOSE(f.overshoot, std::exp(-0.6 * lab::PI / 0.8), 2e-4);
    CHECK_ISCLOSE(f.t_peak_abs - 0.53, lab::PI / 16, 2e-3);
    CHECK(f.t_move > 0.53);
    CHECK(f.t_move < 0.53 + 0.06);
}

TEST_CASE("test_identify_servo_noise_free") {
    const VectorXd t = arange(0, 1.5, 0.02);
    MatrixXd c(t.size(), 1);
    for (Index i = 0; i < t.size(); ++i) c(i, 0) = t(i) >= 0.4 ? 0.06 : 0.0;
    const VectorXd y = lab::simulate_servo(t, c, 18.0, 0.5, 0.04, VectorXd(), nullptr, 0.0).col(0);
    const lab::ServoFit r = lab::identify_servo(t, c.col(0), y);
    INFO("wn = " << r.wn << ", zeta = " << r.zeta << ", delay = " << r.delay);
    CHECK(std::abs(r.wn - 18.0) < 0.4);
    CHECK(std::abs(r.zeta - 0.5) < 0.02);
    CHECK(std::abs(r.delay - 0.04) < 0.004);
}

TEST_CASE("test_identify_servo_station_log") {
    const std::vector<Step> S = steps();
    REQUIRE(S.size() == 2);
    const double expected[2][3] = {{19.75, 0.595, 0.0288}, {19.98, 0.602, 0.0297}};
    for (std::size_t s = 0; s < 2; ++s) {
        const lab::ServoFit r = lab::identify_servo(S[s].t, S[s].cmd, S[s].meas);
        INFO("joint " << s + 2 << ": wn = " << r.wn << ", zeta = " << r.zeta << ", delay = " << r.delay
                      << ", rms = " << r.rms);
        CHECK(std::abs(r.wn - expected[s][0]) < 0.3);
        CHECK(std::abs(r.zeta - expected[s][1]) < 0.01);
        CHECK(std::abs(r.delay - expected[s][2]) < 0.002);
        CHECK(r.rms < 5e-4);
    }
}

// ------------------------------------------------------------------ Part C: static poses
TEST_CASE("test_torque_constant_and_normalised_rms") {
    const VectorXd i = vec({0.2, 0.4, 0.6});
    CHECK_ISCLOSE(lab::torque_constant(i, 1.25 * i), 1.25, 0.0);
    CHECK_ISCLOSE(lab::normalised_rms(vec({1, 2, 3}), vec({1, 2, 3})), 0.0, 0.0);
    CHECK_ISCLOSE(lab::normalised_rms(vec({0, 0, 0}), vec({1, -1, 1})), 1.0, 0.0);
    CHECK_ISCLOSE(lab::normalised_rms(vec({1.1, 2.2}), vec({1.0, 2.0})), 0.1, 0.0);
}

TEST_CASE("test_loop_stiffness") {
    const VectorXd tau = vec({-0.9, -0.6, -0.3});
    CHECK_ISCLOSE(lab::loop_stiffness(tau / 30.0, tau), 30.0, 0.0);
}

TEST_CASE("test_gravity_check_station_log") {
    const lab::StaticPoses st = lab::average_approaches(lab::load_log(lab::data_path("static_log.csv")));
    const lab::GravityCheck gc = lab::gravity_check(st, model(), {1, 2, 3},
                                                    task()["static_fit_poses"].get<std::vector<int>>(),
                                                    task()["static_check_poses"].get<std::vector<int>>());
    REQUIRE(gc.g.rows() == st.meas.rows());
    REQUIRE(gc.g.cols() == 6);
    MatrixXd g(st.meas.rows(), 6);
    for (Index p = 0; p < st.meas.rows(); ++p)
        g.row(p) = mr::inverse_dynamics(st.meas.row(p).transpose(), VectorXd::Zero(6), VectorXd::Zero(6), model().g,
                                        mr::Vector6d::Zero(), model().Mlist, model().Glist, model().Slist)
                       .transpose();
    CHECK_ALLCLOSE(gc.g, g, 1e-10);
    CHECK_ALLCLOSE(gc.k, vec({1.1839, 1.1635, 0.4487}), 2e-3);
    CHECK_ALLCLOSE(gc.K, vec({28.66, 13.95, 5.356}), 0.05);
    REQUIRE(gc.nrms.size() == 3);
    CHECK(gc.nrms.maxCoeff() < 0.3);
}

// ------------------------------------------------------------------ Part D: comparison, stop, pre-flight
TEST_CASE("test_compare_runs_synthetic") {
    const VectorXd t = linspace(0, 2, 101);
    MatrixXd c(t.size(), 2);
    c.col(0) = t.array().sin().matrix();
    c.col(1) = t.array().cos().matrix();
    const Eigen::RowVector2d dm(0.01, -0.02), ds(0.01, 0.0);
    const lab::RunComparison out = lab::compare_runs(t, c, c.rowwise() + dm, c.rowwise() + ds);
    CHECK_ALLCLOSE(out.rms_cmd_meas, vec({0.01, 0.02}), 1e-8);
    CHECK_ALLCLOSE(out.max_cmd_meas, vec({0.01, 0.02}), 1e-8);
    CHECK_ALLCLOSE(out.rms_sim_meas, vec({0.0, 0.02}), 1e-12);
    MatrixXd lagged(t.size(), 2);
    lagged.col(0) = (t.array() - 0.08).sin().matrix();
    lagged.col(1) = (t.array() - 0.08).cos().matrix();
    CHECK_ISCLOSE(lab::compare_runs(t, c, lagged, c).lag, 0.08, 1e-9);
}

TEST_CASE("test_identified_model_beats_nominal") {
    const lab::Log T = lab::load_log(lab::data_path("traj_log.csv"));
    double wn = 0.0, z = 0.0, d = 0.0;
    const std::vector<Step> S = steps();
    for (const Step& s : S) {
        const lab::ServoFit r = lab::identify_servo(s.t, s.cmd, s.meas);
        wn += r.wn / static_cast<double>(S.size());
        z += r.zeta / static_cast<double>(S.size());
        d += r.delay / static_cast<double>(S.size());
    }
    const lab::StaticPoses st = lab::average_approaches(lab::load_log(lab::data_path("static_log.csv")));
    VectorXd K = VectorXd::Constant(6, 1e9);
    const lab::GravityCheck gc = lab::gravity_check(st, model());
    REQUIRE(gc.K.size() == 3);
    K.segment(1, 3) = gc.K;
    const lab::ServoParams nomp = lab::NOMINAL_SERVO;
    const lab::RunComparison nom =
        lab::compare_runs(T.t, T.cmd, T.meas, lab::simulate_servo(T.t, T.cmd, nomp.wn, nomp.zeta, nomp.delay));
    const lab::RunComparison idm =
        lab::compare_runs(T.t, T.cmd, T.meas, lab::simulate_servo(T.t, T.cmd, wn, z, d, K, &model()));
    REQUIRE(nom.rms_cmd_meas.size() == 6);
    REQUIRE(idm.rms_sim_meas.size() == 6);
    CHECK_ISCLOSE(nom.lag, 0.10, 1e-9);
    CHECK_ALLCLOSE(nom.rms_cmd_meas.head(3), vec({0.0395, 0.0329, 0.0332}), 3e-4);
    CHECK_ALLCLOSE(nom.rms_sim_meas.segment(1, 3), vec({0.0282, 0.0261, 0.0243}), 3e-4);
    INFO("identified rms_sim_meas = " << idm.rms_sim_meas.transpose());
    CHECK(idm.rms_sim_meas.maxCoeff() < 1e-3);
    CHECK((idm.rms_sim_meas.segment(1, 3).array() < nom.rms_sim_meas.segment(1, 3).array() / 20).all());
}

TEST_CASE("test_stop_time") {
    const VectorXd t = arange(0, 2, 0.02);
    MatrixXd q(t.size(), 1);
    for (Index i = 0; i < t.size(); ++i) q(i, 0) = t(i) < 1.0 ? 0.5 * t(i) : 0.5;  // stops dead at t = 1
    // window 0.1 s -> +-2 samples: the last centre sample that still sees motion is t = 1.02 s
    CHECK_ISCLOSE(lab::stop_time(t, q, 0.9), 0.14, 1e-9);
    const lab::Log P = lab::load_log(lab::data_path("stop_log.csv"));
    const double ts = lab::stop_time(P.t, P.meas, task()["stop_test"]["t_trigger_s"].get<double>());
    CHECK_ISCLOSE(ts, 0.34, 1e-9);
    CHECK(ts < task()["stop_test"]["pass_s"].get<double>());
    const double never = lab::stop_time(t, (0.5 * t).eval(), 0.5);
    CHECK(never == std::numeric_limits<double>::infinity());
}

TEST_CASE("test_preflight") {
    const std::vector<std::string> got = lab::preflight(good_record());
    CHECK_MESSAGE(got.empty(), "failed rules: " << join(got));
    const json tmpl = sbeg::load_json(lab::data_path("session_record_template.json"));
    CHECK(lab::preflight(tmpl).size() == lab::PREFLIGHT_RULES.size());  // an empty record fails all
    struct Case {
        const char* key;
        json value;
        std::size_t rule;
    };
    const std::vector<Case> cs = {{"inducted", json::array({"Salma"}), 0}, {"supervisor", " ", 1},
                                  {"estop_tested", false, 2}, {"estop_holder", "Youssef", 3},
                                  {"keepout_clear", false, 4}, {"workspace_ready", false, 5},
                                  {"check_speed_fraction", 0.5, 6}, {"margin_deg", 5.0, 6},
                                  {"dry_run_torque_ratio", 1.2, 7}, {"current_limit_fraction", 0.8, 8},
                                  {"payload_kg", 0.2, 9}, {"payload_kg", nullptr, 9}};
    for (const Case& c : cs) {
        json r = good_record();
        r[c.key] = c.value;
        const std::vector<std::string> f = lab::preflight(r);
        INFO(c.key << " = " << c.value.dump());
        CHECK_MESSAGE(f == std::vector<std::string>{lab::PREFLIGHT_RULES[c.rule]}, "failed rules: " << join(f));
    }
    json r = good_record();
    r["first_run"] = false;  // a repeat run may be checked at 50 %
    r["check_speed_fraction"] = 0.5;
    const std::vector<std::string> f1 = lab::preflight(r);
    CHECK_MESSAGE(f1.empty(), "failed rules: " << join(f1));
    r = good_record();
    r["payload_kg"] = 0.125;  // rule A4: exactly 0.125 kg is allowed
    const std::vector<std::string> f2 = lab::preflight(r);
    CHECK_MESSAGE(f2.empty(), "failed rules: " << join(f2));
    // the rule texts are those of the Python version
    CHECK(cases()["preflight"]["rules"].get<std::vector<std::string>>() == lab::PREFLIGHT_RULES);
}
