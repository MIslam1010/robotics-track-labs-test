// Auto-checks for SBEG 46X Lab 11 (simulated powered wheelchair: odometry, EKF landmark localisation, tracking
// control) -- C++ track. Mirrors test_lab.py test by test (25 test cases, same names and order).
//
// Build and run from this folder:
//     cmake -S . -B build -G Ninja && cmake --build build && ./build/lab_tests
//     ./build/lab_tests -tc="*wheel*,*H0*,*diffdrive*,*odometry_step*,*dead_reckoning*"   (like pytest -k)
// Ground truth: the wheel models of Modern Robotics (H(0) of MR Eqs. 13.9 and 13.10, F of MR Eq. 13.34), the matrix
// exponential of the chassis twist (mrcpp matrix_exp6 and the scipy expm values of the Python tests), finite
// differences, closed-form cases, the course model srs_7r_feeding (course_models.cpp), data/expected.json and
// cpp_cases.json (numbers of the Python reference, including the random inputs that test_lab.py draws), so both
// languages are checked against the same values.
//
// Plot files: when the functions they use work, the tests write lab11_cpp_odometry.csv (Experiment A1),
// lab11_cpp_ekf.csv and lab11_cpp_ekf_readings.csv (B1) and lab11_cpp_track_<truth|odometry|ekf>.csv (C1) into the
// folder you run lab_tests from; plot them with  python cpp/plot_results.py
#include "lab.hpp"
#include "course_models.hpp"

#include <sbeg/labio.hpp>
#include <sbeg/mrcpp.hpp>

#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <set>
#include <string>
#include <vector>

using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using lab::Matrix23d;
using lab::Matrix32d;
using sbeg::as;
using sbeg::json;

namespace {
constexpr double PI = 3.14159265358979323846;

const json& cases() {
    static const json C = sbeg::load_json(sbeg::lab_path("cpp_cases.json"));
    return C;
}
const json& EXP() {
    static const json E = sbeg::load_json(sbeg::lab_path("../data/expected.json"));
    return E;
}
const lab::Params& P() {
    static const lab::Params p = lab::load_params();
    return p;
}
double r() { return P().chair.r; }
double d() { return P().chair.d; }

Matrix2d Rm() {
    Matrix2d R = Matrix2d::Zero();
    R(0, 0) = P().ekf.sigma_range * P().ekf.sigma_range;
    R(1, 1) = std::pow(P().ekf.sigma_bearing_deg * PI / 180.0, 2);
    return R;
}

double wrapped(double a) {  // test_lab.py: (a + pi) % (2 pi) - pi  (floored modulo)
    double m = std::fmod(a + PI, 2 * PI);
    if (m < 0) m += 2 * PI;
    return m - PI;
}

/// Independent odometry: T_sb' = T_sb e^[Vb6] (MR Sec. 13.4) with mrcpp's matrix_exp6.
Vector3d exp_odometry_mr(const Vector3d& q, const Vector3d& Vb) {
    const Matrix4d Tsb = mr::rp_to_trans(mr::matrix_exp3(mr::vec_to_so3(Vector3d(0, 0, q(0)))), Vector3d(q(1), q(2), 0));
    mr::Vector6d V6;
    V6 << 0, 0, Vb(0), Vb(1), Vb(2), 0;
    const Matrix4d T = Tsb * mr::matrix_exp6(mr::vec_to_se3(V6));
    return Vector3d(q(0) + Vb(0), T(0, 3), T(1, 3));
}

MatrixXd log_enc(const lab::Log& log) {
    MatrixXd enc(log.enc_L.size(), 2);
    enc << log.enc_L, log.enc_R;
    return enc;
}
MatrixXd log_truth(const lab::Log& log) {
    MatrixXd q(log.phi.size(), 3);
    q << log.phi, log.x, log.y;
    return q;
}
}  // namespace

// ------------------------------------------------------------------------------ data and provided code
TEST_CASE("test_data_files_consistent") {
    const lab::Log log = lab::load_log();
    const std::vector<lab::Reading> meas = lab::load_meas();
    CHECK(log.t.size() == EXP()["N"].get<Eigen::Index>());
    CHECK(meas.size() == EXP()["n_meas"].get<std::size_t>());
    REQUIRE(log.t.size() > 2);
    CHECK((log.t.tail(log.t.size() - 1) - log.t.head(log.t.size() - 1)).minCoeff() > 0);
    bool all_scans = true;
    for (const lab::Reading& z : meas) all_scans = all_scans && (z.k % P().sensor.every == 0);
    CHECK(all_scans);
    // C++ addition: the first encoder increment is the nominal wheel speed of the route times h, quantised.
    const double res = 2 * PI / P().chair.enc_counts_per_rev, h = P().chair.h;
    const auto u = lab::wheel_speeds(P().route.v, 0.0, r(), d());
    CHECK_SCALAR_NEAR(log.enc_L(1), std::floor(u.first * h / res + 1e-9) * res, 1e-9);
    CHECK_SCALAR_NEAR(log.enc_R(1), std::floor(u.second * h / res + 1e-9) * res, 1e-9);
}

TEST_CASE("test_reference_route_closes") {
    const lab::Reference ref = lab::reference_trajectory(P());
    double lap = 0.0;
    for (const lab::RouteSegment& s : P().route.segments) lap += s.kind == "line" ? s.a : s.a * s.b * PI / 180.0;
    CHECK(std::abs(lap - (20.0 + 3 * PI)) < 1e-9);
    const double vh = P().route.v * P().chair.h;
    const Eigen::Index n = ref.qd.rows();
    CHECK_MAT_NEAR(ref.qd.block(n - 1, 1, 1, 2), Eigen::RowVector2d(0, 0), vh);
    CHECK(std::abs(ref.qd(n - 1, 0) - 4 * PI) < vh / 1.5);  // last sample is on the last arc
    // C++ addition: the reference is feasible for the wheel-speed limit.
    double umax = 0.0;
    for (Eigen::Index i = 0; i < n; ++i) {
        const auto u = lab::wheel_speeds(ref.vd(i), ref.wd(i), r(), d());
        umax = std::max({umax, std::abs(u.first), std::abs(u.second)});
    }
    CHECK(umax <= P().chair.u_max);
}

TEST_CASE("test_log_ground_truth_moves_like_a_diff_drive") {
    // the true chair obeys the nonholonomic constraint xdot sin(phi) - ydot cos(phi) = 0 (MR p. 528) to O(h^2)
    const lab::Log log = lab::load_log();
    const Eigen::Index n = log.x.size();
    double worst = 0.0;
    for (Eigen::Index i = 0; i + 1 < n; ++i) {
        const double dx = log.x(i + 1) - log.x(i), dy = log.y(i + 1) - log.y(i);
        const double phm = 0.5 * (log.phi(i + 1) + log.phi(i));
        worst = std::max(worst, std::abs(dx * std::sin(phm) - dy * std::cos(phm)));
    }
    CHECK(worst < 1e-5);
    // C++ addition: one odometry step from the first encoder increment lands within 2 mm / 2 mrad of the true pose
    // (the true chair has slightly different wheel radii, so it already turns a little).
    const MatrixXd q = log_truth(log);
    const Vector2d dth(log.enc_L(1) - log.enc_L(0), log.enc_R(1) - log.enc_R(0));
    CHECK_MAT_NEAR(lab::odometry_step(q.row(0).transpose(), dth, lab::diffdrive_F(r(), d())),
                   q.row(1).transpose(), 2e-3);
}

TEST_CASE("test_wrap_and_landmarks") {
    CHECK(std::abs(lab::wrap(3 * PI / 2) + PI / 2) < 1e-12);
    std::set<int> ids;
    for (const auto& kv : lab::landmark_table(P())) ids.insert(kv.first);
    CHECK((ids == std::set<int>{1, 2, 3, 4, 5, 6, 7, 8}));
    // C++ addition: a marker straight behind the chair is seen at bearing +pi (wrap gives (-pi, pi]).
    CHECK_MAT_NEAR(lab::range_bearing(Vector3d(0, 0, 0), Vector2d(-3.0, 0.0)), Vector2d(3.0, PI), 1e-12);
}

// ------------------------------------------------------------------------------ Part A: kinematics and odometry
TEST_CASE("test_wheel_row_three_omni_mr_eq_13_9") {
    const double dd = 0.20, rr = 0.05;
    const double s3 = std::sin(PI / 3);
    const std::vector<lab::Wheel> wheels = {
        {rr, 0.0, 0.0, 0.0, dd},
        {rr, 0.0, -2 * PI / 3, dd * std::cos(-PI / 6), dd * std::sin(-PI / 6)},
        {rr, 0.0, 2 * PI / 3, -dd * std::cos(-PI / 6), dd * std::sin(-PI / 6)}};
    const MatrixXd H = lab::H0_matrix(wheels);
    Matrix3d H_mr;
    H_mr << -dd, 1, 0, -dd, -0.5, -s3, -dd, -0.5, s3;
    H_mr /= rr;
    CHECK_MAT_NEAR(H, H_mr, 1e-12);
    CHECK_MAT_NEAR(lab::wheel_row(wheels[1].r, wheels[1].gamma, wheels[1].beta, wheels[1].xw, wheels[1].yw),
                   H_mr.row(1).transpose(), 1e-12);
}

TEST_CASE("test_H0_mecanum_mr_eq_13_10") {
    const double rr = 0.0475, l = 0.235, w = 0.15;
    const double pi4 = PI / 4;
    const std::vector<lab::Wheel> wheels = {{rr, -pi4, 0.0, l, w}, {rr, pi4, 0.0, l, -w}, {rr, -pi4, 0.0, -l, -w},
                                            {rr, pi4, 0.0, -l, w}};
    MatrixXd H_mr(4, 3);
    H_mr << -l - w, 1, -1, l + w, 1, 1, l + w, 1, -1, -l - w, 1, 1;
    H_mr /= rr;
    const MatrixXd H = lab::H0_matrix(wheels);
    CHECK_MAT_NEAR(H, H_mr, 1e-12);
    CHECK(Eigen::FullPivLU<MatrixXd>(H).rank() == 3);
}

TEST_CASE("test_diffdrive_F_is_pseudoinverse_of_H0") {
    const MatrixXd F = lab::diffdrive_F(r(), d());
    const MatrixXd H = lab::H0_matrix({{r(), 0.0, 0.0, 0.0, d()}, {r(), 0.0, 0.0, 0.0, -d()}});  // left at y = +d
    REQUIRE(F.rows() == 3);
    REQUIRE(F.cols() == 2);
    CHECK_MAT_NEAR(F, mr::pinv(H), 1e-12);
    CHECK_MAT_NEAR(H * F, Matrix2d::Identity(), 1e-12);
}

TEST_CASE("test_wheel_speeds_inverse_of_F") {
    const double vw[3][2] = {{0.8, 0.4}, {0.0, 1.0}, {-0.3, 0.2}};
    for (const auto& c : vw) {
        INFO("v = " << c[0] << ", omega = " << c[1]);
        const auto u = lab::wheel_speeds(c[0], c[1], r(), d());
        CHECK_MAT_NEAR(lab::diffdrive_F(r(), d()) * Vector2d(u.first, u.second), Vector3d(c[1], c[0], 0.0), 1e-12);
    }
    const auto u = lab::wheel_speeds(0.8, 0.4, r(), d());
    CHECK_MAT_NEAR(Vector2d(u.first, u.second), Vector2d((0.8 - 0.112) / 0.17, (0.8 + 0.112) / 0.17), 1e-8);
}

TEST_CASE("test_odometry_step_matches_matrix_exponential") {
    // Python: parametrized over F = "diff" and "mecanum"; here one test case with both.
    const json& C = cases()["odometry_step_matches_matrix_exponential"];
    const MatrixXd Fd = lab::diffdrive_F(r(), d());
    for (const json& c : C["diff"]) {
        const Vector3d q = as<Vector3d>(c["q"]);
        const VectorXd dth = sbeg::as_vec(c["dtheta"]);
        INFO("diff: q = " << q.transpose() << ", dtheta = " << dth.transpose());
        CHECK_MAT_NEAR(lab::odometry_step(q, dth, Fd), as<Vector3d>(c["q_next"]), 1e-10);  // scipy expm
        CHECK_MAT_NEAR(lab::odometry_step(q, dth, Fd), exp_odometry_mr(q, Fd * dth), 1e-10);
    }
    const MatrixXd Fm = sbeg::as_mat(C["mecanum"]["F"]);
    for (const json& c : C["mecanum"]["cases"]) {
        const Vector3d q = as<Vector3d>(c["q"]);
        const VectorXd dth = sbeg::as_vec(c["dtheta"]);
        INFO("mecanum: q = " << q.transpose() << ", dtheta = " << dth.transpose());
        CHECK_MAT_NEAR(lab::odometry_step(q, dth, Fm), as<Vector3d>(c["q_next"]), 1e-10);
        CHECK_MAT_NEAR(lab::odometry_step(q, dth, Fm), exp_odometry_mr(q, Fm * dth), 1e-10);
    }
}

TEST_CASE("test_odometry_step_hand_values") {
    // Workshop Problem 3a: q = (0, 0, 0), dtheta = (2.0, 2.5) rad -> V_b = (0.15179, 0.3825, 0)
    const Vector3d q1 = lab::odometry_step(Vector3d::Zero(), Vector2d(2.0, 2.5), lab::diffdrive_F(r(), d()));
    const double w = 0.17 * 0.5 / 0.56;
    CHECK_MAT_NEAR(q1, Vector3d(w, 0.3825 * std::sin(w) / w, 0.3825 * (1 - std::cos(w)) / w), 1e-12);
}

TEST_CASE("test_dead_reckoning_on_the_log") {
    const lab::Log log = lab::load_log();
    const MatrixXd enc = log_enc(log), qt = log_truth(log);
    const MatrixXd qd = lab::dead_reckoning(qt.row(0).transpose(), enc, lab::diffdrive_F(r(), d()));
    REQUIRE(qd.rows() == enc.rows());
    REQUIRE(qd.cols() == 3);
    const Eigen::Index n = qd.rows();
    CHECK_MAT_NEAR(qd.row(n - 1), sbeg::as_vec(EXP()["odo_final"]), 1e-6);
    const double e = std::hypot(qd(n - 1, 1) - log.x(n - 1), qd(n - 1, 2) - log.y(n - 1));
    CHECK(std::abs(e - EXP()["odo_final_pos_err"].get<double>()) < 1e-6);
    // Experiment A1 plot file: t, s (distance travelled), true pose, odometry pose, errors
    const lab::PoseErrors pe = lab::pose_errors(qd, qt);
    MatrixXd out(n, 10);
    out << log.t, lab::distance_travelled(qt), qt, qd, pe.e_pos, pe.e_phi;
    sbeg::write_csv("lab11_cpp_odometry.csv", {"t", "s", "phi", "x", "y", "phi_odo", "x_odo", "y_odo", "e_pos", "e_phi"},
                    out);
}

// ------------------------------------------------------------------------------ Part B: EKF localisation
TEST_CASE("test_odometry_jacobians_finite_differences") {
    // Python: parametrized over 5 increments; F_q_fd, F_u_fd = central differences (eps 1e-6) of the expm odometry.
    for (const json& c : cases()["odometry_jacobians_finite_differences"]) {
        const Vector2d dth = as<Vector2d>(c["dtheta"]);
        const Vector3d q = as<Vector3d>(c["q"]);
        INFO("dtheta = " << dth.transpose());
        const lab::OdometryJacobians J = lab::odometry_jacobians(q, dth, r(), d());
        // Python: np.allclose(..., atol=1e-12), which adds rtol = 1e-5; the series branch (|omega| < 1e-4) differs
        // from the exact odometry_step by about 1e-12, so the absolute tolerance here is 1e-9.
        CHECK_MAT_NEAR(J.q_next, lab::odometry_step(q, dth, lab::diffdrive_F(r(), d())), 1e-9);
        CHECK_MAT_NEAR(J.F_q, as<Matrix3d>(c["F_q_fd"]), 1e-7);
        CHECK_MAT_NEAR(J.F_u, as<Matrix32d>(c["F_u_fd"]), 1e-7);
    }
}

TEST_CASE("test_wheel_noise_cov") {
    const Matrix2d Q = lab::wheel_noise_cov(Vector2d(2.0, -1.0), r(), 4e-5);
    CHECK_MAT_NEAR(Q, Vector2d(4e-5 * 2.0 / r(), 4e-5 * 1.0 / r()).asDiagonal().toDenseMatrix(), 1e-15);
}

TEST_CASE("test_ekf_predict_formula_and_growth") {
    const Vector3d q(0.3, 1.0, 2.0);
    const Matrix3d Pm = Vector3d(1e-3, 4e-4, 4e-4).asDiagonal();
    const Vector2d dth(1.2, 1.5);
    const auto pr = lab::ekf_predict(q, Pm, dth, r(), d(), 4e-5);
    const lab::OdometryJacobians J = lab::odometry_jacobians(q, dth, r(), d());
    CHECK_MAT_NEAR(pr.first, lab::odometry_step(q, dth, lab::diffdrive_F(r(), d())), 1e-12);
    const Matrix3d expected =
        J.F_q * Pm * J.F_q.transpose() + J.F_u * lab::wheel_noise_cov(dth, r(), 4e-5) * J.F_u.transpose();
    CHECK_MAT_NEAR(pr.second, expected, 1e-15);
    CHECK_MAT_NEAR(pr.second, pr.second.transpose(), 1e-8);
    CHECK(pr.second.determinant() > Pm.determinant());  // uncertainty grows (Corke Fig. 6.5)
}

TEST_CASE("test_range_bearing_values_and_jacobian") {
    const json& C = cases()["range_bearing_values_and_jacobian"];
    const Vector2d z = lab::range_bearing(Vector3d(PI / 2, 1.0, 1.0), Vector2d(4.0, 5.0));
    CHECK_MAT_NEAR(z, Vector2d(5.0, wrapped(std::atan2(4, 3) - PI / 2)), 1e-12);
    const Vector2d m = as<Vector2d>(C["m"]);
    for (const json& qj : C["qs"]) {  // 5 random poses of test_lab.py (rng 1112)
        const Vector3d q = as<Vector3d>(qj);
        INFO("q = " << q.transpose());
        const Matrix23d H = lab::range_bearing_jacobian(q, m);
        const double eps = 1e-6;
        Matrix23d Hn;
        for (int i = 0; i < 3; ++i) {
            const Vector3d e = Vector3d::Unit(i);
            const Vector2d dz = lab::range_bearing(q + eps * e, m) - lab::range_bearing(q - eps * e, m);
            Hn(0, i) = dz(0) / (2 * eps);
            Hn(1, i) = wrapped(dz(1)) / (2 * eps);
        }
        CHECK_MAT_NEAR(H, Hn, 1e-6);
    }
}

TEST_CASE("test_ekf_update_first_reading_of_the_log") {
    const json& fu = EXP()["first_update"];
    const auto lm = lab::landmark_table(P());
    const Vector3d q_prior = as<Vector3d>(fu["q_prior"]);
    const Matrix3d P_prior = as<Matrix3d>(fu["P_prior"]);
    const Vector2d mk = lm.at(fu["id"].get<int>());
    const lab::EkfUpdate u = lab::ekf_update(q_prior, P_prior, as<Vector2d>(fu["z"]), mk, Rm());
    CHECK(u.accepted);
    CHECK_MAT_NEAR(u.q, as<Vector3d>(fu["q_post"]), 1e-9);
    CHECK_MAT_NEAR(u.P, as<Matrix3d>(fu["P_post"]), 1e-12);
    CHECK(std::abs(u.d2 - fu["d2"].get<double>()) < 1e-7);
    // Joseph form equals (I - K H) P for the optimal gain
    const Matrix23d H = lab::range_bearing_jacobian(q_prior, mk);
    CHECK_MAT_NEAR(u.P, (Matrix3d::Identity() - u.K * H) * P_prior, 1e-12);
}

TEST_CASE("test_ekf_update_wraps_bearing_and_gates") {
    const Vector3d q(0.0, 0.0, 0.0);
    const Matrix3d Pm = Vector3d(1e-3, 1e-2, 1e-2).asDiagonal();
    const Vector2d m(-3.0, 0.0);              // marker straight behind: bearing = pi
    const Vector2d z(3.0, -PI + 0.01);        // measured just across the +-pi cut
    const lab::EkfUpdate u1 = lab::ekf_update(q, Pm, z, m, Rm());
    CHECK(std::abs(u1.nu(1) - 0.01) < 1e-12);
    CHECK(std::abs(u1.q(0)) < 0.02);
    const lab::EkfUpdate u2 = lab::ekf_update(q, Pm, Vector2d(2.0, PI), m, Rm(), 9.21);  // 1 m too short: a reflection
    CHECK_FALSE(u2.accepted);
    CHECK(u2.d2 > 9.21);
    CHECK_MAT_NEAR(u2.q, q, 1e-8);
    CHECK_MAT_NEAR(u2.P, Pm, 1e-8);
}

TEST_CASE("test_ekf_localise_on_the_log") {
    const lab::Log log = lab::load_log();
    const std::vector<lab::Reading> meas = lab::load_meas();
    const lab::EkfRun res = lab::ekf_localise(log, meas, P());
    const MatrixXd qt = log_truth(log);
    REQUIRE(res.q.rows() == qt.rows());
    REQUIRE(res.q.cols() == 3);
    REQUIRE(res.P.size() == static_cast<std::size_t>(qt.rows()));
    const MatrixXd e = res.q - qt;
    const double rms = std::sqrt((e.col(1).array().square() + e.col(2).array().square()).mean());
    const json& E = EXP()["ekf"];
    CHECK(res.n_acc == E["n_acc"].get<int>());
    CHECK(res.n_rej == E["n_rej"].get<int>());
    CHECK(std::abs(rms - E["rms_pos"].get<double>()) < 1e-6);
    CHECK_MAT_NEAR(res.q.row(res.q.rows() - 1), sbeg::as_vec(E["final"]), 1e-6);
    CHECK_MAT_NEAR(res.P.back(), as<Matrix3d>(EXP()["ekf_P_final"]), 1e-10);
    CHECK(rms < 0.02);  // odometry alone: about 1.6 m rms
    // Experiment B1 plot files: estimate, errors and 1-sigma values per sample; d2 of every reading
    const Eigen::Index n = res.q.rows();
    const lab::PoseErrors pe = lab::pose_errors(res.q, qt);
    MatrixXd out(n, 9);
    for (Eigen::Index k = 0; k < n; ++k) {
        const Matrix3d& Pk = res.P[static_cast<std::size_t>(k)];
        out.row(k) << log.t(k), res.q(k, 0), res.q(k, 1), res.q(k, 2), pe.e_pos(k), pe.e_phi(k),
            std::sqrt(Pk(0, 0)), std::sqrt(Pk(1, 1)), std::sqrt(Pk(2, 2));
    }
    sbeg::write_csv("lab11_cpp_ekf.csv",
                    {"t", "phi_est", "x_est", "y_est", "e_pos", "e_phi", "sigma_phi", "sigma_x", "sigma_y"}, out);
    if (res.d2.size() == meas.size()) {
        MatrixXd rd(static_cast<Eigen::Index>(meas.size()), 5);
        for (std::size_t i = 0; i < meas.size(); ++i)
            rd.row(static_cast<Eigen::Index>(i)) << meas[i].k, meas[i].t, meas[i].id, res.d2[i],
                res.d2[i] > P().ekf.gate ? 0.0 : 1.0;
        sbeg::write_csv("lab11_cpp_ekf_readings.csv", {"k", "t", "id", "d2", "accepted"}, rd);
    }
}

// ------------------------------------------------------------------------------ Part C: tracking control
TEST_CASE("test_error_coordinates_mr_eq_13_30") {
    const Vector3d qd(PI / 2, 1.0, 2.0);
    const Vector3d e = lab::error_coordinates(Vector3d(PI / 2 + 0.1, 1.3, 1.8), qd);
    CHECK_MAT_NEAR(e, Vector3d(0.1, -0.2, -0.3), 1e-12);
    CHECK(std::abs(lab::error_coordinates(Vector3d(3.1, 0, 0), Vector3d(-3.1, 0, 0))(0) - (6.2 - 2 * PI)) < 1e-12);
}

TEST_CASE("test_tracking_control_mr_eq_13_31") {
    const double k1 = 2.0, k2 = 4.0, k3 = 3.0;
    const auto vw0 = lab::tracking_control(Vector3d(0.3, 1, 2), Vector3d(0.3, 1, 2), 0.6, 0.4, k1, k2, k3);
    CHECK_MAT_NEAR(Vector2d(vw0.first, vw0.second), Vector2d(0.6, 0.4), 1e-8);
    const auto vw = lab::tracking_control(Vector3d(0.1, 0.1, -0.2), Vector3d(0.0, 0.0, 0.0), 0.6, 0.0, k1, k2, k3);
    const double pe = 0.1, xe = 0.1, ye = -0.2;  // q_d = 0: {d} = {s}
    const double v_ref = (0.6 - k1 * 0.6 * (xe + ye * std::tan(pe))) / std::cos(pe);
    const double w_ref = 0.0 - (k2 * 0.6 * ye + k3 * 0.6 * std::tan(pe)) * std::pow(std::cos(pe), 2);
    CHECK_MAT_NEAR(Vector2d(vw.first, vw.second), Vector2d(v_ref, w_ref), 1e-12);
}

TEST_CASE("test_point_controls_mr_eq_13_21") {
    const double cs[2][3] = {{0.4, 0.5, 0.0}, {-2.0, 0.3, 0.1}};
    for (const auto& c : cs) {
        const double phi = c[0], xr = c[1], yr = c[2];
        INFO("phi = " << phi << ", xr = " << xr << ", yr = " << yr);
        const double v = 0.7, w = -0.4;
        Matrix2d A;
        A << -std::sin(phi), -std::cos(phi), std::cos(phi), -std::sin(phi);
        const Vector2d pdot = Vector2d(v * std::cos(phi), v * std::sin(phi)) + w * A * Vector2d(xr, yr);  // Eq. 13.20
        const auto vw = lab::point_controls(phi, xr, yr, pdot);
        CHECK_MAT_NEAR(Vector2d(vw.first, vw.second), Vector2d(v, w), 1e-12);
    }
}

TEST_CASE("test_point_tracking_control_zero_error_gives_nominal") {
    const Vector3d qd(0.8, 2.0, 1.0);
    const auto vw = lab::point_tracking_control(qd, qd, 0.6, 0.4, 0.5, 1.5);
    CHECK_MAT_NEAR(Vector2d(vw.first, vw.second), Vector2d(0.6, 0.4), 1e-12);
}

TEST_CASE("test_saturate_wheels_keeps_curvature") {
    const auto vw = lab::saturate_wheels(1.0, 3.0, r(), d(), 9.0);
    const auto u = lab::wheel_speeds(vw.first, vw.second, r(), d());
    CHECK(std::abs(std::max(std::abs(u.first), std::abs(u.second)) - 9.0) < 1e-12);
    CHECK(std::abs(vw.second / vw.first - 3.0) < 1e-12);
    const auto a = lab::saturate_wheels(2.0, 0.0, r(), d(), 9.0, 1.2);
    CHECK_MAT_NEAR(Vector2d(a.first, a.second), Vector2d(1.2, 0.0), 1e-8);
    const auto b = lab::saturate_wheels(0.5, 0.2, r(), d(), 9.0, 1.2);
    CHECK_MAT_NEAR(Vector2d(b.first, b.second), Vector2d(0.5, 0.2), 1e-8);
}

TEST_CASE("test_closed_loop_runs") {
    // Python: parametrized over the estimator; here one test case with the three runs (seed 1211: the NumPy noise
    // sequences of the Python simulate(), stored in sim_noise_1211.json).
    for (const std::string est : {"truth", "odometry", "ekf"}) {
        INFO("estimator = " << est);
        const lab::SimResult sim = lab::simulate(P(), est, "nonlinear");
        const lab::TrackingMetrics m = lab::tracking_metrics(sim);
        const json& ref = EXP()["track_" + est + "_nonlinear"];
        CHECK(std::abs(m.rms_pos - ref["rms_pos"].get<double>()) < 1e-6);
        CHECK(std::abs(m.max_pos - ref["max_pos"].get<double>()) < 1e-6);
        CHECK(std::abs(m.final_pos - ref["final_pos"].get<double>()) < 1e-6);
        CHECK(std::abs(m.peak_u - ref["peak_u"].get<double>()) < 1e-6);
        // Experiment C1 plot file: reference, true and estimated pose, controls
        const Eigen::Index N = sim.t.size();
        MatrixXd out(N, 12);
        out << sim.t, sim.qd, sim.q_true.topRows(N), sim.q_est.topRows(N), sim.v, sim.w;
        sbeg::write_csv("lab11_cpp_track_" + est + ".csv",
                        {"t", "phi_d", "x_d", "y_d", "phi", "x", "y", "phi_est", "x_est", "y_est", "v", "omega"}, out);
    }
}

// ------------------------------------------------------------------------------ Part D (optional): mobile manipulation
TEST_CASE("test_base_jacobian_finite_difference_and_expected") {
    const json& C = cases()["base_jacobian_finite_difference_and_expected"];
    const cm::ArmModel arm = cm::srs_7r_feeding();
    CHECK_MAT_NEAR(arm.Blist, sbeg::as_mat(C["Blist"]), 1e-12);  // C++ copy of the course model = Python model
    Matrix4d Tb0 = Matrix4d::Identity();
    Tb0.topRightCorner<3, 1>() = P().arm_mount.p_b0;
    const VectorXd th = P().arm_mount.theta_feed;
    const MatrixXd F = lab::diffdrive_F(r(), d());
    const MatrixXd J = lab::mobile_manip_jacobian(F, Tb0, arm.Blist, arm.M, th);
    REQUIRE(J.rows() == 6);
    REQUIRE(J.cols() == 9);
    CHECK_MAT_NEAR(J.rightCols(7), mr::jacobian_body(arm.Blist, th), 1e-12);
    CHECK_MAT_NEAR(J.leftCols(2), sbeg::as_mat(EXP()["J_base_feed"]), 1e-10);
    const Matrix4d T0e = mr::fkin_body(arm.M, arm.Blist, th);
    for (const json& c : C["fd"]) {  // Ve = finite difference of the spoon pose for a small wheel motion eps u
        const Vector2d u = as<Vector2d>(c["u"]);
        INFO("u = " << u.transpose());
        CHECK_MAT_NEAR(lab::base_jacobian(F, Tb0, T0e) * u, sbeg::as_vec(c["Ve"]), 1e-5);
    }
}
