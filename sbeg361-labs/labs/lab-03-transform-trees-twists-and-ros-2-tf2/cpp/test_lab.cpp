// Auto-checks for SBEG 361 Lab 3 (transform trees and twists) -- C++ track. Mirrors test_lab.py test by test.
//
// Build and run from this folder:
//     cmake -S . -B build -G Ninja && cmake --build build && ./build/lab_tests
//     ./build/lab_tests -tc="*trans_inv*,*is_se3*"      (run only some tests, like pytest -k)
// Ground truth: mrcpp (the C++ port of modern_robotics), the worked examples of MR Section 3.3 (Fig. 3.14,
// Fig. 3.15, Example 3.19, Example 3.23) and cpp_cases.json, which holds the numbers of the Python reference
// (and of scipy.spatial.transform), so both languages are checked against the same values.
#include "lab.hpp"

#include <sbeg/labio.hpp>
#include <sbeg/mrcpp.hpp>

#include <doctest/doctest.h>

#include <cmath>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::Vector4d;
using lab::Matrix6d;
using lab::Vector6d;
using sbeg::as;
using sbeg::as_list;
using sbeg::json;

namespace {
constexpr double PI = 3.14159265358979323846;
const double S2 = 1.0 / std::sqrt(2.0);

const json& cases() {
    static const json C = sbeg::load_json(sbeg::lab_path("cpp_cases.json"));
    return C;
}

/// RANDOM_T of test_lab.py (20 random poses, stored in cpp_cases.json).
const std::vector<Matrix4d>& random_T() {
    static const std::vector<Matrix4d> Ts = as_list<Matrix4d>(cases()["random_T"]);
    return Ts;
}

Matrix4d mat4(double a00, double a01, double a02, double a03, double a10, double a11, double a12, double a13,
              double a20, double a21, double a22, double a23) {
    Matrix4d T;
    T << a00, a01, a02, a03, a10, a11, a12, a13, a20, a21, a22, a23, 0.0, 0.0, 0.0, 1.0;
    return T;
}

Matrix4d rp(const Matrix3d& R, const Vector3d& p) { return mr::rp_to_trans(R, p); }

Matrix3d mat3(double a, double b, double c, double d, double e, double f, double g, double h, double i) {
    Matrix3d R;
    R << a, b, c, d, e, f, g, h, i;
    return R;
}

// MR Fig. 3.14 (printed p. 90)
const Matrix4d T_SB_314 = rp(mat3(0, 0, 1, 0, -1, 0, 1, 0, 0), Vector3d(0, -2, 0));
const Matrix4d T_SC_314 = rp(mat3(-1, 0, 0, 0, 0, 1, 0, 1, 0), Vector3d(-1, 1, 0));
const Matrix4d T_BC_314 = rp(mat3(0, 1, 0, 0, 0, -1, -1, 0, 0), Vector3d(0, -3, -1));

// MR Example 3.19 (printed pp. 93-95)
const Matrix4d T_DB = mat4(0, 0, -1, 250, 0, -1, 0, -150, -1, 0, 0, 200);
const Matrix4d T_DE = mat4(0, 0, -1, 300, 0, -1, 0, 100, -1, 0, 0, 120);
const Matrix4d T_AD = mat4(0, 0, -1, 400, 0, -1, 0, 50, -1, 0, 0, 300);
const Matrix4d T_BC = mat4(0, -S2, -S2, 30, 0, S2, -S2, -40, 1, 0, 0, 25);
const Matrix4d T_CE_MR = mat4(0, 0, 1, -75, -S2, S2, 0, -260 * S2, -S2, -S2, 0, 160 * S2);

/// np.isclose(a, b, rtol, atol): |a - b| <= atol + rtol |b|.
double isclose_tol(double b, double rtol = 1e-5, double atol = 1e-8) { return atol + rtol * std::abs(b); }

/// Built inside each test (not shared) so that the starter fails as a test, not as an error.
lab::TransformTree load_nav() { return lab::load_tree_json(sbeg::lab_path("../nav_tree.json")); }

/// The tracked screwdriver (given loader; read once).
const std::pair<Eigen::VectorXd, std::vector<Matrix4d>>& track() {
    static const std::pair<Eigen::VectorXd, std::vector<Matrix4d>> tr =
        lab::load_track_csv(sbeg::lab_path("../tool_track.csv"));
    return tr;
}
}  // namespace

// ------------------------------------------------------------- Checkpoint 1: SE(3) basics
TEST_CASE("test_rp_to_trans_and_back") {
    for (int i = 0; i < 5; ++i) {
        const Matrix4d& T = random_T()[static_cast<std::size_t>(i)];
        INFO("RANDOM_T[" << i << "]");
        const auto [R, p] = mr::trans_to_rp(T);
        CHECK_MAT_NEAR(lab::rp_to_trans(R, p), T, 1e-12);
        const auto [R2, p2] = lab::trans_to_rp(T);
        CHECK_MAT_NEAR(R2, R, 1e-12);
        CHECK_MAT_NEAR(p2, p, 1e-12);
    }
}

TEST_CASE("test_trans_inv_matches_mr") {
    const json& c = cases()["trans_inv_matches_mr"];
    REQUIRE(c.size() == random_T().size());
    for (std::size_t i = 0; i < random_T().size(); ++i) {
        const Matrix4d& T = random_T()[i];
        INFO("RANDOM_T[" << i << "]");
        CHECK_MAT_NEAR(lab::trans_inv(T), mr::trans_inv(T), 1e-12);
        CHECK_MAT_NEAR(lab::trans_inv(T) * T, Matrix4d::Identity(), 1e-12);
        CHECK_MAT_NEAR(lab::trans_inv(T), as<Matrix4d>(c[i]["T_inv"]), 1e-12);
    }
}

TEST_CASE("test_is_se3") {
    CHECK(lab::is_se3(random_T()[0]));
    Matrix4d bad = random_T()[0];
    bad.topLeftCorner<3, 3>() *= -1.0;  // det R = -1 (a reflection)
    CHECK_FALSE(lab::is_se3(bad));
    Matrix4d bad2 = random_T()[0];
    bad2(3, 0) = 0.1;  // wrong last row
    CHECK_FALSE(lab::is_se3(bad2));
    CHECK_FALSE(lab::is_se3(Matrix3d::Identity()));
    for (const auto& c : cases()["is_se3"]["cases"]) CHECK(lab::is_se3(as<Matrix4d>(c["T"])) == c["expected"].get<bool>());
}

TEST_CASE("test_mr_fig_3_14_frames_and_point") {
    // T_bc = T_sb^-1 T_sc (MR p. 90) and v_b = (0, 0, 1.5) -> v_s.
    const Matrix4d T_bc = lab::trans_inv(T_SB_314) * T_SC_314;
    CHECK_MAT_NEAR(T_bc, T_BC_314, 1e-12);
    CHECK_MAT_NEAR(lab::transform_point(T_SB_314, Vector3d(0, 0, 1.5)), Vector3d(1.5, -2, 0), 1e-12);
    const json& c = cases()["mr_fig_3_14_frames_and_point"];
    const MatrixXd pts = sbeg::as_mat(c["pts"]);  // 7 x 3, one point per row
    MatrixXd hom(4, pts.rows());
    hom.topRows(3) = pts.transpose();
    hom.row(3).setOnes();
    const MatrixXd expected = (T_SB_314 * hom).topRows(3).transpose();
    CHECK_MAT_NEAR(lab::transform_points(T_SB_314, pts), expected, 1e-12);
    CHECK_MAT_NEAR(lab::transform_points(T_SB_314, pts), sbeg::as_mat(c["expected"]), 1e-12);
}

TEST_CASE("test_mr_fig_3_15_fixed_vs_body_frame") {
    // Pre-multiplication = fixed-frame displacement, post-multiplication = body-frame (MR Eqs. 3.66-3.67).
    const Matrix4d T = lab::rp_to_trans(mr::matrix_exp3(mr::vec_to_so3(Vector3d(0, 0, PI / 2))), Vector3d(0, 2, 0));
    const Matrix4d T_sb = mat4(0, 0, 1, 0, 0, -1, 0, -2, 1, 0, 0, 0);
    CHECK_MAT_NEAR(T * T_sb, mat4(0, 1, 0, 2, 0, 0, 1, 2, 1, 0, 0, 0), 1e-12);
    CHECK_MAT_NEAR(T_sb * T, mat4(0, 0, 1, 0, -1, 0, 0, -4, 0, -1, 0, 0), 1e-12);
}

TEST_CASE("test_distance_preserved") {
    // MR Prop. 3.18(a): ||Tx - Ty|| = ||x - y||.
    const json& c = cases()["distance_preserved"];
    const Vector3d x = as<Vector3d>(c["x"]), y = as<Vector3d>(c["y"]);
    const double d = (x - y).norm();
    for (int i = 0; i < 5; ++i) {
        const Matrix4d& T = random_T()[static_cast<std::size_t>(i)];
        const Vector3d Tx = lab::transform_point(T, x), Ty = lab::transform_point(T, y);
        CHECK_SCALAR_NEAR((Tx - Ty).norm(), d, isclose_tol(d));
    }
}

// ------------------------------------------------------------- Checkpoint 2: se(3) and adjoint
TEST_CASE("test_vec_to_se3_matches_mr") {
    for (const auto& c : cases()["vec_to_se3_matches_mr"]) {
        const Vector6d V = as<Vector6d>(c["V"]);
        CHECK_MAT_NEAR(lab::vec_to_se3(V), mr::vec_to_se3(V), 1e-12);
        CHECK_MAT_NEAR(lab::se3_to_vec(mr::vec_to_se3(V)), V, 1e-12);
    }
}

TEST_CASE("test_adjoint_matches_mr_and_properties") {
    const json& c = cases()["adjoint_matches_mr_and_properties"];
    REQUIRE(c.size() == 10);
    for (std::size_t i = 0; i < 10; ++i) {
        const Matrix4d& T1 = random_T()[i];
        const Matrix4d& T2 = random_T()[10 + i];
        INFO("T1 = RANDOM_T[" << i << "], T2 = RANDOM_T[" << 10 + i << "]");
        const Matrix6d A1 = lab::adjoint(T1);
        CHECK_MAT_NEAR(A1, mr::adjoint(T1), 1e-12);
        CHECK_MAT_NEAR(A1, as<Matrix6d>(c[i]["Ad_T1"]), 1e-12);
        const Matrix4d T12 = T1 * T2;
        CHECK_MAT_NEAR(A1 * lab::adjoint(T2), lab::adjoint(T12), 1e-10);                  // (3.77)
        CHECK_MAT_NEAR(Matrix6d(A1.inverse()), lab::adjoint(mr::trans_inv(T1)), 1e-10);  // (3.78)
        const Vector6d V = as<Vector6d>(c[i]["V"]);                                      // [Ad_T V] = T [V] T^-1
        const Vector6d AV = A1 * V;
        const Matrix4d rhs = T1 * mr::vec_to_se3(V) * mr::trans_inv(T1);
        CHECK_MAT_NEAR(mr::vec_to_se3(AV), rhs, 1e-10);
    }
}

// ------------------------------------------------------------- Checkpoint 3: twists and screws
TEST_CASE("test_twists_from_T_Tdot_mr_example_3_23") {
    // MR Example 3.23 (printed pp. 99-100): the three-wheeled car.
    const Matrix4d T_sb = mat4(-1, 0, 0, 4, 0, 1, 0, 0.4, 0, 0, -1, 0);
    Vector6d V_b;
    V_b << 0, 0, -2, 2.8, 4, 0;
    const Matrix4d Tdot = T_sb * mr::vec_to_se3(V_b);
    const auto [V_s, V_b2] = lab::twists_from_T_Tdot(T_sb, Tdot);
    Vector6d V_s_exp;
    V_s_exp << 0, 0, 2, -2, -4, 0;
    CHECK_MAT_NEAR(V_s, V_s_exp, 1e-12);
    CHECK_MAT_NEAR(V_b2, V_b, 1e-12);
    const Vector6d AdVb = mr::adjoint(T_sb) * V_b2;
    CHECK_MAT_NEAR(AdVb, V_s, 1e-12);
}

TEST_CASE("test_screw_to_twist_and_back") {
    const Vector6d V = lab::screw_to_twist(Vector3d(2, -1, 0), Vector3d(0, 0, 1), 0.0, 2.0);  // MR Example 3.23, V_s
    Vector6d V_exp;
    V_exp << 0, 0, 2, -2, -4, 0;
    CHECK_MAT_NEAR(V, V_exp, 1e-12);
    const lab::Screw sc = lab::twist_to_screw(V);
    CHECK_SCALAR_NEAR(sc.thetadot, 2.0, isclose_tol(2.0));
    CHECK_SCALAR_NEAR(sc.h, 0.0, isclose_tol(0.0));
    REQUIRE(sc.q.has_value());
    CHECK_MAT_NEAR(*sc.q, Vector3d(2, -1, 0), 1e-12);
    const Vector6d St = sc.S * sc.thetadot;
    CHECK_MAT_NEAR(St, V, 1e-12);
    for (const auto& c : cases()["screw_to_twist_and_back"]["random"]) {  // random screws, q off the origin
        const Vector3d s = as<Vector3d>(c["s"]), q = as<Vector3d>(c["q"]);
        const double h = c["h"].get<double>(), td = c["thetadot"].get<double>();
        INFO("s = " << s.transpose() << ", q = " << q.transpose() << ", h = " << h << ", thetadot = " << td);
        const lab::Screw r = lab::twist_to_screw(lab::screw_to_twist(q, s, h, td));
        CHECK_SCALAR_NEAR(r.h, h, isclose_tol(h));
        CHECK_SCALAR_NEAR(r.thetadot, td, isclose_tol(td));
        CHECK_MAT_NEAR(r.s_hat, s, 1e-10);
        REQUIRE(r.q.has_value());
        CHECK(lab::point_line_distance(q, *r.q, s) < 1e-10);  // same line
        CHECK(std::abs(r.q->dot(s)) < 1e-10);                // q is the point closest to the origin
    }
}

TEST_CASE("test_pure_translation_and_zero_twist") {
    Vector6d V;
    V << 0, 0, 0, 0, 0, 0.05;
    const lab::Screw sc = lab::twist_to_screw(V);
    CHECK(std::isinf(sc.h));
    CHECK_FALSE(sc.q.has_value());
    CHECK_SCALAR_NEAR(sc.thetadot, 0.05, isclose_tol(0.05));
    Vector6d S_exp;
    S_exp << 0, 0, 0, 0, 0, 1;
    CHECK_MAT_NEAR(sc.S, S_exp, 1e-12);
    // q is ignored for h = INF (Python passes None)
    CHECK_MAT_NEAR(lab::screw_to_twist(Vector3d::Zero(), Vector3d(0, 0, 2), lab::INF, 0.05), V, 1e-12);
    CHECK_THROWS_AS(lab::twist_to_screw(Vector6d::Zero()), std::invalid_argument);
}

TEST_CASE("test_pitch_is_frame_invariant") {
    const json& c = cases()["pitch_is_frame_invariant"];
    const Vector6d V = as<Vector6d>(c["V"]);
    const double h = lab::twist_to_screw(V).h;
    CHECK_SCALAR_NEAR(h, c["h"].get<double>(), isclose_tol(c["h"].get<double>()));
    for (int i = 0; i < 5; ++i) {
        const Vector6d AdV = mr::adjoint(random_T()[static_cast<std::size_t>(i)]) * V;
        CHECK_SCALAR_NEAR(lab::twist_to_screw(AdV).h, h, isclose_tol(h));
    }
}

// ------------------------------------------------------------- Checkpoint 4: transform tree
TEST_CASE("test_tree_mr_example_3_19") {
    lab::TransformTree tree;
    tree.add("a", "d", T_AD);
    tree.add("d", "b", T_DB);
    tree.add("d", "e", T_DE);
    tree.add("b", "c", T_BC);
    CHECK_MAT_NEAR(tree.lookup("c", "e"), T_CE_MR, 1e-9);
    const Matrix4d T_ae = T_AD * T_DE, T_ac = T_AD * T_DB * T_BC;
    CHECK_MAT_NEAR(tree.lookup("a", "e"), T_ae, 1e-9);
    CHECK_MAT_NEAR(tree.lookup("a", "c"), T_ac, 1e-9);
    CHECK_MAT_NEAR(tree.lookup("e", "c"), mr::trans_inv(T_CE_MR), 1e-9);
    CHECK_MAT_NEAR(tree.lookup("b", "b"), Matrix4d::Identity(), 1e-12);
    CHECK(tree.path("c", "e") == std::vector<std::string>{"c", "b", "d", "e"});
    CHECK_MAT_NEAR(tree.lookup("c", "e"), as<Matrix4d>(cases()["tree_mr_example_3_19"]["T_ce"]), 1e-9);
}

TEST_CASE("test_tree_rejects_bad_edges") {
    const std::vector<Matrix4d>& R = random_T();
    lab::TransformTree tree;
    tree.add("world", "a", R[0]);
    tree.add("a", "b", R[1]);
    tree.add("a", "b", R[2]);  // update is allowed
    CHECK_MAT_NEAR(tree.lookup("a", "b"), R[2], 1e-12);
    CHECK_THROWS_AS(tree.add("world", "b", R[3]), std::invalid_argument);   // second parent
    CHECK_THROWS_AS(tree.add("b", "world", R[3]), std::invalid_argument);   // cycle
    const Matrix4d twice = 2.0 * R[3];
    CHECK_THROWS_AS(tree.add("b", "c", twice), std::invalid_argument);      // not in SE(3)
    CHECK_THROWS_AS(tree.lookup("world", "nowhere"), lab::KeyError);
    tree.add("other_root", "z", R[4]);
    CHECK_THROWS_AS(tree.lookup("world", "z"), lab::LookupError);
}

TEST_CASE("test_quaternion_order_and_json_loader") {
    const lab::TransformTree nav = load_nav();
    for (const auto& c : cases()["quaternion_order_and_json_loader"]) {  // scipy Rotation.random(random_state=s)
        const Vector4d q = as<Vector4d>(c["q_xyzw"]);
        INFO("q_xyzw = " << q.transpose());
        CHECK_MAT_NEAR(lab::quat_xyzw_to_rot(q), as<Matrix3d>(c["R"]), 1e-12);
    }
    const std::set<std::string> frames = nav.frames();
    for (const char* f : {"theatre", "tracker", "patient_ref", "ct_image", "plan_entry", "tool_marker", "tool_tip"}) {
        INFO("frame " << f);
        CHECK(frames.count(f) == 1);
    }
}

TEST_CASE("test_eod_tree_matches_workshop_problem_3") {
    const lab::TransformTree eod = lab::load_tree_json(sbeg::lab_path("../eod_tree.json"));
    const Matrix4d T_ce = eod.lookup("gripper", "package");
    const Vector3d p_ce = T_ce.topRightCorner<3, 1>();
    const Matrix3d R_ce = T_ce.topLeftCorner<3, 3>();
    CHECK_MAT_NEAR(p_ce, Vector3d(0.1, -0.1, 0.5), 1e-9);
    CHECK_MAT_NEAR(R_ce, mat3(0, 0, -1, 0, 1, 0, 1, 0, 0), 1e-9);
    const Vector3d p_bp = eod.lookup("base_link", "package").topRightCorner<3, 1>();
    CHECK_MAT_NEAR(p_bp, Vector3d(1.4, -0.1, 0.3), 1e-9);
}

// ------------------------------------------------------------- Checkpoint 5: surgical navigation
TEST_CASE("test_tool_tip_in_ct_at_start") {
    lab::TransformTree nav = load_nav();
    const std::vector<Matrix4d>& Ts = track().second;
    REQUIRE(Ts.size() > 1);
    nav.add("tracker", "tool_marker", Ts[0]);
    const Matrix4d T = nav.lookup("ct_image", "tool_tip");
    const Matrix4d direct = mr::trans_inv(nav.lookup("tracker", "ct_image")) * Ts[0] * nav.lookup("tool_marker", "tool_tip");
    CHECK_MAT_NEAR(T, direct, 1e-12);
    const Vector3d tip_mm = T.topRightCorner<3, 1>() * 1000.0;
    CHECK_MAT_NEAR(tip_mm, Vector3d(142.0, 84.035, 210.262), 1e-2);  // mm
    const Vector3d tip_plan = nav.lookup("plan_entry", "tool_tip").topRightCorner<3, 1>() * 1000.0;
    CHECK_SCALAR_NEAR(tip_plan.norm(), 1.0, isclose_tol(1.0, 1e-5, 1e-3));  // 1.0 mm entry error
}

TEST_CASE("test_body_twist_from_tracking_data") {
    const Eigen::VectorXd& t = track().first;
    const std::vector<Matrix4d>& Ts = track().second;
    REQUIRE(Ts.size() > 2);
    const double dt = t(1) - t(0);
    const auto [Vs, Vb] = lab::twists_from_samples(Ts, dt);
    const Eigen::Index n = static_cast<Eigen::Index>(Ts.size()) - 2;
    REQUIRE(Vs.rows() == n);
    REQUIRE(Vs.cols() == 6);
    REQUIRE(Vb.rows() == n);
    REQUIRE(Vb.cols() == 6);
    const Matrix4d step = mr::trans_inv(Ts[0]) * Ts[1];
    const Vector6d exact = mr::se3_to_vec(mr::matrix_log6(step)) / dt;  // constant screw motion
    CHECK_MAT_NEAR(exact, as<Vector6d>(cases()["body_twist_from_tracking_data"]["exact"]), 1e-9);
    const MatrixXd tiled = exact.transpose().replicate(n, 1);
    CHECK_MAT_NEAR(Vb, tiled, 5e-4);
    for (Eigen::Index k : {Eigen::Index(0), Eigen::Index(150), n - 1}) {  // V_s = [Ad_Tsb] V_b
        INFO("k = " << k);
        const Vector6d vb = Vb.row(k).transpose();
        const Vector6d expected = mr::adjoint(Ts[static_cast<std::size_t>(k + 1)]) * vb;
        CHECK_MAT_NEAR(Vs.row(k), expected, 1e-9);
    }
}

TEST_CASE("test_screw_axis_and_lead_of_the_screwdriver") {
    const lab::TransformTree nav = load_nav();
    const Eigen::VectorXd& t = track().first;
    const std::vector<Matrix4d>& Ts = track().second;
    REQUIRE(Ts.size() > 2);
    const auto [Vs, Vb] = lab::twists_from_samples(Ts, t(1) - t(0));
    REQUIRE(Vs.rows() > 0);
    REQUIRE(Vs.cols() == 6);
    REQUIRE(Vb.rows() > 0);
    REQUIRE(Vb.cols() == 6);
    const Vector6d Vb_mean = Vb.colwise().mean().transpose();
    const Vector6d Vs_mean = Vs.colwise().mean().transpose();
    const lab::Screw sc_b = lab::twist_to_screw(Vb_mean);
    CHECK_SCALAR_NEAR(sc_b.h * 2 * PI * 1000, 2.5, isclose_tol(2.5, 1e-5, 0.01));  // lead in mm/rev
    CHECK_SCALAR_NEAR(sc_b.thetadot, PI / 2, isclose_tol(PI / 2, 1e-3));
    CHECK_MAT_NEAR(sc_b.s_hat, Vector3d(0, 0, -1), 1e-6);
    REQUIRE(sc_b.q.has_value());
    CHECK_MAT_NEAR(*sc_b.q, Vector3d(0.05, 0, 0), 1e-6);
    const Vector6d V_ct = lab::adjoint(nav.lookup("ct_image", "tracker")) * Vs_mean;
    const lab::Screw sc = lab::twist_to_screw(V_ct);
    REQUIRE(sc.q.has_value());
    const Matrix4d T_plan = nav.lookup("ct_image", "plan_entry");
    const double angle_deg = lab::axis_angle_between(sc.s_hat, T_plan.block<3, 1>(0, 2)) * 180.0 / PI;
    CHECK_SCALAR_NEAR(angle_deg, 2.0, isclose_tol(2.0, 1e-5, 0.01));
    const double dist_mm = lab::point_line_distance(T_plan.topRightCorner<3, 1>(), *sc.q, sc.s_hat) * 1000;
    CHECK_SCALAR_NEAR(dist_mm, 1.0, isclose_tol(1.0, 1e-5, 0.01));
}
