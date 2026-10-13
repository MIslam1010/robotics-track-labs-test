// Auto-checks for SBEG 361 Lab 1 (rotations, SO(3)) -- C++ track. Mirrors test_lab.py test by test.
//
// Build and run from this folder:
//     cmake -S . -B build -G Ninja && cmake --build build && ./build/lab_tests
//     ./build/lab_tests -tc="*elementary*,*axis_angle*"      (run only some tests, like pytest -k)
// Results are compared with mrcpp (the C++ port of modern_robotics) and with cpp_cases.json, which holds the
// numbers of the Python reference, so both languages are checked against the same values.
#include "lab.hpp"

#include <sbeg/labio.hpp>
#include <sbeg/mrcpp.hpp>

#include <doctest/doctest.h>

#include <cmath>
#include <set>
#include <stdexcept>
#include <tuple>
#include <vector>

using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using sbeg::as;
using sbeg::as_list;
using sbeg::json;

namespace {
constexpr double PI = 3.14159265358979323846;
const std::set<int> CORRUPTED = {4, 9, 13, 17};

const json& cases() {
    static const json C = sbeg::load_json(sbeg::lab_path("cpp_cases.json"));
    return C;
}

std::string data_file() { return sbeg::lab_path("../data/tracker_orientations.csv"); }

/// Reference rotation from mrcpp: exp([w_hat] theta).
Matrix3d mr_rot(const Vector3d& axis, double theta) { return mr::matrix_exp3(mr::vec_to_so3(axis * theta)); }

/// The tracker samples, read with the given helper lab::load_tracker_csv.
std::vector<Matrix3d> tracker_samples() { return lab::load_tracker_csv(data_file()); }
}  // namespace

// ---------------- Part A ----------------
TEST_CASE("test_elementary_rotations_match_mr") {
    for (const auto& c : cases()["elementary_rotations"]) {
        const double theta = c["theta"].get<double>();
        INFO("theta = " << theta);
        CHECK_MAT_NEAR(lab::rot_x(theta), mr_rot(Vector3d::UnitX(), theta), 1e-12);
        CHECK_MAT_NEAR(lab::rot_y(theta), mr_rot(Vector3d::UnitY(), theta), 1e-12);
        CHECK_MAT_NEAR(lab::rot_z(theta), mr_rot(Vector3d::UnitZ(), theta), 1e-12);
        CHECK_MAT_NEAR(lab::rot_y(theta), as<Matrix3d>(c["rot_y"]), 1e-12);
        CHECK_MAT_NEAR(lab::rot_z(theta), as<Matrix3d>(c["rot_z"]), 1e-12);
    }
}

TEST_CASE("test_axis_angle_matches_mr") {
    for (const auto& c : cases()["axis_angle"]) {
        const Vector3d w = as<Vector3d>(c["omega_hat"]);
        const double th = c["theta"].get<double>();
        CHECK_MAT_NEAR(lab::rot_axis_angle(w, th), mr_rot(w, th), 1e-12);
        CHECK_MAT_NEAR(lab::rot_axis_angle(w, th), as<Matrix3d>(c["R"]), 1e-12);
    }
}

TEST_CASE("test_axis_angle_identities") {
    const Vector3d w = Vector3d(1.0, 2.0, 2.0) / 3.0;
    CHECK_MAT_NEAR(lab::rot_axis_angle(w, 0.7), lab::rot_axis_angle(-w, -0.7), 1e-12);  // MR p. 72
    CHECK_MAT_NEAR(lab::rot_axis_angle(w, 0.0), Matrix3d::Identity(), 1e-12);
    CHECK_MAT_NEAR(lab::rot_axis_angle(w, 0.9) * w, w, 1e-12);  // axis is unchanged
}

TEST_CASE("test_axis_angle_rejects_non_unit") {
    CHECK_THROWS_AS(lab::rot_axis_angle(Vector3d(1.0, 1.0, 0.0), 0.5), std::invalid_argument);
}

// ---------------- Part B ----------------
TEST_CASE("test_so3_errors_values") {
    auto [e, d] = lab::so3_errors(Matrix3d::Identity());
    CHECK_APPROX(e, 0.0);  // Python: e == pytest.approx(0.0)
    CHECK_APPROX(d, 1.0);
    std::tie(e, d) = lab::so3_errors(2.0 * Matrix3d::Identity());
    CHECK_APPROX(e, std::sqrt(27.0));
    CHECK_APPROX(d, 8.0);
    CHECK_THROWS_AS(lab::so3_errors(Matrix2d::Identity()), std::invalid_argument);
    for (const auto& c : cases()["so3_errors"]) {
        const auto [e2, d2] = lab::so3_errors(as<Matrix3d>(c["R"]));
        CHECK_SCALAR_NEAR(e2, c["orth_err"].get<double>(), 1e-12);
        CHECK_SCALAR_NEAR(d2, c["det"].get<double>(), 1e-12);
    }
}

TEST_CASE("test_is_rotation_cases") {
    Matrix3d Rc, swap, mr_example;
    Rc << 0, -1, 0, 0, 0, -1, 1, 0, 0;    // MR Fig. 3.7
    swap << 0, 1, 0, 1, 0, 0, 0, 0, 1;    // det = -1
    mr_example << 1.0, 0.0, 0.0, 0.0, 0.1, -0.95, 0.0, 1.0, 0.1;  // MR library docstring
    CHECK(lab::is_rotation(Rc));
    CHECK(mr::test_if_so3(Rc));
    CHECK_FALSE(lab::is_rotation(swap));
    CHECK_FALSE(mr::test_if_so3(swap));
    CHECK_FALSE(lab::is_rotation(1.05 * Rc));
    CHECK_FALSE(lab::is_rotation(Matrix2d::Identity()));
    CHECK_FALSE(lab::is_rotation(mr_example));
    CHECK_FALSE(mr::test_if_so3(mr_example));
}

TEST_CASE("test_group_properties") {
    // Closure, inverse = transpose, non-commutativity, length preservation (MR Props. 3.3-3.6).
    for (const auto& c : cases()["group_properties"]) {
        const Matrix3d A = as<Matrix3d>(c["A"]), B = as<Matrix3d>(c["B"]);
        const Vector3d x = as<Vector3d>(c["x"]);
        CHECK(lab::is_rotation(A * B));
        CHECK_MAT_NEAR(lab::rot_inv(A), mr::rot_inv(A), 1e-12);
        CHECK_MAT_NEAR(lab::rot_inv(A), as<Matrix3d>(c["rot_inv_A"]), 1e-12);
        CHECK_MAT_NEAR(lab::rot_inv(A) * A, Matrix3d::Identity(), 1e-12);
        CHECK_APPROX((A * x).norm(), x.norm());
    }
    const Matrix3d X = lab::rot_x(PI / 2), Y = lab::rot_y(PI / 2);
    CHECK_FALSE(sbeg::compare_near(X * Y, Y * X, 1e-8).ok);  // rotations about different axes do not commute
    const Matrix3d Z1 = lab::rot_z(0.4), Z2 = lab::rot_z(1.1);
    CHECK_MAT_NEAR(Z1 * Z2, Z2 * Z1, 1e-12);  // same-axis (planar) rotations commute
}

TEST_CASE("test_project_to_so3") {
    Matrix3d M;
    M << 0.675, 0.150, 0.720, 0.370, 0.771, -0.511, -0.630, 0.619, 0.472;
    CHECK_MAT_NEAR(lab::project_to_so3(M), mr::project_to_so3(M), 1e-8);
    const Matrix3d reflect = Eigen::Vector3d(1.0, 1.0, -1.0).asDiagonal();
    const Matrix3d R = lab::project_to_so3(reflect);
    CHECK(lab::is_rotation(R));
    for (const auto& c : cases()["project_to_so3"])
        CHECK_MAT_NEAR(lab::project_to_so3(as<Matrix3d>(c["M"])), as<Matrix3d>(c["R"]), 1e-9);
}

// ---------------- Part C ----------------
TEST_CASE("test_mr_figure_3_7") {
    Matrix3d Rb, Rc;
    Rb << 0, -1, 0, 1, 0, 0, 0, 0, 1;
    Rc << 0, -1, 0, 0, 0, -1, 1, 0, 0;
    const Vector3d pa(1.0, 1.0, 0.0);
    CHECK_MAT_NEAR(lab::change_frame(lab::rot_inv(Rb), pa), Vector3d(1, -1, 0), 1e-12);
    CHECK_MAT_NEAR(lab::change_frame(lab::rot_inv(Rc), pa), Vector3d(0, -1, -1), 1e-12);
    CHECK_MAT_NEAR(Rb, lab::rot_z(PI / 2), 1e-12);
    CHECK_MAT_NEAR(lab::rotate_in_body(Rb, lab::rot_y(-PI / 2)), Rc, 1e-12);
    CHECK_MAT_NEAR(lab::rotate_in_fixed(Rb, lab::rot_x(PI / 2)), Rc, 1e-12);
}

TEST_CASE("test_eod_base_pan_and_wrist") {
    // Defence checkpoint: EOD robot gripper, base yaw (fixed z_s) vs wrist roll (body x_b).
    const Matrix3d R_sb = lab::rot_z(PI / 6) * lab::rot_y(PI / 4);
    const Matrix3d pan = lab::rot_z(PI / 2);
    const Matrix3d roll = lab::rot_x(PI / 3);
    CHECK_MAT_NEAR(lab::rotate_in_fixed(R_sb, pan), pan * R_sb, 1e-12);
    CHECK_MAT_NEAR(lab::rotate_in_body(R_sb, roll), R_sb * roll, 1e-12);
    // roll about the gripper's own x axis leaves that axis (first column) unchanged
    CHECK_MAT_NEAR(lab::rotate_in_body(R_sb, roll).col(0), R_sb.col(0), 1e-12);
    const json& c = cases()["eod_base_pan_and_wrist"];
    CHECK_MAT_NEAR(lab::rotate_in_fixed(R_sb, pan), as<Matrix3d>(c["fixed"]), 1e-12);
    CHECK_MAT_NEAR(lab::rotate_in_body(R_sb, roll), as<Matrix3d>(c["body"]), 1e-12);
}

TEST_CASE("test_planar_compose") {
    const Matrix2d P = lab::rot_z(PI / 3).topLeftCorner<2, 2>();
    const Matrix2d Q = lab::rot_z(PI / 6).topLeftCorner<2, 2>();
    const auto [R, r] = lab::planar_compose(P, Vector2d(2.0, 1.0), Q, Vector2d(1.0, 0.5));
    Matrix2d R_exp;
    R_exp << 0, -1, 1, 0;
    CHECK_MAT_NEAR(R, R_exp, 1e-12);
    CHECK_MAT_NEAR(r, Vector2d(2.0 + 0.5 - std::sqrt(3.0) / 4, 1.0 + std::sqrt(3.0) / 2 + 0.25), 1e-12);
    const json& c = cases()["planar_compose"];
    CHECK_MAT_NEAR(r, as<Vector2d>(c["r"]), 1e-12);
}

// ---------------- Part D ----------------
TEST_CASE("test_tracker_classification") {
    const std::vector<Matrix3d> Rs = tracker_samples();
    REQUIRE(Rs.size() == 20);
    const std::vector<bool> ok = lab::classify_samples(Rs);
    REQUIRE(ok.size() == Rs.size());
    std::set<int> invalid;
    for (std::size_t k = 0; k < ok.size(); ++k)
        if (!ok[k]) invalid.insert(static_cast<int>(k));
    CHECK(invalid == CORRUPTED);
    for (std::size_t k = 0; k < Rs.size(); ++k) {
        INFO("sample " << k);
        CHECK(ok[k] == mr::test_if_so3(Rs[k]));
    }
    CHECK(ok == sbeg::as_bools(cases()["tracker"]["valid"]));
}

TEST_CASE("test_tracker_projection_and_tool") {
    const std::vector<Matrix3d> Rs = tracker_samples();
    const std::vector<bool> ok = lab::classify_samples(Rs);
    REQUIRE(ok.size() == Rs.size());
    std::vector<Matrix3d> valid;
    for (std::size_t k = 0; k < Rs.size(); ++k)
        if (ok[k]) valid.push_back(Rs[k]);
    const json& c = cases()["tracker"];
    const auto projected = as_list<Matrix3d>(c["projected"]);
    REQUIRE(valid.size() == projected.size());
    for (std::size_t k = 0; k < valid.size(); ++k) {
        const Matrix3d Rp = lab::project_to_so3(valid[k]);
        CHECK_MAT_NEAR(Rp, mr::project_to_so3(valid[k]), 1e-9);
        CHECK_MAT_NEAR(Rp, projected[k], 1e-9);
        CHECK(lab::so3_errors(Rp).first < 1e-12);
    }
    const Matrix3d R_mt = lab::rot_x(PI / 6);  // marker-to-tip calibration (illustrative)
    const std::vector<Matrix3d> R_st = lab::tool_orientations(valid, R_mt);
    REQUIRE(R_st.size() == valid.size());
    CHECK_MAT_NEAR(R_st[0], valid[0] * R_mt, 1e-12);
    const auto R_st_ref = as_list<Matrix3d>(c["R_st"]);
    for (std::size_t k = 0; k < R_st.size(); ++k) CHECK_MAT_NEAR(R_st[k], R_st_ref[k], 1e-12);
}
