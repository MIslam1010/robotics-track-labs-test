// Auto-checks for SBEG 361 Lab 2 (exponential coordinates, matrix log, quaternions, IMU) -- C++ track.
// Mirrors test_lab.py test by test (same names, same order, same tolerances).
//
// Build and run from this folder:
//     cmake -S . -B build -G Ninja && cmake --build build && ./build/lab_tests
//     ./build/lab_tests -tc="*vec_to_so3*,*skew*,*axis_ang3*"      (run only some tests, like pytest -k)
// Results are compared with mrcpp (the C++ port of modern_robotics), with Eigen's own quaternion class (as the
// Python tests use scipy.spatial.transform) and with cpp_cases.json, which holds the numbers of the Python
// reference and the random inputs drawn by test_lab.py, so both languages are checked against the same values.
#include "lab.hpp"

#include <sbeg/labio.hpp>
#include <sbeg/mrcpp.hpp>

#include <doctest/doctest.h>

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using sbeg::as;
using sbeg::json;

namespace {
constexpr double PI = 3.14159265358979323846;
constexpr double DT = 0.01;

const json& cases() {
    static const json C = sbeg::load_json(sbeg::lab_path("cpp_cases.json"));
    return C;
}

/// The IMU data file imu_knee.csv (same file as the Python tests), loaded once.
const sbeg::CsvTable& imu() {
    static const sbeg::CsvTable T = sbeg::read_csv(sbeg::lab_path("../imu_knee.csv"));
    return T;
}

/// Reference rotation from mrcpp: exp([axis] ang).
Matrix3d rot(const Vector3d& axis, double ang) { return mr::matrix_exp3(mr::vec_to_so3(axis * ang)); }

/// np.allclose(a, b, atol=atol) with NumPy's default rtol = 1e-5: every entry |a - b| <= atol + rtol * |b|.
bool allclose(const Vector4d& a, const Vector4d& b, double atol) {
    return a.allFinite() && ((a - b).cwiseAbs() - 1e-5 * b.cwiseAbs()).maxCoeff() <= atol;
}

/// q and -q represent the same rotation (MR App. B.3). Same test as Python's same_quat (np.allclose).
bool same_quat(const Vector4d& q, const Vector4d& p, double tol = 1e-9) {
    return allclose(q, p, tol) || allclose(q, Vector4d(-p), tol);
}

/// Quaternion columns <prefix>_q0..q3 as an N x 4 matrix (row k = sample k).
MatrixXd quats(const std::string& prefix) {
    const sbeg::CsvTable& T = imu();
    MatrixXd Q(T.data.rows(), 4);
    for (int i = 0; i < 4; ++i) Q.col(i) = T.data.col(T.col(prefix + "_q" + std::to_string(i)));
    return Q;
}

/// Gyro columns <prefix>_wx..wz (or _nwx..nwz) without the last (padding) row, as in Python's [:-1].
MatrixXd gyro(const std::string& prefix, bool noisy = false) {
    const sbeg::CsvTable& T = imu();
    const std::string n = noisy ? "n" : "";
    MatrixXd W(T.data.rows() - 1, 3);
    const char* comps[3] = {"x", "y", "z"};
    for (int c = 0; c < 3; ++c) W.col(c) = T.data.col(T.col(prefix + "_" + n + "w" + comps[c])).head(W.rows());
    return W;
}

/// np.testing.assert_allclose(got, expected, atol=atol) with NumPy's default rtol = 1e-7:
/// every entry must satisfy |got - expected| <= atol + rtol * |expected|.
#define CHECK_ALLCLOSE(got, expected, atol)                                                                  \
    do {                                                                                                     \
        const Matrix3d g_ = (got), e_ = (expected);                                                          \
        const bool ok_ = g_.allFinite() && ((g_ - e_).cwiseAbs() - 1e-7 * e_.cwiseAbs()).maxCoeff() <= (atol); \
        CHECK_MESSAGE(ok_, "CHECK_ALLCLOSE(" #got ", " #expected ") (rtol 1e-7, atol " << (atol) \
                                             << ")\n  " << ::sbeg::compare_near(g_, e_, (atol)).msg);          \
    } while (false)

Vector4d qrow(const MatrixXd& Q, Eigen::Index k) { return Q.row(k).transpose(); }

/// Ground-truth rotation of a scalar-first quaternion with Eigen's quaternion class (independent of the lab code;
/// the Python test uses scipy Rotation.from_quat).
Matrix3d truth_rot(const Vector4d& q) { return Eigen::Quaterniond(q(0), q(1), q(2), q(3)).normalized().toRotationMatrix(); }
}  // namespace

// ------------------------------------------------------------- Checkpoint 1: skew matrices
TEST_CASE("test_vec_to_so3_matches_mr") {
    for (const auto& c : cases()["vec_to_so3_matches_mr"]) {
        const Vector3d w = as<Vector3d>(c["w"]);
        CHECK_MAT_NEAR(lab::vec_to_so3(w), mr::vec_to_so3(w), 1e-12);
        CHECK_MAT_NEAR(lab::vec_to_so3(w), as<Matrix3d>(c["so3"]), 1e-12);
    }
}

TEST_CASE("test_skew_properties") {
    const json& c = cases()["skew_properties"];
    const Vector3d a = as<Vector3d>(c["a"]), b = as<Vector3d>(c["b"]);
    const Matrix3d A = lab::vec_to_so3(a);
    CHECK_MAT_NEAR(A, Matrix3d(-A.transpose()), 1e-12);                            // [a] = -[a]^T
    CHECK_MAT_NEAR(Vector3d(A * b), a.cross(b), 1e-12);                           // [a]b = a x b
    const Matrix3d R = as<Matrix3d>(c["R"]);
    CHECK_MAT_NEAR(Matrix3d(R * A * R.transpose()), lab::vec_to_so3(R * a), 1e-12);  // MR Eq. 3.31
    CHECK_MAT_NEAR(lab::so3_to_vec(A), a, 1e-12);
}

TEST_CASE("test_axis_ang3") {
    const Vector3d expc3(1.0, 2.0, 2.0);
    const auto [w, th] = lab::axis_ang3(expc3);
    const auto [ref_w, ref_th] = mr::axis_ang3(expc3);
    CHECK_MAT_NEAR(w, ref_w, 1e-12);
    CHECK_APPROX(th, ref_th);
    CHECK_THROWS_AS(lab::axis_ang3(Vector3d::Zero()), std::invalid_argument);
}

// ------------------------------------------------------------- Checkpoint 2: exp
TEST_CASE("test_matrix_exp3_mr_example_3_12") {
    // MR Example 3.12: w1 = (0, 0.866, 0.5), theta1 = 30 deg.
    const Vector3d w(0.0, std::sqrt(3.0) / 2, 0.5);
    const Matrix3d R = lab::matrix_exp3(lab::vec_to_so3(w * PI / 6));
    Matrix3d expected;
    expected << 0.866, -0.250, 0.433, 0.250, 0.967, 0.058, -0.433, 0.058, 0.899;
    CHECK_MAT_NEAR(R, expected, 1.5e-3);
}

TEST_CASE("test_matrix_exp3_matches_mr_and_is_rotation") {
    for (const auto& c : cases()["matrix_exp3_matches_mr_and_is_rotation"]) {
        const Matrix3d so3 = mr::vec_to_so3(as<Vector3d>(c["omega"]));
        const Matrix3d R = lab::matrix_exp3(so3);
        CHECK_MAT_NEAR(R, mr::matrix_exp3(so3), 1e-12);
        CHECK_MAT_NEAR(R, as<Matrix3d>(c["R"]), 1e-12);
        CHECK_MAT_NEAR(Matrix3d(R.transpose() * R), Matrix3d::Identity(), 1e-12);
        CHECK_APPROX(R.determinant(), 1.0);
    }
    CHECK_ALLCLOSE(lab::matrix_exp3(Matrix3d::Zero()), Matrix3d::Identity(), 0.0);  // Python default: atol 0
}

// ------------------------------------------------------------- Checkpoint 3: log
TEST_CASE("test_matrix_log3_matches_mr_random") {
    for (const auto& c : cases()["matrix_log3_matches_mr_random"]) {
        const Matrix3d R = as<Matrix3d>(c["R"]);
        CHECK_MAT_NEAR(lab::matrix_log3(R), mr::matrix_log3(R), 1e-9);
        CHECK_MAT_NEAR(lab::matrix_log3(R), as<Matrix3d>(c["log"]), 1e-9);
    }
}

TEST_CASE("test_matrix_log3_identity_and_pi_cases") {
    CHECK_ALLCLOSE(lab::matrix_log3(Matrix3d::Identity()), Matrix3d::Zero(), 0.0);  // Python default: exact zero
    for (const auto& c : cases()["matrix_log3_identity_and_pi_cases"]) {
        const Vector3d w = as<Vector3d>(c["w"]);
        INFO("axis w = " << w.transpose());
        const Matrix3d R = rot(w, PI);
        CHECK_MAT_NEAR(R, as<Matrix3d>(c["R"]), 1e-12);
        const Matrix3d L = lab::matrix_log3(R);
        const Vector3d v = lab::so3_to_vec(L);
        CHECK_APPROX(v.norm(), PI);
        CHECK_MAT_NEAR(mr::matrix_exp3(L), R, 1e-9);  // either +w or -w is fine
    }
}

TEST_CASE("test_exp_log_round_trip_small_and_large_angles") {
    for (const auto& c : cases()["exp_log_round_trip_small_and_large_angles"]) {
        const double th = c["theta"].get<double>();
        const Vector3d w = as<Vector3d>(c["w"]);
        INFO("theta = " << th);
        const Matrix3d R = mr::matrix_exp3(mr::vec_to_so3(w * th));
        const Vector3d v = lab::so3_to_vec(lab::matrix_log3(R));
        CHECK_MAT_NEAR(v, Vector3d(w * th), 1e-6);
    }
}

// ------------------------------------------------------------- Checkpoint 4: quaternions
TEST_CASE("test_rot_to_quat_matches_scipy") {
    for (const auto& c : cases()["rot_to_quat_matches_scipy"]) {
        const Matrix3d R = as<Matrix3d>(c["R"]);
        const Vector4d q = lab::rot_to_quat(R);
        INFO("R =\n" << R << "\nq = " << q.transpose() << ", reference " << as<Vector4d>(c["q"]).transpose());
        CHECK_APPROX(q.norm(), 1.0);
        CHECK(q(0) >= -1e-12);
        CHECK(same_quat(q, as<Vector4d>(c["q"]), 1e-7));
    }
}

TEST_CASE("test_quat_to_rot_round_trip") {
    for (const auto& c : cases()["quat_to_rot_round_trip"]) {
        const Matrix3d R = as<Matrix3d>(c["R"]);
        // Python: assert_allclose(..., atol=1e-12) also allows rtol = 1e-7 (needed near theta = pi)
        CHECK_ALLCLOSE(lab::quat_to_rot(lab::rot_to_quat(R)), R, 1e-12);
        CHECK_ALLCLOSE(lab::quat_to_rot(Vector4d(-lab::rot_to_quat(R))), R, 1e-12);
    }
}

TEST_CASE("test_quat_mul_is_matrix_product") {
    for (const auto& c : cases()["quat_mul_is_matrix_product"]) {
        const Matrix3d Ra = as<Matrix3d>(c["Ra"]), Rb = as<Matrix3d>(c["Rb"]);
        const Vector4d q = lab::quat_mul(lab::rot_to_quat(Ra), lab::rot_to_quat(Rb));
        INFO("q = " << q.transpose() << ", reference " << as<Vector4d>(c["q"]).transpose());
        CHECK(same_quat(q, as<Vector4d>(c["q"]), 1e-9));
        const Vector4d qc = lab::quat_conj(lab::rot_to_quat(Ra));
        CHECK_ALLCLOSE(lab::quat_to_rot(qc), Matrix3d(Ra.transpose()), 1e-12);  // atol 1e-12 + rtol 1e-7, as Python
    }
}

TEST_CASE("test_workshop_quaternion_example") {
    // Rot(x, 90 deg) Rot(y, 90 deg) -> q = (1, 1, 1, 1)/2, a 120 deg rotation about (1,1,1)/sqrt 3.
    const Vector4d qx(std::cos(PI / 4), std::sin(PI / 4), 0.0, 0.0);
    const Vector4d qy(std::cos(PI / 4), 0.0, std::sin(PI / 4), 0.0);
    CHECK_MAT_NEAR(lab::quat_mul(qx, qy), Vector4d(0.5 * Vector4d::Ones()), 1e-12);
}

// ------------------------------------------------------------- Checkpoint 5: IMU data
TEST_CASE("test_gyro_exponential_integration_reproduces_truth") {
    for (const std::string seg : {"thigh", "shank"}) {
        INFO("segment " << seg);
        const MatrixXd q = quats(seg);
        const Matrix3d R0 = lab::quat_to_rot(qrow(q, 0));
        const std::vector<Matrix3d> Rs = lab::integrate_gyro_exp(R0, gyro(seg), DT);
        REQUIRE(Rs.size() == static_cast<std::size_t>(q.rows()));
        double err = 0.0;
        for (Eigen::Index k = 0; k < q.rows(); ++k) {
            const double a = lab::rotation_angle(Rs[static_cast<std::size_t>(k)], truth_rot(qrow(q, k)));
            if (!(a <= err)) err = a;  // like std::max, but a NaN angle is kept (and fails the check)
        }
        CHECK(err < 1e-8);
        const json& c = cases()["gyro_exponential_integration_reproduces_truth"][seg];
        CHECK_MAT_NEAR(Rs.back(), as<Matrix3d>(c["R_last"]), 1e-9);
    }
}

TEST_CASE("test_euler_integration_drifts_off_SO3") {
    const MatrixXd q = quats("thigh");
    const std::vector<Matrix3d> Rs = lab::integrate_gyro_euler(lab::quat_to_rot(qrow(q, 0)), gyro("thigh"), DT);
    REQUIRE(Rs.size() == static_cast<std::size_t>(q.rows()));
    // forward Euler does not preserve orthonormality: det grows above 1
    CHECK(Rs.back().determinant() > 1.01);
    CHECK_MAT_NEAR(Rs.back(), as<Matrix3d>(cases()["euler_integration_drifts_off_SO3"]["R_last"]), 1e-9);
    const std::vector<Matrix3d> Re = lab::integrate_gyro_exp(lab::quat_to_rot(qrow(q, 0)), gyro("thigh"), DT);
    REQUIRE(Re.size() == static_cast<std::size_t>(q.rows()));
    CHECK_SCALAR_NEAR(Re.back().determinant(), 1.0, 1e-12);  // Python: pytest.approx(1.0, abs=1e-12)
}

TEST_CASE("test_knee_flexion_from_quaternions") {
    const MatrixXd qth = quats("thigh"), qsh = quats("shank");
    VectorXd kap(qth.rows());
    for (Eigen::Index k = 0; k < qth.rows(); ++k)
        kap(k) = lab::knee_flexion(lab::quat_to_rot(qrow(qth, k)), lab::quat_to_rot(qrow(qsh, k)));
    CHECK(kap.size() == cases()["knee_flexion_from_quaternions"]["n"].get<int>());
    const VectorXd kappa = imu().data.col(imu().col("kappa"));
    CHECK_MAT_NEAR(kap, kappa, 1e-8);
}

TEST_CASE("test_noisy_gyro_drift_is_bounded_but_nonzero") {
    // Bias makes integrated orientation drift (reflection question); check the order of magnitude.
    const MatrixXd q = quats("shank");
    const std::vector<Matrix3d> Rs =
        lab::integrate_gyro_exp(lab::quat_to_rot(qrow(q, 0)), gyro("shank", true), DT);
    REQUIRE(!Rs.empty());
    const double drift = lab::rotation_angle(Rs.back(), lab::quat_to_rot(qrow(q, q.rows() - 1)));
    INFO("drift = " << drift << " rad");
    CHECK(drift > 0.01);
    CHECK(drift < 0.2);
}

// ------------------------------------------------------------- Checkpoint 6: EOD gripper error
TEST_CASE("test_orientation_error_eod_gripper") {
    // Defence/EOD: gripper wrist must turn from R_cur to R_des; error in the gripper frame.
    const double deg = PI / 180.0;
    const Matrix3d R_cur = rot(Vector3d::UnitZ(), 40 * deg) * rot(Vector3d::UnitY(), -20 * deg);
    const Matrix3d R_des = R_cur * rot(Vector3d::UnitX(), 35 * deg);
    const Vector3d e = lab::orientation_error(R_cur, R_des);
    CHECK_MAT_NEAR(e, Vector3d(35 * deg, 0.0, 0.0), 1e-12);
    for (const auto& c : cases()["orientation_error_eod_gripper"]["pairs"]) {
        const Matrix3d Ra = as<Matrix3d>(c["Ra"]), Rb = as<Matrix3d>(c["Rb"]);
        const Vector3d ref = mr::so3_to_vec(mr::matrix_log3(Matrix3d(Ra.transpose() * Rb)));
        CHECK_MAT_NEAR(lab::orientation_error(Ra, Rb), ref, 1e-9);
        CHECK_MAT_NEAR(lab::orientation_error(Ra, Rb), as<Vector3d>(c["e"]), 1e-9);
        const double ang = lab::rotation_angle(Ra, Rb);
        CHECK(ang >= 0.0);
        CHECK(ang <= PI + 1e-12);
    }
}
