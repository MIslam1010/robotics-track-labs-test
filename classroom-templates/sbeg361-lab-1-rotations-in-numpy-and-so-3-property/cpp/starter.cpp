// SBEG 361 Lab 1 -- Rotations with Eigen and SO(3) property tests (C++ STARTER).
//
// Fill in every function marked  // TODO  (replace the `throw std::logic_error("TODO: ...")` line).
// Do not change function names or signatures (they are declared in lab.hpp). Build and test from the cpp folder:
//     cmake -S . -B build -G Ninja && cmake --build build && ./build/lab_tests
// or from your Lab code folder:   python run_lab.py 1 --cpp
//
// Notation follows Lynch & Park, *Modern Robotics* (MR), Sections 3.1-3.2.1 (see lab.hpp).
// Use Eigen only (no mrcpp inside this file) -- the tests compare your answers with mrcpp, the C++ port of
// modern_robotics, and with numbers computed by the Python reference.
//
// Eigen reminders:  Eigen::Matrix3d R;  R << a, b, c,  d, e, f,  g, h, i;   (comma initialiser, row by row)
//   R.transpose(), R.determinant(), R.norm() (Frobenius norm), Eigen::Matrix3d::Identity(), A * B (matrix product),
//   v.norm(), R.col(0), R(i, j).
#include "lab.hpp"

#include <sbeg/labio.hpp>

#include <cmath>
#include <stdexcept>

namespace lab {

// ---------------------------------------------------------------------------
// Part A -- elementary rotations
// ---------------------------------------------------------------------------

Eigen::Matrix3d rot_x(double theta) {
    // Given as an example.
    const double c = std::cos(theta), s = std::sin(theta);
    Eigen::Matrix3d R;
    R << 1.0, 0.0, 0.0,
         0.0, c, -s,
         0.0, s, c;
    return R;
}

Eigen::Matrix3d rot_y(double theta) {
    // TODO: build the matrix; watch the sign of the sin terms (it differs from rot_x and rot_z)
    throw std::logic_error("TODO: rot_y");
}

Eigen::Matrix3d rot_z(double theta) {
    // TODO
    throw std::logic_error("TODO: rot_z");
}

Eigen::Matrix3d rot_axis_angle(const Eigen::Vector3d& omega_hat, double theta) {
    // TODO: throw std::invalid_argument("omega_hat must be a unit vector") if |omega_hat.norm() - 1| > 1e-6.
    // Hint: write c = cos(theta), s = sin(theta), v = 1 - c and fill the 9 entries.
    throw std::logic_error("TODO: rot_axis_angle");
}

// ---------------------------------------------------------------------------
// Part B -- SO(3) property tests
// ---------------------------------------------------------------------------

std::pair<double, double> so3_errors(const Eigen::MatrixXd& R) {
    // TODO: throw std::invalid_argument if R is not 3x3 (R.rows(), R.cols()).
    // orth_err = Frobenius norm of R^T R - I   (MR Eq. 3.19)   -> .norm()
    // det      = det(R)                         (MR Eq. 3.21 requires +1)
    // return {orth_err, det};
    throw std::logic_error("TODO: so3_errors");
}

bool is_rotation(const Eigen::MatrixXd& R, double tol) {
    // TODO: return false (do not throw) for a wrong size; otherwise use so3_errors.
    throw std::logic_error("TODO: is_rotation");
}

Eigen::Matrix3d rot_inv(const Eigen::Matrix3d& R) {
    // TODO: do NOT call .inverse()
    throw std::logic_error("TODO: rot_inv");
}

Eigen::Matrix3d project_to_so3(const Eigen::Matrix3d& M) {
    // TODO: Eigen::JacobiSVD<Eigen::Matrix3d> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
    //       svd.matrixU() is U and svd.matrixV() is V -- NOT V^T as in NumPy: transpose it yourself.
    //       R = U diag(1, 1, d) V^T with d = sign(det(U V^T))   (Eigen::Vector3d(1, 1, d).asDiagonal())
    throw std::logic_error("TODO: project_to_so3");
}

// ---------------------------------------------------------------------------
// Part C -- the three uses of a rotation matrix (MR Section 3.2.1.2)
// ---------------------------------------------------------------------------

Eigen::Vector3d change_frame(const Eigen::Matrix3d& R_ab, const Eigen::Vector3d& p_b) {
    // TODO
    throw std::logic_error("TODO: change_frame");
}

Eigen::Matrix3d rotate_in_fixed(const Eigen::Matrix3d& R_sb, const Eigen::Matrix3d& R) {
    // TODO
    throw std::logic_error("TODO: rotate_in_fixed");
}

Eigen::Matrix3d rotate_in_body(const Eigen::Matrix3d& R_sb, const Eigen::Matrix3d& R) {
    // TODO
    throw std::logic_error("TODO: rotate_in_body");
}

std::pair<Eigen::Matrix2d, Eigen::Vector2d> planar_compose(const Eigen::Matrix2d& P, const Eigen::Vector2d& p,
                                                           const Eigen::Matrix2d& Q, const Eigen::Vector2d& q) {
    // TODO: return {R, r};
    throw std::logic_error("TODO: planar_compose");
}

// ---------------------------------------------------------------------------
// Part D -- application: optical tracker data of a laparoscopic instrument
// ---------------------------------------------------------------------------

std::vector<Eigen::Matrix3d> load_tracker_csv(const std::string& path) {
    // Given.
    const sbeg::CsvTable t = sbeg::read_csv(path);  // skips the header line
    std::vector<Eigen::Matrix3d> Rs;
    for (Eigen::Index k = 0; k < t.data.rows(); ++k) {
        Eigen::Matrix3d R;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) R(i, j) = t.data(k, 1 + 3 * i + j);  // column 0 is the sample id
        Rs.push_back(R);
    }
    return Rs;
}

std::vector<bool> classify_samples(const std::vector<Eigen::Matrix3d>& Rs, double tol) {
    // TODO: one flag per sample (use is_rotation)
    throw std::logic_error("TODO: classify_samples");
}

std::vector<Eigen::Matrix3d> tool_orientations(const std::vector<Eigen::Matrix3d>& R_sm_list,
                                               const Eigen::Matrix3d& R_mt) {
    // TODO: which matrix goes first?  Use subscript cancellation.
    throw std::logic_error("TODO: tool_orientations");
}

}  // namespace lab
