// SBEG 361 Lab 3 -- Homogeneous transforms, transform trees and twists (C++ STARTER).
//
// Fill in every function marked  // TODO  (replace the `throw std::logic_error("TODO: ...")` line).
// Do not change function names or signatures (they are declared in lab.hpp). Build and test from the cpp folder:
//     cmake -S . -B build -G Ninja && cmake --build build && ./build/lab_tests
// or from your Lab code folder:   python run_lab.py 3 --cpp
//
// You may NOT use mrcpp (the C++ port of modern_robotics) or Eigen's general .inverse() inside these functions --
// they are what the tests compare against. The helpers that are already complete (vec_to_so3, so3_to_vec,
// frames, _ancestors, _common, path, the loaders and the analysis helpers) may be used freely.
// Notation follows Lynch & Park, *Modern Robotics* (MR), Sections 3.3.1-3.3.2 (see lab.hpp).
//
// Eigen reminders:  T.topLeftCorner<3, 3>() (= T[:3, :3]),  T.topRightCorner<3, 1>() (= T[:3, 3]),
//   T.row(3), V.head<3>() (= V[:3]), V.tail<3>() (= V[3:]), a.cross(b), a.dot(b), v.norm(),
//   Eigen::Matrix4d::Identity(), Matrix6d::Zero(), A.block<3, 3>(3, 0) (= A[3:, :3]),
//   Vector6d V;  V << w, v;  (stack two 3-vectors).  Write the type of every Eigen result
//   (Eigen::Matrix4d M = A * B;), never `auto M = A * B;`.
#include "lab.hpp"

#include <sbeg/labio.hpp>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <stdexcept>

namespace lab {

// ---------------------------------------------------------------- helpers reused from Lab 2 (given)
Eigen::Matrix3d vec_to_so3(const Eigen::Vector3d& omega) {
    Eigen::Matrix3d m;
    m << 0.0, -omega(2), omega(1),
         omega(2), 0.0, -omega(0),
         -omega(1), omega(0), 0.0;
    return m;
}

Eigen::Vector3d so3_to_vec(const Eigen::Matrix3d& so3mat) {
    return Eigen::Vector3d(so3mat(2, 1), so3mat(0, 2), so3mat(1, 0));
}

Eigen::Matrix3d quat_xyzw_to_rot(const Eigen::Vector4d& q_xyzw) {
    // TODO: reorder to (q0,q1,q2,q3) = (w,x,y,z), normalise, then use MR Eq. B.12 (your Lab 2 quat_to_rot)
    //       q_xyzw(3) is the scalar w; q_xyzw.norm() is the norm
    throw std::logic_error("TODO: quat_xyzw_to_rot");
}

// ---------------------------------------------------------------- SE(3) basics
Eigen::Matrix4d rp_to_trans(const Eigen::Matrix3d& R, const Eigen::Vector3d& p) {
    // TODO: start from Eigen::Matrix4d::Identity() and fill the R block and the p column
    throw std::logic_error("TODO: rp_to_trans");
}

std::pair<Eigen::Matrix3d, Eigen::Vector3d> trans_to_rp(const Eigen::Matrix4d& T) {
    // TODO: return {top-left 3x3 block, top-right 3x1 column}
    throw std::logic_error("TODO: trans_to_rp");
}

Eigen::Matrix4d trans_inv(const Eigen::Matrix4d& T) {
    // TODO: use MR Eq. 3.64 -- do NOT call .inverse()
    throw std::logic_error("TODO: trans_inv");
}

bool is_se3(const Eigen::MatrixXd& T, double tol) {
    // TODO: return false for a size other than 4x4 (T.rows(), T.cols()); then check R^T R = I,
    //       det R = +1 and the last row (0, 0, 0, 1) within tol (Python: np.allclose with atol=tol)
    throw std::logic_error("TODO: is_se3");
}

Eigen::Vector3d transform_point(const Eigen::Matrix4d& T, const Eigen::Vector3d& x) {
    // TODO: R x + p
    throw std::logic_error("TODO: transform_point");
}

Eigen::MatrixXd transform_points(const Eigen::Matrix4d& T, const Eigen::MatrixXd& X) {
    // TODO: one point per ROW of X: X R^T + p on every row (Python: x @ R.T + p);
    //       Y.rowwise() += p.transpose() adds p to every row
    throw std::logic_error("TODO: transform_points");
}

// ---------------------------------------------------------------- se(3) and the adjoint
Eigen::Matrix4d vec_to_se3(const Vector6d& V) {
    // TODO: top-left block [omega] (vec_to_so3), top-right column v, bottom row zeros
    throw std::logic_error("TODO: vec_to_se3");
}

Vector6d se3_to_vec(const Eigen::Matrix4d& se3mat) {
    // TODO: inverse of vec_to_se3
    throw std::logic_error("TODO: se3_to_vec");
}

Matrix6d adjoint(const Eigen::Matrix4d& T) {
    // TODO: MR Def. 3.20: blocks R, 0, [p]R, R
    throw std::logic_error("TODO: adjoint");
}

// ---------------------------------------------------------------- twists
std::pair<Vector6d, Vector6d> twists_from_T_Tdot(const Eigen::Matrix4d& T, const Eigen::Matrix4d& Tdot) {
    // TODO: [V_s] = Tdot T^-1, [V_b] = T^-1 Tdot (MR Eqs. 3.81-3.82); use trans_inv and se3_to_vec; return {V_s, V_b}
    throw std::logic_error("TODO: twists_from_T_Tdot");
}

std::pair<Eigen::MatrixXd, Eigen::MatrixXd> twists_from_samples(const std::vector<Eigen::Matrix4d>& Ts, double dt) {
    // TODO: central difference Tdot_k = (T_{k+1}-T_{k-1})/(2 dt); read omega from the skew part (M - M^T)/2;
    //       return two (N-2) x 6 matrices {Vs, Vb}, row k-1 for sample k
    throw std::logic_error("TODO: twists_from_samples");
}

Vector6d screw_to_twist(const Eigen::Vector3d& q, const Eigen::Vector3d& s_hat, double h, double thetadot) {
    // TODO: normalise s_hat; h = INF (std::isinf(h)) -> (0, s_hat*thetadot); else (s_hat, -s_hat x q + h s_hat)*thetadot
    throw std::logic_error("TODO: screw_to_twist");
}

Screw twist_to_screw(const Vector6d& V) {
    // TODO: case |omega| > TOL: thetadot = |omega|, h = omega.v/|omega|^2, q = omega x v/|omega|^2;
    //       case omega = 0: h = INF, leave q empty (sc.q.reset()); zero twist -> throw std::invalid_argument
    //       Screw sc;  sc.S = ...;  sc.thetadot = ...;  sc.h = ...;  sc.s_hat = ...;  sc.q = ...;  return sc;
    throw std::logic_error("TODO: twist_to_screw");
}

// ---------------------------------------------------------------- transform tree (tf2-like)
std::set<std::string> TransformTree::frames() const {
    std::set<std::string> f;
    for (const auto& [c, p] : parent) {
        f.insert(c);
        f.insert(p);
    }
    return f;
}

void TransformTree::add(const std::string& parent_frame, const std::string& child,
                        const Eigen::Matrix4d& T_parent_child) {
    // TODO: validate (parent_frame != child, is_se3, single parent, no cycle via _ancestors(parent_frame)) and
    //       throw std::invalid_argument on failure; then store parent[child] = parent_frame; T[child] = T_parent_child;
    throw std::logic_error("TODO: add");
}

std::vector<std::string> TransformTree::_ancestors(const std::string& frame) const {
    std::vector<std::string> chain{frame};
    for (auto it = parent.find(chain.back()); it != parent.end(); it = parent.find(chain.back()))
        chain.push_back(it->second);
    return chain;
}

Eigen::Matrix4d TransformTree::_to_ancestor(const std::string& frame, const std::string& ancestor) const {
    // TODO: walk up from frame to ancestor, left-multiplying by T.at(f) (subscript cancellation); f = parent.at(f)
    throw std::logic_error("TODO: _to_ancestor");
}

std::string TransformTree::_common(const std::string& target, const std::string& source) const {
    const std::set<std::string> all = frames();
    for (const std::string& f : {target, source})
        if (all.count(f) == 0) throw KeyError("unknown frame '" + f + "'");
    const std::vector<std::string> up_s = _ancestors(source);
    for (const std::string& f : _ancestors(target))
        if (std::find(up_s.begin(), up_s.end(), f) != up_s.end()) return f;
    throw LookupError("frames '" + target + "' and '" + source + "' are not connected");
}

std::vector<std::string> TransformTree::path(const std::string& target, const std::string& source) const {
    const std::string common = _common(target, source);
    const std::vector<std::string> up_t = _ancestors(target), up_s = _ancestors(source);
    std::vector<std::string> out;
    for (const std::string& f : up_t) {
        out.push_back(f);
        if (f == common) break;
    }
    const auto it = std::find(up_s.begin(), up_s.end(), common);
    for (auto r = std::make_reverse_iterator(it); r != up_s.rend(); ++r) out.push_back(*r);
    return out;
}

Eigen::Matrix4d TransformTree::lookup(const std::string& target, const std::string& source) const {
    // TODO: common = _common(target, source); combine two _to_ancestor results (one inverted with trans_inv)
    throw std::logic_error("TODO: lookup");
}

Eigen::Matrix4d transform_from_tf(const Eigen::Vector3d& translation, const Eigen::Vector4d& rotation_xyzw) {
    return rp_to_trans(quat_xyzw_to_rot(rotation_xyzw), translation);
}

TransformTree load_tree_json(const std::string& path) {
    const sbeg::json data = sbeg::load_json(path);
    TransformTree tree;
    for (const auto& e : data.at("transforms"))
        tree.add(e.at("parent").get<std::string>(), e.at("child").get<std::string>(),
                 transform_from_tf(sbeg::as<Eigen::Vector3d>(e.at("translation")),
                                   sbeg::as<Eigen::Vector4d>(e.at("rotation_xyzw"))));
    return tree;
}

std::pair<Eigen::VectorXd, std::vector<Eigen::Matrix4d>> load_track_csv(const std::string& path) {
    const sbeg::CsvTable d = sbeg::read_csv(path);  // skips the header line
    const Eigen::VectorXd t = d.data.col(0);
    std::vector<Eigen::Matrix4d> Ts;
    for (Eigen::Index k = 0; k < d.data.rows(); ++k) {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) T(i, j) = d.data(k, 1 + 3 * i + j);  // r11..r33, row-major
            T(i, 3) = d.data(k, 10 + i);                                     // px, py, pz
        }
        Ts.push_back(T);
    }
    return {t, Ts};
}

// ---------------------------------------------------------------- analysis helpers
double axis_angle_between(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    const double c = a.dot(b) / (a.norm() * b.norm());
    return std::acos(std::max(-1.0, std::min(1.0, c)));
}

double point_line_distance(const Eigen::Vector3d& x, const Eigen::Vector3d& q, const Eigen::Vector3d& s_hat) {
    const Eigen::Vector3d s = s_hat / s_hat.norm();
    const Eigen::Vector3d d = x - q;
    return (d - d.dot(s) * s).norm();
}

}  // namespace lab
