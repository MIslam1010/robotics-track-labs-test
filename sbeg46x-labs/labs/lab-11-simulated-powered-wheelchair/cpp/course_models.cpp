// Robot model used in SBEG 46X Lab 11 (C++ track): srs_7r_feeding. Given code: do not edit (see course_models.hpp).
#include "course_models.hpp"

#include <sbeg/mrcpp.hpp>

namespace cm {

Eigen::Matrix<double, 6, 1> screw_revolute(const Eigen::Vector3d& w, const Eigen::Vector3d& q, double h) {
    Eigen::Matrix<double, 6, 1> S;
    S << w, -w.cross(q) + h * w;
    return S;
}

ArmModel srs_7r_feeding() {
    constexpr double DEG = 3.14159265358979323846 / 180.0;
    ArmModel a;
    const Eigen::Vector3d sh(0.0, 0.0, a.h), el(a.Lu, 0.0, a.h), wr(a.Lu + a.Lf, 0.0, a.h);
    const Eigen::Vector3d w[7] = {{0, 0, 1}, {0, 1, 0}, {1, 0, 0}, {0, 1, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    const Eigen::Vector3d q[7] = {sh, sh, sh, el, wr, wr, wr};
    a.Slist.resize(6, 7);
    for (int i = 0; i < 7; ++i) a.Slist.col(i) = screw_revolute(w[i], q[i]);
    a.M = mr::rp_to_trans(Eigen::Matrix3d::Identity(), wr + Eigen::Vector3d(a.Lt, 0.0, 0.0));
    a.Blist = mr::adjoint(mr::trans_inv(a.M)) * a.Slist;
    a.limits.resize(7, 2);
    a.limits << -170, 170, -120, 120, -170, 170, -150, 5, -170, 170, -115, 115, -170, 170;
    a.limits *= DEG;
    a.qdot_max = Eigen::VectorXd::Constant(7, 40 * DEG);
    return a;
}

}  // namespace cm
