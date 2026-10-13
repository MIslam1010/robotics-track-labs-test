// Robot model used in SBEG 46X Lab 11 (C++ track) -- a self-contained copy of the shared course model (numbers
// identical to course_models.py and to the course robot models in Course resources). Given code: do not edit.
//
//   srs_7r_feeding()   Wheelchair-mounted SRS 7R assistive (feeding) arm, SBEG 361 project scenario S2:
//                      shoulder z-y-x at height 0.25 m above the arm base {0}, elbow y, wrist x-y-z;
//                      upper arm 0.40 m, forearm 0.35 m, wrist-to-spoon 0.12 m.
//                      Lab 11 (optional Part D) mounts it on the powered wheelchair of data/wheelchair.json.
// ILLUSTRATIVE VALUES: teaching values, not the data of a real product.
// Notation: Modern Robotics (MR) -- screw axes S = (w, v) (MR Sec. 3.3.2), home pose M, body screws B = [Ad_{M^-1}] S.
#ifndef SBEG46X_LAB11_COURSE_MODELS_HPP
#define SBEG46X_LAB11_COURSE_MODELS_HPP

#include <sbeg/eigen.hpp>  // always first: includes Eigen/Dense with the course settings

namespace cm {

/// Screw axis S = (w, -w x q + h w) of a joint about unit axis w through point q (MR Sec. 3.3.2).
Eigen::Matrix<double, 6, 1> screw_revolute(const Eigen::Vector3d& w, const Eigen::Vector3d& q, double h = 0.0);

/// The wheelchair-mounted SRS 7R arm (Python: the dict of srs_7r_feeding()).
struct ArmModel {
    Eigen::MatrixXd Slist;     ///< 6 x 7 space screw axes, one per COLUMN
    Eigen::MatrixXd Blist;     ///< 6 x 7 body screw axes, one per COLUMN: [Ad_{M^-1}] Slist
    Eigen::Matrix4d M;         ///< home pose of the spoon frame {e} in the arm base {0}
    Eigen::MatrixXd limits;    ///< 7 x 2 joint limits (rad)
    Eigen::VectorXd qdot_max;  ///< 7 joint-speed limits (rad/s), 40 deg/s each
    double h = 0.25, Lu = 0.40, Lf = 0.35, Lt = 0.12;  ///< shoulder height, upper arm, forearm, wrist-to-spoon (m)
};

/// Shoulder z-y-x, elbow y, wrist x-y-z; shoulder height 0.25, upper arm 0.40, forearm 0.35, wrist-to-spoon 0.12 m.
ArmModel srs_7r_feeding();

}  // namespace cm

#endif  // SBEG46X_LAB11_COURSE_MODELS_HPP
