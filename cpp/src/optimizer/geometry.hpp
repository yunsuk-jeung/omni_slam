#pragma once

#include <cmath>
#include <limits>

#include <sophus/se3.hpp>

namespace omni_slam {
class Geometry {
 public:
  static Eigen::Matrix<double, 4, 1> triangulate(const Eigen::Vector3d& r0,
                                                 const Eigen::Vector3d& r1,
                                                 const Sophus::SE3d&    T_1_0) {
    Eigen::Matrix<double, 3, 4> P1;
    Eigen::Matrix<double, 3, 4> P2;
    P1.setIdentity();
    P2 = T_1_0.matrix3x4();

    Eigen::Matrix<double, 4, 4> A;
    A.row(0) = r0[0] * P1.row(2) - r0[2] * P1.row(0);
    A.row(1) = r0[1] * P1.row(2) - r0[2] * P1.row(1);
    A.row(2) = r1[0] * P2.row(2) - r1[2] * P2.row(0);
    A.row(3) = r1[1] * P2.row(2) - r1[2] * P2.row(1);

    Eigen::JacobiSVD<Eigen::Matrix<double, 4, 4>> svd(A, Eigen::ComputeFullV);
    Eigen::Vector4d world_point = svd.matrixV().col(3);

    const auto invalid_result = [] {
      return Eigen::Vector4d::Constant(
        std::numeric_limits<double>::quiet_NaN());
    };

    const double w         = world_point[3];
    const double head_norm = world_point.template head<3>().norm();
    // Degenerate: point at infinity (head_norm ~ 0) or homogeneous collapse (w ~ 0).
    if (head_norm <= std::numeric_limits<double>::epsilon()
        || std::abs(w) <= std::numeric_limits<double>::epsilon()) {
      return invalid_result();
    }

    const Eigen::Vector3d p_c0 = world_point.template head<3>() / w;
    const double          dist = p_c0.norm();
    // Degenerate: point coincides with the camera center (dist ~ 0).
    if (dist <= std::numeric_limits<double>::epsilon()) {
      return invalid_result();
    }

    Eigen::Vector3d bearing  = p_c0 / dist;
    double          inv_dist = 1.0 / dist;

    // Enforce same direction of bearing vector and initial point
    if (r0.dot(bearing) < 0.0) {
      bearing *= -1.0;
      inv_dist *= -1.0;
    }

    Eigen::Vector4d bearing_inv_dist;
    bearing_inv_dist.template head<3>() = bearing;
    bearing_inv_dist[3]                 = inv_dist;

    return bearing_inv_dist;
  }
};
}  // namespace omni_slam
