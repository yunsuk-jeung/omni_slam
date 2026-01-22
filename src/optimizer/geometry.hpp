#pragma once

#include <sophus/se3.hpp>

namespace omni_slam {
class Geometry {
public:
  static Eigen::Matrix<double, 4, 1> triangulate(const Eigen::Vector3d& r0,
                                                 const Eigen::Vector3d& r1,
                                                 const Sophus::SE3d&    T_0_1) {
    Eigen::Matrix<double, 3, 4> P1, P2;
    P1.setIdentity();
    P2 = T_0_1.inverse().matrix3x4();

    Eigen::Matrix<double, 4, 4> A(4, 4);
    A.row(0) = r0[0] * P1.row(2) - r0[2] * P1.row(0);
    A.row(1) = r0[1] * P1.row(2) - r0[2] * P1.row(1);
    A.row(2) = r1[0] * P2.row(2) - r1[2] * P2.row(0);
    A.row(3) = r1[1] * P2.row(2) - r1[2] * P2.row(1);

    Eigen::JacobiSVD<Eigen::Matrix<double, 4, 4>> mySVD(A, Eigen::ComputeFullV);
    Eigen::Vector4d                               worldPoint = mySVD.matrixV().col(3);
    worldPoint /= worldPoint.template head<3>().norm();

    // Enforce same direction of bearing vector and initial point
    if (r0.dot(worldPoint.template head<3>()) < 0)
      worldPoint *= -1;

    return worldPoint;
  }
};
}  // namespace omni_slam