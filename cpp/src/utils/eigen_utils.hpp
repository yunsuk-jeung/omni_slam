
#pragma once

#include <bitset>
#include <cstdint>

#include <Eigen/Dense>

namespace Eigen {
using Matrix66d    = Eigen::Matrix<double, 6, 6>;
using Matrix26d    = Eigen::Matrix<double, 2, 6>;
using Matrix62d    = Eigen::Matrix<double, 6, 2>;
using Matrix23d    = Eigen::Matrix<double, 2, 3>;
using Matrix36d    = Eigen::Matrix<double, 3, 6>;
using Vector6d     = Eigen::Matrix<double, 6, 1>;
using Vector9d     = Eigen::Matrix<double, 9, 1>;
using Matrix15d    = Eigen::Matrix<double, 15, 15>;
using Matrix12d    = Eigen::Matrix<double, 12, 12>;
using Matrix15x12d = Eigen::Matrix<double, 15, 12>;

}  // namespace Eigen

namespace omni_slam::EigenUtil {

// Orthonormal basis (columns of B) for the plane orthogonal to f. Falls back
// to UnitY as the pivot when f is near-parallel to UnitZ to keep the cross
// product well-conditioned.
inline void tangent_basis(const Eigen::Vector3d&       f,
                          Eigen::Matrix<double, 3, 2>& B) {
  constexpr double kNearParallelDotThreshold = 0.9;

  Eigen::Vector3d fn = f.normalized();

  Eigen::Vector3d pivot_axis = Eigen::Vector3d::UnitZ();
  if (std::fabs(fn.dot(pivot_axis)) > kNearParallelDotThreshold)
    pivot_axis = Eigen::Vector3d::UnitY();

  Eigen::Vector3d b1 = (pivot_axis.cross(fn)).normalized();
  Eigen::Vector3d b2 = fn.cross(b1);

  B.col(0) = b1;
  B.col(1) = b2;
}
}  // namespace omni_slam::EigenUtil
