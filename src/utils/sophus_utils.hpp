#pragma once

#include <Eigen/Core>
#include <sophus/so3.hpp>

namespace omni_slam::SophusUtils {

template <typename Derived>
inline Sophus::Matrix<typename Derived::Scalar, 3, 3> SO3RightJacobian(
  const Eigen::MatrixBase<Derived>& omega) {
  static_assert(Derived::RowsAtCompileTime == 3
                  && Derived::ColsAtCompileTime == 1,
                "SO3RightJacobian expects a 3x1 vector");
  using Scalar = typename Derived::Scalar;
  // Jr(omega) = Jl(-omega)
  return Sophus::SO3<Scalar>::leftJacobian(-omega.template cast<Scalar>());
}

template <typename Derived>
inline Sophus::Matrix<typename Derived::Scalar, 3, 3> SO3RightJacobianInverse(
  const Eigen::MatrixBase<Derived>& omega) {
  static_assert(Derived::RowsAtCompileTime == 3
                  && Derived::ColsAtCompileTime == 1,
                "SO3RightJacobianInverse expects a 3x1 vector");
  using Scalar = typename Derived::Scalar;
  // Jr^{-1}(omega) = Jl^{-1}(-omega)
  return Sophus::SO3<Scalar>::leftJacobianInverse(
    -omega.template cast<Scalar>());
}

}  // namespace omni_slam::SophusUtils
