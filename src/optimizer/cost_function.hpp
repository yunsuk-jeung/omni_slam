#pragma once

#include <Sophus/se3.hpp>
#include <ceres/ceres.h>

#include "utils/eigen_utils.hpp"

namespace omni_slam {
class PoseOnlyReprojectionCost : ceres::SizedCostFunction<2, 6> {
public:
  PoseOnlyReprojectionCost() = delete;

  PoseOnlyReprojectionCost(const Eigen::Vector2d& uv,
                           const Eigen::Vector3d& t_c_x_,
                           const Sophus::SE3d&    T_b_c)
    : uv_{uv}
    , t_c_x_{t_c_x_}
    , T_b_c_{T_b_c} {}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  virtual bool Evaluate(double const* const* params, double* Res, double** J) const {
    return true;
  }

private:
  const Eigen::Vector2d& uv_;
  const Eigen::Vector3d& t_c_x_;
  const Sophus::SE3d&    T_b_c_;
};
}  // namespace omni_slam