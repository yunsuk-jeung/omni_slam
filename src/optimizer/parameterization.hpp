#pragma once

#include <ceres/manifold.h>
#include <sophus/se3.hpp>

#include "utils/eigen_utils.hpp"

namespace omni_slam {

// Boxplus update: t += dt, R = Exp(dtheta) * R
class SE3BoxplusManifold final : public ceres::Manifold {
public:
  static constexpr int kAmbientSize = 6;
  static constexpr int kTangentSize = 6;

  int AmbientSize() const override { return kAmbientSize; }
  int TangentSize() const override { return kTangentSize; }

  bool Plus(const double* x, const double* delta, double* x_plus_delta) const override {
    Eigen::Map<const Eigen::Vector3d> t(x);
    Eigen::Map<const Eigen::Vector3d> so3(x + 3);
    Eigen::Map<const Eigen::Vector3d> dt(delta);
    Eigen::Map<const Eigen::Vector3d> dtheta(delta + 3);

    const Sophus::SO3d R     = Sophus::SO3d::exp(so3);
    const Sophus::SO3d R_new = Sophus::SO3d::exp(dtheta) * R;

    Eigen::Map<Eigen::Vector3d> t_out(x_plus_delta);
    Eigen::Map<Eigen::Vector3d> so3_out(x_plus_delta + 3);
    t_out   = t + dt;
    so3_out = R_new.log();
    return true;
  }

  bool Minus(const double* y, const double* x, double* y_minus_x) const override {
    Eigen::Map<const Eigen::Vector3d> t_y(y);
    Eigen::Map<const Eigen::Vector3d> so3_y(y + 3);
    Eigen::Map<const Eigen::Vector3d> t_x(x);
    Eigen::Map<const Eigen::Vector3d> so3_x(x + 3);

    const Sophus::SO3d R_y = Sophus::SO3d::exp(so3_y);
    const Sophus::SO3d R_x = Sophus::SO3d::exp(so3_x);

    Eigen::Map<Eigen::Vector3d> dt(y_minus_x);
    Eigen::Map<Eigen::Vector3d> dtheta(y_minus_x + 3);
    dt     = t_y - t_x;
    dtheta = (R_y * R_x.inverse()).log();
    return true;
  }

  static Sophus::SE3d FromParams(const double* x) {
    Eigen::Map<const Eigen::Vector3d> t(x);
    Eigen::Map<const Eigen::Vector3d> so3(x + 3);
    return Sophus::SE3d(Sophus::SO3d::exp(so3), t);
  }

  static Eigen::Vector6d ToParams(const Sophus::SE3d& T) {
    Eigen::Vector6d out;
    out.template head<3>() = T.translation();
    out.template tail<3>() = T.so3().log();
    return out;
  }
};

}  // namespace omni_slam
