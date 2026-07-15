#pragma once

#include <ceres/manifold.h>
#include <sophus/se3.hpp>

#include "utils/eigen_utils.hpp"
#include "utils/sophus_utils.hpp"

namespace omni_slam {

// Boxplus update: t += dt, R = R * Exp(dtheta)
class SE3BoxplusManifold final : public ceres::Manifold {
 public:
  static constexpr int kAmbientSize = 6;
  static constexpr int kTangentSize = 6;

  int AmbientSize() const override { return kAmbientSize; }
  int TangentSize() const override { return kTangentSize; }

  bool Plus(const double* x,
            const double* delta,
            double*       x_plus_delta) const override {
    Eigen::Map<const Eigen::Vector3d> t(x);
    Eigen::Map<const Eigen::Vector3d> so3(x + 3);
    Eigen::Map<const Eigen::Vector3d> dt(delta);
    Eigen::Map<const Eigen::Vector3d> dtheta(delta + 3);

    const Sophus::SO3d R     = Sophus::SO3d::exp(so3);
    const Sophus::SO3d R_new = R * Sophus::SO3d::exp(dtheta);

    Eigen::Map<Eigen::Vector3d> t_out(x_plus_delta);
    Eigen::Map<Eigen::Vector3d> so3_out(x_plus_delta + 3);
    t_out   = t + dt;
    so3_out = R_new.log();
    return true;
  }

  bool PlusJacobian(const double* x, double* jacobian) const override {
    Eigen::Map<const Eigen::Vector3d>                        so3(x + 3);
    Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> J(jacobian);

    J.setZero();
    J.topLeftCorner<3, 3>().setIdentity();  // ∂t_new/∂dt = I
    J.bottomRightCorner<3, 3>() =
      SophusUtils::SO3RightJacobianInverse(so3);  // ∂so3_new/∂dtheta
    return true;
  }

  bool Minus(const double* y,
             const double* x,
             double*       y_minus_x) const override {
    Eigen::Map<const Eigen::Vector3d> t_y(y);
    Eigen::Map<const Eigen::Vector3d> so3_y(y + 3);
    Eigen::Map<const Eigen::Vector3d> t_x(x);
    Eigen::Map<const Eigen::Vector3d> so3_x(x + 3);

    const Sophus::SO3d R_y = Sophus::SO3d::exp(so3_y);
    const Sophus::SO3d R_x = Sophus::SO3d::exp(so3_x);

    Eigen::Map<Eigen::Vector3d> dt(y_minus_x);
    Eigen::Map<Eigen::Vector3d> dtheta(y_minus_x + 3);
    dt     = t_y - t_x;
    dtheta = (R_x.inverse() * R_y).log();
    return true;
  }

  bool MinusJacobian(const double* x, double* jacobian) const override {
    Eigen::Map<const Eigen::Vector3d>                        so3_x(x + 3);
    Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> J(jacobian);

    J.setZero();
    J.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity();  // ∂dt/∂t_y = I
    J.bottomRightCorner<3, 3>() =
      SophusUtils::SO3RightJacobian(so3_x);  // ∂dtheta/∂so3_y = Jr(so3_x)
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

class BearingTangentManifold : public ceres::Manifold {
 public:
  // f ∈ R^3
  int AmbientSize() const override { return 3; }
  //
  // δ ∈ R^2
  int TangentSize() const override { return 2; }

  // ---------------------------------------------
  // Plus: f ⊞ δ = Exp(B(f)δ) f
  // ---------------------------------------------
  bool Plus(const double* x,
            const double* delta,
            double*       x_plus_delta) const override {
    Eigen::Vector3d f = Eigen::Map<const Eigen::Vector3d>(x).normalized();

    // tangent basis at f
    Eigen::Matrix<double, 3, 2> B;
    EigenUtil::TangentBasis(f, B);

    // lift 2D delta → 3D tangent vector
    Eigen::Vector3d w = B * Eigen::Vector2d(delta[0], delta[1]);

    // rotate f using Sophus Exp
    Sophus::SO3d dR = Sophus::SO3d::exp(w);

    Eigen::Vector3d f_new = dR * f;

    Eigen::Map<Eigen::Vector3d> out(x_plus_delta);
    out = f_new.normalized();
    return true;
  }

  // ---------------------------------------------
  // PlusJacobian: ∂(Exp(B·δ)·f)/∂δ at δ=0
  //
  // d/dδ [Exp(Bδ)·f] = (Bδ)×f = -[f]× Bδ
  // so Jacobian = -[f]× B
  // ---------------------------------------------
  bool PlusJacobian(const double* x, double* jacobian) const override {
    Eigen::Vector3d f = Eigen::Map<const Eigen::Vector3d>(x).normalized();

    Eigen::Matrix<double, 3, 2> B;
    EigenUtil::TangentBasis(f, B);

    // ∂(Exp(B·δ)·f)/∂δ|_{δ=0} = -[f]× B
    Eigen::Map<Eigen::Matrix<double, 3, 2, Eigen::RowMajor>> J(jacobian);
    J = -Sophus::SO3d::hat(f) * B;
    return true;
  }

  bool Minus(const double* y,
             const double* x,
             double*       y_minus_x) const override {
    Eigen::Vector3d fx = Eigen::Map<const Eigen::Vector3d>(x).normalized();
    Eigen::Vector3d fy = Eigen::Map<const Eigen::Vector3d>(y).normalized();

    // Rotation vector w such that Exp(w)*fx = fy, w ⊥ fx
    Eigen::Vector3d cross     = fx.cross(fy);
    double          sin_theta = cross.norm();
    double          cos_theta = fx.dot(fy);

    Eigen::Vector3d w;
    if (sin_theta < 1e-10) {
      w = cross;  // theta/sin(theta) → 1 for small angles
    }
    else {
      double theta = std::atan2(sin_theta, cos_theta);
      w            = (theta / sin_theta) * cross;
    }

    // Project onto 2D tangent coordinates
    Eigen::Matrix<double, 3, 2> B;
    EigenUtil::TangentBasis(fx, B);

    Eigen::Map<Eigen::Vector2d> delta(y_minus_x);
    delta = B.transpose() * w;
    return true;
  }

  bool MinusJacobian(const double* x, double* jacobian) const override {
    Eigen::Vector3d f = Eigen::Map<const Eigen::Vector3d>(x).normalized();

    Eigen::Matrix<double, 3, 2> B;
    EigenUtil::TangentBasis(f, B);

    // ∂(y ⊟ x)/∂y|_{y=x} = Bᵀ [f]×
    Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> J(jacobian);
    J = B.transpose() * Sophus::SO3d::hat(f);
    return true;
  }
};

}  // namespace omni_slam
