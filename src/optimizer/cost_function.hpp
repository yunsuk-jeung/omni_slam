#pragma once

#include <Sophus/se3.hpp>
#include <ceres/ceres.h>

#include "utils/eigen_utils.hpp"
#include "optimizer/parameterization.hpp"

namespace omni_slam {
class PoseOnlyBearingCost : public ceres::SizedCostFunction<2, 6> {
public:
  PoseOnlyBearingCost() = delete;

  PoseOnlyBearingCost(const Eigen::Vector3d& bearing,
                      const Eigen::Vector3d& p_w,
                      const Sophus::SE3d&    T_b_c)
    : b_{bearing}
    , p_w_{p_w}
    , T_b_c_{T_b_c} {}

  bool Evaluate(double const* const* params,
                double*              residuals,
                double**             jacobians) const override {
    // pose (world -> body)
    const Sophus::SE3d T_w_b = SE3BoxplusManifold::FromParams(params[0]);

    // world -> camera
    const Sophus::SE3d T_w_c = T_w_b * T_b_c_;

    // point in camera frame
    const Eigen::Vector3d p_c = T_w_c.inverse() * p_w_;

    if (p_c.norm() < 1e-8) {
      residuals[0] = residuals[1] = 0.0;
      return true;
    }

    // predicted bearing
    const Eigen::Vector3d b = p_c.normalized();

    // tangent basis at observed bearing
    Eigen::Matrix<double, 3, 2> B;
    TangentBasis(b_, B);

    // residual: 2D bearing error
    Eigen::Map<Eigen::Vector2d> r(residuals);
    r = B.transpose() * (b - b_);

    // Jacobian (optional – Ceres can numeric-diff if nullptr)
    if (jacobians && jacobians[0]) {
      Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>> J(jacobians[0]);
      J.setZero();

      // db / dp_c
      const double    inv_norm = 1.0 / p_c.norm();
      Eigen::Matrix3d J_norm   = (Eigen::Matrix3d::Identity() - b * b.transpose())
                               * inv_norm;

      // dp_c / dpose
      Eigen::Matrix<double, 3, 6> J_p_c;
      J_p_c.leftCols<3>()  = -T_w_c.so3().matrix().transpose();
      J_p_c.rightCols<3>() = T_w_c.so3().matrix().transpose()
                             * Sophus::SO3d::hat(p_w_ - T_w_c.translation());

      // chain rule
      J = B.transpose() * J_norm * J_p_c;
    }

    return true;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  const Eigen::Vector3d& b_;  // bearing
  const Eigen::Vector3d& p_w_;
  const Sophus::SE3d&    T_b_c_;
};

class BearingTangentManifold : public ceres::Manifold {
public:
  // f ∈ R^3
  int AmbientSize() const override { return 3; }

  // δ ∈ R^2
  int TangentSize() const override { return 2; }

  // ---------------------------------------------
  // Plus: f ⊞ δ = Exp(B(f)δ) f
  // ---------------------------------------------
  bool Plus(const double* x, const double* delta, double* x_plus_delta) const override {
    Eigen::Vector3d f = Eigen::Map<const Eigen::Vector3d>(x).normalized();

    // tangent basis at f
    Eigen::Matrix<double, 3, 2> B;
    TangentBasis(f, B);

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
  // PlusJacobian: ∂(f ⊞ δ)/∂δ at δ=0
  //
  // At δ=0:
  // f ⊞ δ ≈ f + B δ
  //
  // so Jacobian = B(f)
  // ---------------------------------------------
  bool PlusJacobian(const double* x, double* jacobian) const override {
    Eigen::Vector3d f = Eigen::Map<const Eigen::Vector3d>(x).normalized();

    Eigen::Matrix<double, 3, 2> B;
    TangentBasis(f, B);

    Eigen::Map<Eigen::Matrix<double, 3, 2, Eigen::RowMajor>> J(jacobian);
    J = B;
    return true;
  }

  // Minus not required unless you implement marginalization manually
  bool Minus(const double*, const double*, double*) const override { return false; }

  bool MinusJacobian(const double*, double*) const override { return false; }
};

}  // namespace omni_slam