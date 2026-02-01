#pragma once

#include <Sophus/se3.hpp>
#include <ceres/ceres.h>

#include "utils/eigen_utils.hpp"
#include "optimizer/parameterization.hpp"

namespace omni_slam {
class PoseOnlyBearingCost final : public ceres::SizedCostFunction<2, 6> {
public:
  PoseOnlyBearingCost(const Eigen::Vector3d& p_w,
                      const Eigen::Vector3d& b_obs,
                      const Sophus::SE3d&    T_b_c)
    : p_w_(p_w)
    , b_obs_(b_obs.normalized())
    , T_b_c_(T_b_c) {}

  bool Evaluate(double const* const* params,
                double*              residuals,
                double**             jacobians) const override {
    // -----------------------------
    // Pose: world -> body
    // -----------------------------
    Sophus::SE3d T_w_b = SE3BoxplusManifold::FromParams(params[0]);

    // world -> camera
    Sophus::SE3d T_w_c = T_w_b * T_b_c_;

    // camera -> world rotation
    Eigen::Matrix3d R_c_w = T_w_c.so3().matrix().transpose();
    Eigen::Vector3d t_w_c = T_w_c.translation();

    // point in camera frame
    Eigen::Vector3d p_c = R_c_w * (p_w_ - t_w_c);

    // reject invalid depth
    if (p_c.z() <= 1e-6) {
      residuals[0] = residuals[1] = 0.0;
      if (jacobians && jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>> J(jacobians[0]);
        J.setZero();
      }
      return true;
    }

    // -----------------------------
    // predicted bearing
    // -----------------------------
    Eigen::Vector3d b = p_c.normalized();

    // tangent basis at observed bearing
    Eigen::Matrix<double, 3, 2> B;
    TangentBasis(b_obs_, B);

    // residual: 2D tangent error
    Eigen::Map<Eigen::Vector2d> r(residuals);
    r = B.transpose() * (b - b_obs_);

    // -----------------------------
    // Jacobian
    // -----------------------------
    if (jacobians && jacobians[0]) {
      Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>> J(jacobians[0]);
      J.setZero();

      // ---- (1) db/dp_c ----
      double inv_norm = 1.0 / p_c.norm();

      Eigen::Matrix3d J_norm = (Eigen::Matrix3d::Identity() - b * b.transpose())
                               * inv_norm;

      // ---- (2) dp_c/dt ----
      // p_c = R_c_w (p_w - t)
      Eigen::Matrix3d dp_dt = -R_c_w;

      // ---- (3) dp_c/dtheta ----
      //
      // Under your manifold:
      //   t += dt
      //   R <- Exp(dθ) R
      //
      // Perturbation is applied on body rotation (left-mult),
      // so camera point variation:
      //
      //   dp_c ≈ R_c_w * hat(p_w - t) * dθ
      //
      Eigen::Matrix3d dp_dtheta = R_c_w * Sophus::SO3d::hat(p_w_ - t_w_c);

      // assemble dp_c/dpose
      Eigen::Matrix<double, 3, 6> J_p;
      J_p.leftCols<3>()  = dp_dt;
      J_p.rightCols<3>() = dp_dtheta;

      // chain rule: r = Bᵀ * b(p_c)
      J = B.transpose() * J_norm * J_p;
    }

    return true;
  }

private:
  Eigen::Vector3d p_w_;
  Eigen::Vector3d b_obs_;
  Sophus::SE3d    T_b_c_;
};

struct PoseOnlyBearingCostFunctor {
  PoseOnlyBearingCostFunctor(const Eigen::Vector3d& p_w,
                             const Eigen::Vector3d& b_obs,
                             const Sophus::SE3d&    T_b_c)
    : p_w_(p_w)
    , b_obs_(b_obs.normalized())
    , T_b_c_(T_b_c) {}

  template <typename T>
  bool operator()(const T* const pose, T* residuals) const {
    // pose = [t(3), so3(3)]
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t(pose);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> so3(pose + 3);

    // world -> body
    Sophus::SO3<T> R_w_b = Sophus::SO3<T>::exp(so3);
    Sophus::SE3<T> T_w_b(R_w_b, t);

    // body -> camera extrinsic (constant)
    Sophus::SE3<T> T_b_c = T_b_c_.cast<T>();

    // world -> camera
    Sophus::SE3<T> T_w_c = T_w_b * T_b_c;

    // camera frame point: p_c = R_c_w (p_w - t)
    Eigen::Matrix<T, 3, 1> p_w = p_w_.cast<T>();
    Eigen::Matrix<T, 3, 1> p_c = T_w_c.inverse() * p_w;

    // depth check
    if (p_c.z() <= T(1e-6)) {
      residuals[0] = T(0);
      residuals[1] = T(0);
      return true;
    }

    // predicted bearing
    Eigen::Matrix<T, 3, 1> b = p_c.normalized();

    // tangent basis at observed bearing (constant)
    Eigen::Matrix<double, 3, 2> B_d;
    TangentBasis(b_obs_, B_d);

    Eigen::Matrix<T, 3, 2> B = B_d.cast<T>();

    // residual = Bᵀ (b - b_obs)
    Eigen::Matrix<T, 3, 1> b_obs = b_obs_.cast<T>();

    Eigen::Matrix<T, 2, 1> r = B.transpose() * (b - b_obs);

    residuals[0] = r[0];
    residuals[1] = r[1];

    return true;
  }

  Eigen::Vector3d p_w_;
  Eigen::Vector3d b_obs_;
  Sophus::SE3d    T_b_c_;
};
}  // namespace omni_slam