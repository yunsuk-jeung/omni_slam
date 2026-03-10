#pragma once

#include <cmath>
#include <sophus/se3.hpp>
#include <ceres/ceres.h>

#include "utils/eigen_utils.hpp"
#include "utils/sophus_utils.hpp"
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
    EigenUtil::TangentBasis(b_obs_, B);

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
      // Right perturbation on body pose:
      //   R_w_b <- R_w_b * Exp(dθ)
      //
      // p_c = T_b_c^{-1} * (T_w_b^{-1} * p_w)
      // => dp_c/dθ = R_c_b * hat(p_b),  p_b = T_w_b^{-1} * p_w
      //
      const Eigen::Vector3d p_b   = T_w_b.inverse() * p_w_;
      const Eigen::Matrix3d R_c_b = T_b_c_.inverse().so3().matrix();
      const Eigen::Map<const Eigen::Vector3d> so3(params[0] + 3);
      const Eigen::Matrix3d J_r = SophusUtils::SO3RightJacobian(so3);
      const Eigen::Matrix3d dp_dtheta_local = R_c_b * Sophus::SO3d::hat(p_b);
      Eigen::Matrix3d       dp_dtheta       = dp_dtheta_local * J_r;

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

struct PoseOnlyBearingCostAuto {
  PoseOnlyBearingCostAuto(const Eigen::Vector3d& p_w,
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
    EigenUtil::TangentBasis(b_obs_, B_d);

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

class BearingPriorCost final : public ceres::SizedCostFunction<2, 3> {
public:
  explicit BearingPriorCost(const Eigen::Vector3d& b_prior)
    : b_prior_(b_prior.normalized()) {
    EigenUtil::TangentBasis(b_prior_, B_);
  }

  bool Evaluate(double const* const* params,
                double*              residuals,
                double**             jacobians) const override {
    Eigen::Vector3d b(params[0][0], params[0][1], params[0][2]);
    const double    b_norm = b.norm();
    if (b_norm <= 0.0) {
      residuals[0] = 0.0;
      residuals[1] = 0.0;
      if (jacobians && jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> J(jacobians[0]);
        J.setZero();
      }
      return true;
    }

    b /= b_norm;
    const Eigen::Vector2d r = B_.transpose() * (b - b_prior_);
    residuals[0]            = r[0];
    residuals[1]            = r[1];

    if (jacobians && jacobians[0]) {
      Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> J(jacobians[0]);
      const Eigen::Matrix3d J_bearing = (Eigen::Matrix3d::Identity() - b * b.transpose())
                                        / b_norm;
      J = B_.transpose() * J_bearing;
    }
    return true;
  }

private:
  Eigen::Vector3d             b_prior_;
  Eigen::Matrix<double, 3, 2> B_;
};

class BearingCost final : public ceres::SizedCostFunction<2, 6, 6, 3, 1> {
public:
  BearingCost(const Eigen::Vector3d& b_obs,
              const Sophus::SE3d&    T_b_c_obs,
              const Sophus::SE3d&    T_b_c_host)
    : b_obs_(b_obs.normalized())
    , T_b_c_obs_(T_b_c_obs)
    , T_b_c_host_(T_b_c_host) {
    EigenUtil::TangentBasis(b_obs_, B_);
  }

  bool Evaluate(double const* const* params,
                double*              residuals,
                double**             jacobians) const override {
    const double* pose_obs       = params[0];
    const double* pose_host      = params[1];
    const double* bearing_param  = params[2];
    const double* inv_dist_param = params[3];

    const Sophus::SE3d T_w_b_obs  = SE3BoxplusManifold::FromParams(pose_obs);
    const Sophus::SE3d T_w_b_host = SE3BoxplusManifold::FromParams(pose_host);

    const Sophus::SE3d T_w_c_obs  = T_w_b_obs * T_b_c_obs_;
    const Sophus::SE3d T_w_c_host = T_w_b_host * T_b_c_host_;

    Eigen::Vector3d b_h(bearing_param[0], bearing_param[1], bearing_param[2]);
    const double    b_norm = b_h.norm();
    if (b_norm <= 0.0) {
      residuals[0] = 0.0;
      residuals[1] = 0.0;
      zeroJacobians(jacobians);
      return true;
    }
    b_h /= b_norm;

    const double inv_dist = inv_dist_param[0];
    if (!std::isfinite(inv_dist) || inv_dist <= 0.0) {
      residuals[0] = 0.0;
      residuals[1] = 0.0;
      zeroJacobians(jacobians);
      return true;
    }

    const Eigen::Vector3d p_c_host = b_h / inv_dist;
    const Eigen::Vector3d p_w      = T_w_c_host * p_c_host;
    const Eigen::Vector3d p_c_obs  = T_w_c_obs.inverse() * p_w;

    if (p_c_obs.z() <= 1e-6) {
      residuals[0] = 0.0;
      residuals[1] = 0.0;
      zeroJacobians(jacobians);
      return true;
    }

    const Eigen::Vector3d b_pred = p_c_obs.normalized();
    const Eigen::Vector2d r      = B_.transpose() * (b_pred - b_obs_);

    residuals[0] = r[0];
    residuals[1] = r[1];

    if (jacobians) {
      const double p_norm = p_c_obs.norm();
      if (p_norm <= 0.0) {
        zeroJacobians(jacobians);
        return true;
      }

      const Eigen::Matrix3d J_norm = (Eigen::Matrix3d::Identity()
                                      - b_pred * b_pred.transpose())
                                     / p_norm;
      const Eigen::Matrix<double, 2, 3> J_r_pc = B_.transpose() * J_norm;

      const Eigen::Matrix3d R_w_c_obs = T_w_c_obs.so3().matrix();
      const Eigen::Matrix3d R_c_obs_w = R_w_c_obs.transpose();
      const Eigen::Map<const Eigen::Vector3d> so3_obs(pose_obs + 3);
      const Eigen::Map<const Eigen::Vector3d> so3_host(pose_host + 3);
      const Eigen::Matrix3d J_r_obs  = SophusUtils::SO3RightJacobian(so3_obs);
      const Eigen::Matrix3d J_r_host = SophusUtils::SO3RightJacobian(so3_host);

      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>> J(jacobians[0]);
        const Eigen::Matrix3d dp_dt = -R_c_obs_w;
        const Eigen::Vector3d p_b_obs = T_w_b_obs.inverse() * p_w;
        const Eigen::Matrix3d R_c_obs_b_obs = T_b_c_obs_.inverse().so3().matrix();
        const Eigen::Matrix3d dp_dtheta_local = R_c_obs_b_obs
                                                * Sophus::SO3d::hat(p_b_obs);
        const Eigen::Matrix3d dp_dtheta = dp_dtheta_local * J_r_obs;

        J.leftCols<3>()  = J_r_pc * dp_dt;
        J.rightCols<3>() = J_r_pc * dp_dtheta;
      }

      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>> J(jacobians[1]);
        const Eigen::Matrix3d dp_dt = R_c_obs_w;
        const Eigen::Matrix3d R_w_b_host = T_w_b_host.so3().matrix();
        const Eigen::Vector3d p_b_host = T_b_c_host_ * p_c_host;
        const Eigen::Matrix3d dp_dtheta_local = -R_c_obs_w * R_w_b_host
                                                * Sophus::SO3d::hat(p_b_host);
        const Eigen::Matrix3d dp_dtheta = dp_dtheta_local * J_r_host;

        J.leftCols<3>()  = J_r_pc * dp_dt;
        J.rightCols<3>() = J_r_pc * dp_dtheta;
      }

      if (jacobians[2]) {
        Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> J(jacobians[2]);
        const Eigen::Matrix3d R_w_c_host = T_w_c_host.so3().matrix();
        const Eigen::Matrix3d dp_dbh     = (1.0 / inv_dist) * (R_c_obs_w * R_w_c_host);
        const Eigen::Matrix3d J_bearing  = (Eigen::Matrix3d::Identity()
                                           - b_h * b_h.transpose())
                                          / b_norm;
        J = J_r_pc * dp_dbh * J_bearing;
      }

      if (jacobians[3]) {
        Eigen::Map<Eigen::Matrix<double, 2, 1>> J(jacobians[3]);
        const Eigen::Matrix3d                   R_w_c_host = T_w_c_host.so3().matrix();
        const Eigen::Vector3d dp_dinv_dist = -(R_c_obs_w * R_w_c_host * b_h)
                                             / (inv_dist * inv_dist);
        J = J_r_pc * dp_dinv_dist;
      }
    }

    return true;
  }

private:
  static void zeroJacobians(double** jacobians) {
    if (!jacobians) {
      return;
    }
    if (jacobians[0]) {
      Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>> J(jacobians[0]);
      J.setZero();
    }
    if (jacobians[1]) {
      Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>> J(jacobians[1]);
      J.setZero();
    }
    if (jacobians[2]) {
      Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> J(jacobians[2]);
      J.setZero();
    }
    if (jacobians[3]) {
      Eigen::Map<Eigen::Matrix<double, 2, 1>> J(jacobians[3]);
      J.setZero();
    }
  }

  Eigen::Vector3d             b_obs_;
  Sophus::SE3d                T_b_c_obs_;
  Sophus::SE3d                T_b_c_host_;
  Eigen::Matrix<double, 3, 2> B_;
};

class BearingStereoCost final : public ceres::SizedCostFunction<2, 3, 1> {
public:
  BearingStereoCost(const Eigen::Vector3d& b_obs,
                    const Sophus::SE3d&    T_b_c_obs,
                    const Sophus::SE3d&    T_b_c_host)
    : b_obs_(b_obs.normalized())
    , T_b_c_obs_(T_b_c_obs)
    , T_b_c_host_(T_b_c_host) {
    EigenUtil::TangentBasis(b_obs_, B_);
  }

  bool Evaluate(double const* const* params,
                double*              residuals,
                double**             jacobians) const override {
    const double* bearing_param  = params[0];
    const double* inv_dist_param = params[1];

    Eigen::Vector3d b_h(bearing_param[0], bearing_param[1], bearing_param[2]);
    const double    b_norm = b_h.norm();
    if (b_norm <= 0.0) {
      residuals[0] = 0.0;
      residuals[1] = 0.0;
      zeroJacobians(jacobians);
      return true;
    }
    b_h /= b_norm;

    const double inv_dist = inv_dist_param[0];
    if (!std::isfinite(inv_dist) || inv_dist <= 0.0) {
      residuals[0] = 0.0;
      residuals[1] = 0.0;
      zeroJacobians(jacobians);
      return true;
    }

    const Eigen::Vector3d p_c_host       = b_h / inv_dist;
    const Sophus::SE3d    T_c_obs_b      = T_b_c_obs_.inverse();
    const Sophus::SE3d    T_c_obs_c_host = T_c_obs_b * T_b_c_host_;
    const Eigen::Vector3d p_c_obs        = T_c_obs_c_host * p_c_host;

    if (p_c_obs.z() <= 1e-6) {
      residuals[0] = 0.0;
      residuals[1] = 0.0;
      zeroJacobians(jacobians);
      return true;
    }

    const Eigen::Vector3d b_pred = p_c_obs.normalized();
    const Eigen::Vector2d r      = B_.transpose() * (b_pred - b_obs_);

    residuals[0] = r[0];
    residuals[1] = r[1];

    if (jacobians) {
      const double p_norm = p_c_obs.norm();
      if (p_norm <= 0.0) {
        zeroJacobians(jacobians);
        return true;
      }

      const Eigen::Matrix3d J_norm = (Eigen::Matrix3d::Identity()
                                      - b_pred * b_pred.transpose())
                                     / p_norm;
      const Eigen::Matrix<double, 2, 3> J_r_pc = B_.transpose() * J_norm;

      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> J(jacobians[0]);
        const Eigen::Matrix3d R_c_obs_c_host = T_c_obs_c_host.so3().matrix();
        const Eigen::Matrix3d dp_dbh         = (1.0 / inv_dist) * R_c_obs_c_host;
        const Eigen::Matrix3d J_bearing      = (Eigen::Matrix3d::Identity()
                                           - b_h * b_h.transpose())
                                          / b_norm;
        J = J_r_pc * dp_dbh * J_bearing;
      }

      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, 2, 1>> J(jacobians[1]);
        const Eigen::Matrix3d R_c_obs_c_host = T_c_obs_c_host.so3().matrix();
        const Eigen::Vector3d dp_dinv_dist   = -(R_c_obs_c_host * b_h)
                                             / (inv_dist * inv_dist);
        J = J_r_pc * dp_dinv_dist;
      }
    }

    return true;
  }

private:
  static void zeroJacobians(double** jacobians) {
    if (!jacobians) {
      return;
    }
    if (jacobians[0]) {
      Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> J(jacobians[0]);
      J.setZero();
    }
    if (jacobians[1]) {
      Eigen::Map<Eigen::Matrix<double, 2, 1>> J(jacobians[1]);
      J.setZero();
    }
  }

  Eigen::Vector3d             b_obs_;
  Sophus::SE3d                T_b_c_obs_;
  Sophus::SE3d                T_b_c_host_;
  Eigen::Matrix<double, 3, 2> B_;
};

struct BearingStereoCostAuto {
  BearingStereoCostAuto(const Eigen::Vector3d& b_obs,
                        const Sophus::SE3d&    T_b_c_obs,
                        const Sophus::SE3d&    T_b_c_host)
    : b_obs_(b_obs.normalized())
    , T_b_c_obs_(T_b_c_obs)
    , T_b_c_host_(T_b_c_host) {
    EigenUtil::TangentBasis(b_obs_, B_);
  }

  template <typename T>
  bool operator()(const T* const bearing_param,
                  const T* const inv_dist_param,
                  T*             residuals) const {
    const Sophus::SE3<T> T_b_c_obs      = T_b_c_obs_.template cast<T>();
    const Sophus::SE3<T> T_b_c_host     = T_b_c_host_.template cast<T>();
    const Sophus::SE3<T> T_c_obs_b      = T_b_c_obs.inverse();
    const Sophus::SE3<T> T_c_obs_c_host = T_c_obs_b * T_b_c_host;

    Eigen::Matrix<T, 3, 1> b_h(bearing_param[0], bearing_param[1], bearing_param[2]);
    const T                b_norm = b_h.norm();
    if (b_norm > T(0)) {
      b_h /= b_norm;
    }

    const T inv_d = inv_dist_param[0];
    if (inv_d <= T(0)) {
      residuals[0] = T(0);
      residuals[1] = T(0);
      return true;
    }

    const Eigen::Matrix<T, 3, 1> p_c_host = b_h / inv_d;
    const Eigen::Matrix<T, 3, 1> p_c_obs  = T_c_obs_c_host * p_c_host;

    if (p_c_obs.z() <= T(1e-6)) {
      residuals[0] = T(0);
      residuals[1] = T(0);
      return true;
    }

    const Eigen::Matrix<T, 3, 1> b_pred = p_c_obs.normalized();
    const Eigen::Matrix<T, 3, 2> B      = B_.template cast<T>();
    const Eigen::Matrix<T, 3, 1> b_obs  = b_obs_.template cast<T>();
    const Eigen::Matrix<T, 2, 1> r      = B.transpose() * (b_pred - b_obs);

    residuals[0] = r[0];
    residuals[1] = r[1];
    return true;
  }

  Eigen::Vector3d             b_obs_;
  Sophus::SE3d                T_b_c_obs_;
  Sophus::SE3d                T_b_c_host_;
  Eigen::Matrix<double, 3, 2> B_;
};

struct BearingCostAuto {
  BearingCostAuto(const Eigen::Vector3d& b_obs,
                  const Sophus::SE3d&    T_b_c_obs,
                  const Sophus::SE3d&    T_b_c_host)
    : b_obs_(b_obs.normalized())
    , T_b_c_obs_(T_b_c_obs)
    , T_b_c_host_(T_b_c_host) {
    EigenUtil::TangentBasis(b_obs_, B_);
  }

  template <typename T>
  bool operator()(const T* const pose,
                  const T* const bearing_param,
                  const T* const inv_dist_param,
                  T*             residuals) const {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t(pose);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> so3(pose + 3);
    Sophus::SO3<T>                           R_w_b = Sophus::SO3<T>::exp(so3);
    Sophus::SE3<T>                           T_w_b(R_w_b, t);

    const Sophus::SE3<T> T_b_c_obs  = T_b_c_obs_.template cast<T>();
    const Sophus::SE3<T> T_b_c_host = T_b_c_host_.template cast<T>();

    const Sophus::SE3<T> T_w_c_obs  = T_w_b * T_b_c_obs;
    const Sophus::SE3<T> T_w_c_host = T_w_b * T_b_c_host;

    Eigen::Matrix<T, 3, 1> b_h(bearing_param[0], bearing_param[1], bearing_param[2]);
    const T                b_norm = b_h.norm();
    if (b_norm > T(0)) {
      b_h /= b_norm;
    }

    const T inv_d = inv_dist_param[0];
    if (inv_d <= T(0)) {
      residuals[0] = T(0);
      residuals[1] = T(0);
      return true;
    }

    const Eigen::Matrix<T, 3, 1> p_c_host = b_h / inv_d;
    const Eigen::Matrix<T, 3, 1> p_w      = T_w_c_host * p_c_host;
    const Eigen::Matrix<T, 3, 1> p_c_obs  = T_w_c_obs.inverse() * p_w;

    if (p_c_obs.z() <= T(1e-6)) {
      residuals[0] = T(0);
      residuals[1] = T(0);
      return true;
    }

    const Eigen::Matrix<T, 3, 1> b_pred = p_c_obs.normalized();
    const Eigen::Matrix<T, 3, 2> B      = B_.template cast<T>();
    const Eigen::Matrix<T, 3, 1> b_obs  = b_obs_.template cast<T>();
    const Eigen::Matrix<T, 2, 1> r      = B.transpose() * (b_pred - b_obs);

    residuals[0] = r[0];
    residuals[1] = r[1];
    return true;
  }

  Eigen::Vector3d             b_obs_;
  Sophus::SE3d                T_b_c_obs_;
  Sophus::SE3d                T_b_c_host_;
  Eigen::Matrix<double, 3, 2> B_;
};
}  // namespace omni_slam
