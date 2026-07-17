#pragma once

#include <Eigen/Dense>
#include <sophus/se3.hpp>

namespace omni_slam {

// Relative camera pose T_t_h = (T_w_b_t * T_b_c_t)^-1 * (T_w_b_h * T_b_c_h)
// and its jacobians w.r.t. the SE3BoxplusManifold increments of the two body
// poses (t += dt in world, R = R * Exp(dtheta) in body). Jacobian rows/cols
// are ordered [dt; dtheta] to match the manifold tangent.
//
// Pass twb_lin() for FEJ evaluation, twb() (or the current ceres params)
// otherwise — the formulas are the same, only the evaluation point differs.
inline Sophus::SE3d compute_rel_pose(
  const Sophus::SE3d&          T_w_b_h,
  const Sophus::SE3d&          T_b_c_h,
  const Sophus::SE3d&          T_w_b_t,
  const Sophus::SE3d&          T_b_c_t,
  Eigen::Matrix<double, 6, 6>* d_rel_d_h = nullptr,
  Eigen::Matrix<double, 6, 6>* d_rel_d_t = nullptr) {
  const Sophus::SE3d T_w_c_h = T_w_b_h * T_b_c_h;
  const Sophus::SE3d T_w_c_t = T_w_b_t * T_b_c_t;
  const Sophus::SE3d T_t_h   = T_w_c_t.inverse() * T_w_c_h;

  if (d_rel_d_h) {
    const Eigen::Matrix3d R_ct_w = T_w_c_t.so3().inverse().matrix();
    const Eigen::Matrix3d R_wb_h = T_w_b_h.so3().matrix();
    d_rel_d_h->setZero();
    d_rel_d_h->topLeftCorner<3, 3>() = R_ct_w;
    d_rel_d_h->topRightCorner<3, 3>() =
      -R_ct_w * R_wb_h * Sophus::SO3d::hat(T_b_c_h.translation());
    d_rel_d_h->bottomRightCorner<3, 3>() = T_b_c_h.so3().inverse().matrix();
  }

  if (d_rel_d_t) {
    const Eigen::Matrix3d R_ct_w = T_w_c_t.so3().inverse().matrix();
    const Eigen::Matrix3d R_cb_t = T_b_c_t.so3().inverse().matrix();
    const Eigen::Vector3d p_h_in_tb = T_w_b_t.so3().inverse()
                              * (T_w_c_h.translation() - T_w_b_t.translation());
    d_rel_d_t->setZero();
    d_rel_d_t->topLeftCorner<3, 3>()     = -R_ct_w;
    d_rel_d_t->topRightCorner<3, 3>()    = R_cb_t * Sophus::SO3d::hat(p_h_in_tb);
    d_rel_d_t->bottomRightCorner<3, 3>() = -(T_t_h.so3().inverse().matrix())
                                           * R_cb_t;
  }

  return T_t_h;
}

}  // namespace omni_slam
