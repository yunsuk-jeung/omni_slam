#pragma once

#include <cstdint>

#include <Eigen/Dense>
#include <sophus/se3.hpp>

namespace omni_slam {

// Non-linear factors recovered from a marginalization prior (see
// docs/nfr_mapper.md §3-B). A pose-only prior (H, b) is inverted to a
// covariance Σ = H^-1 and re-expressed as a small set of re-linearizable
// factors: one unary roll-pitch factor on the anchor keyframe, plus relative
// pose factors between the anchor and every other keyframe.
//
// Conventions match optimizer/relative_pose.hpp: body poses T_w_b, tangent
// increments ordered [dt; dtheta] (t += dt in world, R = R * Exp(dtheta)).

// Unary factor constraining the world-gravity tilt (roll, pitch) of a single
// keyframe. Only 2-DoF — position and yaw are gauge and left to RelPoseFactor.
struct RollPitchFactor {
  uint64_t        keyframe_id = 0;
  Sophus::SO3d    R_w_b_meas;                                  // measurement z
  Eigen::Matrix2d information = Eigen::Matrix2d::Identity();   // Ω = Σ_rp^-1

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Binary factor constraining the relative pose T_i_j between two keyframes.
// Gauge-invariant, so it carries the position/yaw information the roll-pitch
// factor drops.
struct RelPoseFactor {
  uint64_t                    keyframe_id_i = 0;
  uint64_t                    keyframe_id_j = 0;
  Sophus::SE3d                T_i_j_meas;                       // measurement z
  Eigen::Matrix<double, 6, 6> information =                     // Ω = Σ_ij^-1
    Eigen::Matrix<double, 6, 6>::Identity();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// --- Residual functions (docs/nfr_mapper.md §3-B-Step2) --------------------
//
// These define the factor coordinate systems used both when *extracting* the
// factor covariance (covariance propagation Σ_factor = J Σ J^T) and when
// *evaluating* the factor during optimization. The jacobians must match at
// both sites, so extraction and optimization share these same functions.
//
// TODO(nfr): all four are stubs returning zero residual / zero jacobian.
// Implement per docs/nfr_mapper.md §3-B (Step 3 for the first three, Step 4
// for rel_pose_error).

// Absolute position residual r = t_w_b - pos_meas. Jacobian is [I_3, 0] at the
// linearization point (rows = world position, cols = [dt; dtheta]).
inline Eigen::Vector3d abs_position_error(
  const Sophus::SE3d&          T_w_b,
  const Eigen::Vector3d&       pos_meas,
  Eigen::Matrix<double, 3, 6>* d_pos_d_T_w_b = nullptr) {
  // TODO(nfr): r = T_w_b.translation() - pos_meas; jac = [I_3 | 0].
  (void)T_w_b;
  (void)pos_meas;
  if (d_pos_d_T_w_b) {
    d_pos_d_T_w_b->setZero();
  }
  return Eigen::Vector3d::Zero();
}

// Yaw residual about the world gravity axis. yaw_dir_body pins the reference
// heading; residual isolates rotation about world Z.
inline double yaw_error(const Sophus::SE3d&          T_w_b,
                        const Eigen::Vector3d&       yaw_dir_body,
                        Eigen::Matrix<double, 1, 6>* d_yaw_d_T_w_b = nullptr) {
  // TODO(nfr): see docs/nfr_mapper.md §3-B-Step3-2 (yawError).
  (void)T_w_b;
  (void)yaw_dir_body;
  if (d_yaw_d_T_w_b) {
    d_yaw_d_T_w_b->setZero();
  }
  return 0.0;
}

// Roll-pitch residual: tilt of T_w_b relative to the measured orientation,
// expressed in the world gravity frame. This is the factor the map actually
// keeps (2-DoF, observable via IMU gravity).
inline Eigen::Vector2d roll_pitch_error(
  const Sophus::SE3d&          T_w_b,
  const Sophus::SO3d&          R_w_b_meas,
  Eigen::Matrix<double, 2, 6>* d_rp_d_T_w_b = nullptr) {
  // TODO(nfr): see docs/nfr_mapper.md §3-B-Step3-2 (rollPitchError).
  (void)T_w_b;
  (void)R_w_b_meas;
  if (d_rp_d_T_w_b) {
    d_rp_d_T_w_b->setZero();
  }
  return Eigen::Vector2d::Zero();
}

// Relative pose residual r = Log(T_i_j_meas^-1 * (T_w_b_i^-1 * T_w_b_j)) with
// jacobians w.r.t. both body poses (tangent [dt; dtheta]).
inline Eigen::Matrix<double, 6, 1> rel_pose_error(
  const Sophus::SE3d&          T_i_j_meas,
  const Sophus::SE3d&          T_w_b_i,
  const Sophus::SE3d&          T_w_b_j,
  Eigen::Matrix<double, 6, 6>* d_res_d_i = nullptr,
  Eigen::Matrix<double, 6, 6>* d_res_d_j = nullptr) {
  // TODO(nfr): see docs/nfr_mapper.md §3-B-Step4 (relPoseError). The forward
  // measurement mirrors optimizer/relative_pose.hpp::compute_rel_pose.
  (void)T_i_j_meas;
  (void)T_w_b_i;
  (void)T_w_b_j;
  if (d_res_d_i) {
    d_res_d_i->setZero();
  }
  if (d_res_d_j) {
    d_res_d_j->setZero();
  }
  return Eigen::Matrix<double, 6, 1>::Zero();
}

}  // namespace omni_slam
