#include <algorithm>
#include <cmath>

#include "odometry/imu_preintegration.hpp"
#include "utils/logger.hpp"

namespace omni_slam {

namespace {
constexpr double kNsToSec = 1e-9;
}

ImuPreintegration::ImuPreintegration(uint64_t               from_frame_id,
                                     uint64_t               to_frame_id,
                                     const Eigen::Vector3d& bias_acc,
                                     const Eigen::Vector3d& bias_gyr)
  : ImuPreintegration(from_frame_id, to_frame_id, bias_acc, bias_gyr, Parameters{}) {}

ImuPreintegration::ImuPreintegration(uint64_t               from_frame_id,
                                     uint64_t               to_frame_id,
                                     const Eigen::Vector3d& bias_acc,
                                     const Eigen::Vector3d& bias_gyr,
                                     const Parameters&      parameters)
  : parameters_(parameters)
  , delta_r_()
  , delta_v_(Eigen::Vector3d::Zero())
  , delta_p_(Eigen::Vector3d::Zero())
  , delta_t_sec_(0.0)
  , bias_acc_(bias_acc)
  , bias_gyr_(bias_gyr)
  , from_frame_id_(from_frame_id)
  , to_frame_id_(to_frame_id)
  , jacobian_(Eigen::Matrix15d::Identity())
  , covariance_(Eigen::Matrix15d::Zero())
  , integration_steps_(0) {}

void ImuPreintegration::Reset(const Eigen::Vector3d& bias_acc,
                              const Eigen::Vector3d& bias_gyr) {
  delta_r_ = Sophus::SO3d();
  delta_v_.setZero();
  delta_p_.setZero();
  delta_t_sec_ = 0.0;

  bias_acc_ = bias_acc;
  bias_gyr_ = bias_gyr;

  jacobian_.setIdentity();
  covariance_.setZero();
  integration_steps_ = 0;
}

void ImuPreintegration::SetBias(const Eigen::Vector3d& bias_acc,
                                const Eigen::Vector3d& bias_gyr) {
  bias_acc_ = bias_acc;
  bias_gyr_ = bias_gyr;
}

void ImuPreintegration::SetParameters(const Parameters& parameters) {
  parameters_ = parameters;
}

bool ImuPreintegration::IntegrateMeasurement(const ImuData& imu0, const ImuData& imu1) {
  const int64_t dt_ns = imu1.t_ns - imu0.t_ns;
  if (dt_ns <= 0) {
    return false;
  }

  const double dt_sec = static_cast<double>(dt_ns) * kNsToSec;
  if (dt_sec < parameters_.min_integration_dt_s) {
    return false;
  }

  // Midpoint state propagation with start/end samples.
  const Eigen::Vector3d acc0 = imu0.acc - bias_acc_;
  const Eigen::Vector3d acc1 = imu1.acc - bias_acc_;
  const Eigen::Vector3d gyr0 = imu0.gyr - bias_gyr_;
  const Eigen::Vector3d gyr1 = imu1.gyr - bias_gyr_;

  const Eigen::Vector3d gyr_mid       = 0.5 * (gyr0 + gyr1);
  const Sophus::SO3d    delta_r_start = delta_r_;
  const Sophus::SO3d    delta_r_step  = Sophus::SO3d::exp(gyr_mid * dt_sec);
  const Sophus::SO3d    delta_r_next  = delta_r_start * delta_r_step;

  const Eigen::Vector3d acc_world_start = delta_r_start.matrix() * acc0;
  const Eigen::Vector3d acc_world_end   = delta_r_next.matrix() * acc1;
  const Eigen::Vector3d acc_world_mid   = 0.5 * (acc_world_start + acc_world_end);

  PropagateError(delta_r_start.matrix(),
                 delta_r_next.matrix(),
                 acc0,
                 acc1,
                 gyr_mid,
                 dt_sec);
  PropagateState(delta_r_next, acc_world_mid, dt_sec);
  ++integration_steps_;
  return true;
}

bool ImuPreintegration::IntegrateMeasurements(const std::vector<ImuData>& imu_data) {
  if (imu_data.size() < 2) {
    return false;
  }

  bool integrated = false;
  for (size_t i = 0; i + 1 < imu_data.size(); ++i) {
    if (IntegrateMeasurement(imu_data[i], imu_data[i + 1])) {
      integrated = true;
    }
  }
  return integrated;
}

void ImuPreintegration::PropagateState(const Sophus::SO3d&    delta_r_next,
                                       const Eigen::Vector3d& acc_world_mid,
                                       double                 dt_sec) {
  delta_p_ += delta_v_ * dt_sec + 0.5 * acc_world_mid * dt_sec * dt_sec;
  delta_v_ += acc_world_mid * dt_sec;
  delta_r_ = delta_r_next;

  delta_t_sec_ += dt_sec;
}

void ImuPreintegration::PropagateError(const Eigen::Matrix3d& R_start,
                                       const Eigen::Matrix3d& R_next,
                                       const Eigen::Vector3d& acc0_body,
                                       const Eigen::Vector3d& acc1_body,
                                       const Eigen::Vector3d& gyr_mid,
                                       double                 dt_sec) {
  const Eigen::Matrix3d I3     = Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d w_hat  = Sophus::SO3d::hat(gyr_mid);
  const Eigen::Matrix3d a0_hat = Sophus::SO3d::hat(acc0_body);
  const Eigen::Matrix3d a1_hat = Sophus::SO3d::hat(acc1_body);
  const Eigen::Matrix3d tmp    = I3 - w_hat * dt_sec;

  const double dt2 = dt_sec * dt_sec;
  const double dt3 = dt2 * dt_sec;

  // Error-state ordering:
  // [0..2]: dp, [3..5]: dtheta, [6..8]: dv, [9..11]: dba, [12..14]: dbg
  Eigen::Matrix15d F   = Eigen::Matrix15d::Identity();
  F.block<3, 3>(3, 3)  = tmp;
  F.block<3, 3>(3, 12) = -I3 * dt_sec;

  F.block<3, 3>(6, 3) = -0.5 * R_start * a0_hat * dt_sec
                        - 0.5 * R_next * a1_hat * tmp * dt_sec;
  F.block<3, 3>(6, 9)  = -0.5 * (R_start + R_next) * dt_sec;
  F.block<3, 3>(6, 12) = 0.5 * R_next * a1_hat * dt2;

  F.block<3, 3>(0, 3) = -0.25 * R_start * a0_hat * dt2
                        - 0.25 * R_next * a1_hat * tmp * dt2;
  F.block<3, 3>(0, 6)  = I3 * dt_sec;
  F.block<3, 3>(0, 9)  = -0.25 * (R_start + R_next) * dt2;
  F.block<3, 3>(0, 12) = 0.25 * R_next * a1_hat * dt3;

  jacobian_ = F * jacobian_;

  // VINS-style midpoint noise injection (kept local to preserve public API).
  // noise = [na0, ng0, na1, ng1, nba_rw, nbg_rw] (each 3D) => 18D
  Eigen::Matrix<double, 15, 18> V = Eigen::Matrix<double, 15, 18>::Zero();
  V.block<3, 3>(3, 3)             = 0.5 * I3 * dt_sec;
  V.block<3, 3>(3, 9)             = 0.5 * I3 * dt_sec;

  V.block<3, 3>(6, 0) = 0.5 * R_start * dt_sec;
  V.block<3, 3>(6, 3) = -0.25 * R_next * a1_hat * dt2;
  V.block<3, 3>(6, 6) = 0.5 * R_next * dt_sec;
  V.block<3, 3>(6, 9) = -0.25 * R_next * a1_hat * dt2;

  V.block<3, 3>(0, 0) = 0.25 * R_start * dt2;
  V.block<3, 3>(0, 3) = -0.125 * R_next * a1_hat * dt3;
  V.block<3, 3>(0, 6) = 0.25 * R_next * dt2;
  V.block<3, 3>(0, 9) = -0.125 * R_next * a1_hat * dt3;

  V.block<3, 3>(9, 12)  = I3 * dt_sec;
  V.block<3, 3>(12, 15) = I3 * dt_sec;

  Eigen::Matrix<double, 18, 18> Q      = Eigen::Matrix<double, 18, 18>::Zero();
  const double                  inv_dt = 1.0 / dt_sec;
  const double sigma_acc2   = parameters_.acc_noise_sigma * parameters_.acc_noise_sigma;
  const double sigma_gyr2   = parameters_.gyr_noise_sigma * parameters_.gyr_noise_sigma;
  const double sigma_ba_rw2 = parameters_.acc_bias_rw_sigma
                              * parameters_.acc_bias_rw_sigma;
  const double sigma_bg_rw2 = parameters_.gyr_bias_rw_sigma
                              * parameters_.gyr_bias_rw_sigma;
  Q.block<3, 3>(0, 0)   = I3 * sigma_acc2 * inv_dt;
  Q.block<3, 3>(3, 3)   = I3 * sigma_gyr2 * inv_dt;
  Q.block<3, 3>(6, 6)   = I3 * sigma_acc2 * inv_dt;
  Q.block<3, 3>(9, 9)   = I3 * sigma_gyr2 * inv_dt;
  Q.block<3, 3>(12, 12) = I3 * sigma_ba_rw2 * inv_dt;
  Q.block<3, 3>(15, 15) = I3 * sigma_bg_rw2 * inv_dt;

  covariance_ = F * covariance_ * F.transpose() + V * Q * V.transpose();
  covariance_ = 0.5 * (covariance_ + covariance_.transpose());
}

ImuPreintegration::CorrectedDelta ImuPreintegration::GetBiasCorrectedDelta(
  const Eigen::Vector3d& bias_acc,
  const Eigen::Vector3d& bias_gyr) const {
  const Eigen::Vector3d dba = bias_acc - bias_acc_;
  const Eigen::Vector3d dbg = bias_gyr - bias_gyr_;

  CorrectedDelta corrected;
  corrected.delta_r = delta_r_ * Sophus::SO3d::exp(GetJDeltaRDbg() * dbg);
  corrected.delta_v = delta_v_ + GetJDeltaVDBa() * dba + GetJDeltaVDbg() * dbg;
  corrected.delta_p = delta_p_ + GetJDeltaPDBa() * dba + GetJDeltaPDbg() * dbg;
  return corrected;
}

Eigen::Matrix15d ImuPreintegration::GetInformation(double damping) const {
  Eigen::Matrix15d covariance_regularized = covariance_;
  covariance_regularized.diagonal().array() += std::max(damping, 0.0);

  Eigen::LDLT<Eigen::Matrix15d> ldlt(covariance_regularized);
  if (ldlt.info() != Eigen::Success) {
    LogE("eigen ldlt failed");
    return Eigen::Matrix15d::Zero();
  }
  return ldlt.solve(Eigen::Matrix15d::Identity());
}

Eigen::Matrix3d ImuPreintegration::GetJDeltaRDbg() const {
  return jacobian_.block<3, 3>(3, 12);
}

Eigen::Matrix3d ImuPreintegration::GetJDeltaVDBa() const {
  return jacobian_.block<3, 3>(6, 9);
}

Eigen::Matrix3d ImuPreintegration::GetJDeltaVDbg() const {
  return jacobian_.block<3, 3>(6, 12);
}

Eigen::Matrix3d ImuPreintegration::GetJDeltaPDBa() const {
  return jacobian_.block<3, 3>(0, 9);
}

Eigen::Matrix3d ImuPreintegration::GetJDeltaPDbg() const {
  return jacobian_.block<3, 3>(0, 12);
}

}  // namespace omni_slam
