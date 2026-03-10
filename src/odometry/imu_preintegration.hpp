#pragma once

#include <cstddef>
#include <vector>

#include <sophus/so3.hpp>

#include "utils/eigen_utils.hpp"
#include "utils/types.hpp"

namespace omni_slam {

// IMU preintegration with a 15D error-state model:
// [dp, dtheta, dv, dba, dbg].
class ImuPreintegration {
public:
  struct Options {
    // Standard deviations of IMU white noise and bias random walk.
    double acc_noise_sigma      = 0.08;
    double gyr_noise_sigma      = 0.004;
    double acc_bias_rw_sigma    = 0.0002;
    double gyr_bias_rw_sigma    = 0.00002;
    double min_integration_dt_s = 1e-6;
  };

  struct CorrectedDelta {
    Eigen::Vector3d delta_p;
    Sophus::SO3d    delta_r;
    Eigen::Vector3d delta_v;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  explicit ImuPreintegration(const Eigen::Vector3d& bias_acc = Eigen::Vector3d::Zero(),
                             const Eigen::Vector3d& bias_gyr = Eigen::Vector3d::Zero());
  ImuPreintegration(const Eigen::Vector3d& bias_acc,
                    const Eigen::Vector3d& bias_gyr,
                    const Options&         options);

  void Reset(const Eigen::Vector3d& bias_acc, const Eigen::Vector3d& bias_gyr);
  void SetBias(const Eigen::Vector3d& bias_acc, const Eigen::Vector3d& bias_gyr);
  void SetOptions(const Options& options);

  bool IntegrateMeasurement(const ImuData& imu0, const ImuData& imu1);
  bool IntegrateMeasurements(const std::vector<ImuData>& imu_samples);

  CorrectedDelta GetBiasCorrectedDelta(const Eigen::Vector3d& bias_acc,
                                       const Eigen::Vector3d& bias_gyr) const;

  Eigen::Matrix15d GetInformation(double damping = 1e-12) const;

  const Sophus::SO3d&   GetDeltaR() const { return delta_r_; }
  const Eigen::Vector3d GetDeltaV() const { return delta_v_; }
  const Eigen::Vector3d GetDeltaP() const { return delta_p_; }
  double                GetDeltaTimeSec() const { return delta_t_sec_; }

  const Eigen::Vector3d& GetBiasAcc() const { return bias_acc_; }
  const Eigen::Vector3d& GetBiasGyr() const { return bias_gyr_; }

  const Eigen::Matrix15d& GetJacobian() const { return jacobian_; }
  const Eigen::Matrix15d& GetCovariance() const { return covariance_; }

  Eigen::Matrix3d GetJDeltaRDbg() const;
  Eigen::Matrix3d GetJDeltaVDBa() const;
  Eigen::Matrix3d GetJDeltaVDbg() const;
  Eigen::Matrix3d GetJDeltaPDBa() const;
  Eigen::Matrix3d GetJDeltaPDbg() const;

  size_t GetIntegrationStepCount() const { return integration_steps_; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  void PropagateState(const Sophus::SO3d&    delta_r_next,
                      const Eigen::Vector3d& acc_world_mid,
                      double                 dt_sec);
  void PropagateError(const Eigen::Matrix3d& R_start,
                      const Eigen::Matrix3d& R_next,
                      const Eigen::Vector3d& acc0_body,
                      const Eigen::Vector3d& acc1_body,
                      const Eigen::Vector3d& gyr_mid,
                      double                 dt_sec);

private:
  Options options_;

  Eigen::Vector3d delta_p_;
  Sophus::SO3d    delta_r_;
  Eigen::Vector3d delta_v_;
  double          delta_t_sec_;

  Eigen::Vector3d bias_acc_;
  Eigen::Vector3d bias_gyr_;

  Eigen::Matrix15d jacobian_;
  Eigen::Matrix15d covariance_;

  size_t integration_steps_;
};

}  // namespace omni_slam
