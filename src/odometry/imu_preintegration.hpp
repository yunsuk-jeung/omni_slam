#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <sophus/so3.hpp>

#include "utils/eigen_utils.hpp"
#include "utils/types.hpp"

namespace omni_slam {
struct InertialState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d v_w_b    = Eigen::Vector3d::Zero();
  Eigen::Vector3d bias_acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d bias_gyr = Eigen::Vector3d::Zero();
};

class ImuPreintegration {
 public:
  ImuPreintegration() = delete;

  struct Parameters {
    double acc_noise_sigma      = 0.016;
    double gyr_noise_sigma      = 0.000282;
    double acc_bias_rw_sigma    = 0.001;
    double gyr_bias_rw_sigma    = 0.0001;
    double min_integration_dt_s = 1e-6;
  };

  struct CorrectedDelta {
    Eigen::Vector3d delta_p;
    Sophus::SO3d    delta_r;
    Eigen::Vector3d delta_v;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  explicit ImuPreintegration(
    uint64_t               from_frame_id,
    uint64_t               to_frame_id,
    const Eigen::Vector3d& bias_acc = Eigen::Vector3d::Zero(),
    const Eigen::Vector3d& bias_gyr = Eigen::Vector3d::Zero());
  ImuPreintegration(uint64_t               from_frame_id,
                    uint64_t               to_frame_id,
                    const Eigen::Vector3d& bias_acc,
                    const Eigen::Vector3d& bias_gyr,
                    const Parameters&      parameters);

  void Reset(const Eigen::Vector3d& bias_acc, const Eigen::Vector3d& bias_gyr);
  void SetBias(const Eigen::Vector3d& bias_acc,
               const Eigen::Vector3d& bias_gyr);
  void SetParameters(const Parameters& parameters);

  bool IntegrateMeasurement(const ImuData& imu0, const ImuData& imu1);
  bool IntegrateMeasurements(const std::vector<ImuData>& imu_data);

  // Re-integrate the buffered IMU samples with a new bias linearization point.
  // The cost function captures bias_acc_/bias_gyr_ at construction and only
  // applies first-order corrections; calling this after the optimizer updates
  // the bias keeps the deltas/Jacobians consistent with the new operating
  // point and prevents drift from stale linearization.
  bool Repropagate(const Eigen::Vector3d& bias_acc,
                   const Eigen::Vector3d& bias_gyr);

  const std::vector<ImuData>& GetImuMeasurements() const {
    return imu_measurements_;
  }

  CorrectedDelta GetBiasCorrectedDelta(const Eigen::Vector3d& bias_acc,
                                       const Eigen::Vector3d& bias_gyr) const;

  Eigen::Matrix15d GetInformation(double damping = 1e-12) const;

  const Sophus::SO3d&   GetDeltaR() const { return delta_r_; }
  const Eigen::Vector3d GetDeltaV() const { return delta_v_; }
  const Eigen::Vector3d GetDeltaP() const { return delta_p_; }
  double                GetDeltaTimeSec() const { return delta_t_sec_; }

  const Eigen::Vector3d& GetBiasAcc() const { return bias_acc_; }
  const Eigen::Vector3d& GetBiasGyr() const { return bias_gyr_; }
  uint64_t               GetFromFrameId() const { return from_frame_id_; }
  uint64_t               GetToFrameId() const { return to_frame_id_; }

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
  Parameters parameters_;

  Eigen::Vector3d delta_p_;
  Sophus::SO3d    delta_r_;
  Eigen::Vector3d delta_v_;
  double          delta_t_sec_;

  Eigen::Vector3d bias_acc_;
  Eigen::Vector3d bias_gyr_;
  uint64_t        from_frame_id_;
  uint64_t        to_frame_id_;

  Eigen::Matrix15d jacobian_;
  Eigen::Matrix15d covariance_;

  size_t integration_steps_;

  // Buffer of raw IMU samples consumed by IntegrateMeasurement(s).
  // Retained so Repropagate can replay integration with an updated bias
  // linearization point.
  std::vector<ImuData> imu_measurements_;
};

}  // namespace omni_slam
