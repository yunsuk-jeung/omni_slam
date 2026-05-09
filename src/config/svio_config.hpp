#pragma once

#include <cstddef>
#include <string>

#include <Eigen/Core>

#include "config/svo_config.hpp"

namespace omni_slam {

class SVIOConfig : public SVOConfig {
public:
  static void ParseConfig(const std::string& file);

  // Maximum number of inertial states kept during VIO optimization.
  static size_t max_inertial_states;

  // Initial prior weight for bias blocks in VIO marginalizer.
  static double marginalizer_initial_bias_weight;

  // Multipliers applied to the square-root information of IMU residual groups.
  // Values below 1.0 make visual residuals dominate when IMU calibration/timing is rough.
  static double imu_residual_scale;
  static double imu_position_residual_scale;
  static double imu_rotation_residual_scale;
  static double imu_velocity_residual_scale;
  static double imu_bias_residual_scale;

  // IMU calibration outputs (typically from kalibr/allan tools).
  static double                acc_noise_density;
  static double                gyr_noise_density;
  static double                acc_random_walk;
  static double                gyr_random_walk;
  static double                imu_min_integration_dt_s;
  static const Eigen::Vector3d g_w;
};

}  // namespace omni_slam
