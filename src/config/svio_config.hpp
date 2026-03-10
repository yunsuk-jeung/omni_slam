#pragma once

#include <string>

#include <Eigen/Core>

#include "config/svo_config.hpp"

namespace omni_slam {

class SVIOConfig : public SVOConfig {
public:
  static void ParseConfig(const std::string& file);

  // IMU calibration outputs (typically from kalibr/allan tools).
  static double          imu_acc_noise_density;
  static double          imu_gyr_noise_density;
  static double          imu_acc_random_walk;
  static double          imu_gyr_random_walk;
  static double          imu_min_integration_dt_s;
  static Eigen::Vector3d imu_init_bias_acc;
  static Eigen::Vector3d imu_init_bias_gyr;
  static Eigen::Vector3d gravity_vector_w;
};

}  // namespace omni_slam
