#pragma once

#include <string>

#include <Eigen/Core>

#include "config/svo_config.hpp"

namespace omni_slam {

class SVIOConfig : public SVOConfig {
public:
  static void ParseConfig(const std::string& file);

  // IMU calibration outputs (typically from kalibr/allan tools).
  static double                acc_noise_density;
  static double                gyr_noise_density;
  static double                acc_random_walk;
  static double                gyr_random_walk;
  static double                imu_min_integration_dt_s;
  static const Eigen::Vector3d g_w;
};

}  // namespace omni_slam
