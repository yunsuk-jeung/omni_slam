#include <algorithm>
#include <cmath>
#include <fstream>
#include <vector>

#include <nlohmann/json.hpp>

#include "config/svio_config.hpp"
#include "utils/logger.hpp"

namespace omni_slam {

constexpr double kDefaultMarginalizerInitialBiasWeight = 100.0;

size_t                SVIOConfig::max_inertial_states              = 5;
bool                  SVIOConfig::enable_fej                       = true;
double SVIOConfig::marginalizer_initial_bias_weight =
  kDefaultMarginalizerInitialBiasWeight;
double                SVIOConfig::imu_residual_scale               = 1.0;
double                SVIOConfig::imu_position_residual_scale      = 1.0;
double                SVIOConfig::imu_rotation_residual_scale      = 1.0;
double                SVIOConfig::imu_velocity_residual_scale      = 1.0;
double                SVIOConfig::imu_bias_residual_scale          = 1.0;
double                SVIOConfig::acc_noise_density                = 0.016;
double                SVIOConfig::gyr_noise_density                = 0.000282;
double                SVIOConfig::acc_random_walk                  = 0.001;
double                SVIOConfig::gyr_random_walk                  = 0.0001;
double                SVIOConfig::imu_min_integration_dt_s         = 1e-6;
const Eigen::Vector3d SVIOConfig::g_w = Eigen::Vector3d(0.0, 0.0, -9.81);

namespace {

double read_double_with_aliases(const nlohmann::json&           node,
                                const std::vector<std::string>& aliases,
                                double                          default_value) {
  for (const auto& key : aliases) {
    if (node.contains(key) && node[key].is_number()) {
      return node[key].get<double>();
    }
  }
  return default_value;
}

size_t read_size_t_with_aliases(const nlohmann::json&           node,
                                const std::vector<std::string>& aliases,
                                size_t                          default_value) {
  for (const auto& key : aliases) {
    if (!node.contains(key) || !node[key].is_number()) {
      continue;
    }

    if (node[key].is_number_unsigned()) {
      return node[key].get<size_t>();
    }

    if (node[key].is_number_integer()) {
      const auto value = node[key].get<long long>();
      if (value > 0) {
        return static_cast<size_t>(value);
      }
      continue;
    }

    const double value = node[key].get<double>();
    if (value > 0.0) {
      return static_cast<size_t>(value);
    }
  }
  return default_value;
}

}  // namespace

void SVIOConfig::ParseConfig(const std::string& file) {
  // Parse visual/stereo parameters first.
  SVOConfig::parse_config(file);

  std::ifstream input(file);
  if (!input.is_open()) {
    Logger::warn("Failed to open SVIO config: {}", file);
    return;
  }

  nlohmann::json config;
  try {
    input >> config;
  } catch (const std::exception&) {
    Logger::warn("SVIO config JSON parse error: {}", file);
    return;
  }

  const nlohmann::json* imu_node = &config;
  if (config.contains("imu") && config["imu"].is_object()) {
    imu_node = &config["imu"];
  }
  const nlohmann::json& imu = *imu_node;

  // Each IMU field falls back in order: nested "imu" alias keys, then the
  // canonical root-level "imu_xxx" key, then the existing static default.

  marginalizer_initial_bias_weight =
    config.value("marginalizer_initial_bias_weight",
                 marginalizer_initial_bias_weight);
  max_inertial_states = read_size_t_with_aliases(config,
                                                 {"max_inertial_states"},
                                                 max_inertial_states);
  enable_fej          = config.value("enable_fej", enable_fej);
  imu_residual_scale =
    read_double_with_aliases(imu,
                             {"residual_scale", "imu_residual_scale"},
                             config.value("imu_residual_scale",
                                          imu_residual_scale));
  imu_position_residual_scale =
    read_double_with_aliases(imu,
                             {"position_residual_scale",
                              "pos_residual_scale",
                              "imu_position_residual_scale"},
                             config.value("imu_position_residual_scale",
                                          imu_position_residual_scale));
  imu_rotation_residual_scale =
    read_double_with_aliases(imu,
                             {"rotation_residual_scale",
                              "rot_residual_scale",
                              "imu_rotation_residual_scale"},
                             config.value("imu_rotation_residual_scale",
                                          imu_rotation_residual_scale));
  imu_velocity_residual_scale =
    read_double_with_aliases(imu,
                             {"velocity_residual_scale",
                              "vel_residual_scale",
                              "imu_velocity_residual_scale"},
                             config.value("imu_velocity_residual_scale",
                                          imu_velocity_residual_scale));
  imu_bias_residual_scale =
    read_double_with_aliases(imu,
                             {"bias_residual_scale", "imu_bias_residual_scale"},
                             config.value("imu_bias_residual_scale",
                                          imu_bias_residual_scale));
  acc_noise_density = read_double_with_aliases(imu,
                                               {"acc_noise_density",
                                                "accelerometer_noise_density",
                                                "accel_noise_std",
                                                "acc_noise_sigma"},
                                               acc_noise_density);
  gyr_noise_density = read_double_with_aliases(imu,
                                               {"gyr_noise_density",
                                                "gyro_noise_density",
                                                "gyroscope_noise_density",
                                                "gyro_noise_std",
                                                "gyr_noise_sigma"},
                                               gyr_noise_density);
  acc_random_walk   = read_double_with_aliases(imu,
                                               {"acc_random_walk",
                                                "accelerometer_random_walk",
                                                "accel_bias_std",
                                                "acc_bias_rw_sigma"},
                                             acc_random_walk);
  gyr_random_walk   = read_double_with_aliases(imu,
                                               {"gyr_random_walk",
                                                "gyro_random_walk",
                                                "gyroscope_random_walk",
                                                "gyro_bias_std",
                                                "gyr_bias_rw_sigma"},
                                             gyr_random_walk);
  imu_min_integration_dt_s =
    read_double_with_aliases(imu,
                             {"min_integration_dt_s", "integration_min_dt_s"},
                             imu_min_integration_dt_s);

  constexpr double kMinNoiseParam    = 1e-12;
  constexpr double kMinIntegrationDt = 1e-9;
  acc_noise_density        = std::max(acc_noise_density, kMinNoiseParam);
  gyr_noise_density        = std::max(gyr_noise_density, kMinNoiseParam);
  acc_random_walk          = std::max(acc_random_walk, kMinNoiseParam);
  gyr_random_walk          = std::max(gyr_random_walk, kMinNoiseParam);
  imu_min_integration_dt_s = std::max(imu_min_integration_dt_s, kMinIntegrationDt);
  if (!std::isfinite(marginalizer_initial_bias_weight)
      || marginalizer_initial_bias_weight <= 0.0) {
    marginalizer_initial_bias_weight = kDefaultMarginalizerInitialBiasWeight;
  }
  if (max_inertial_states < 1) {
    max_inertial_states = 1;
  }
  auto sanitize_scale = [](double scale) {
    return (std::isfinite(scale) && scale >= 0.0) ? scale : 1.0;
  };
  imu_residual_scale          = sanitize_scale(imu_residual_scale);
  imu_position_residual_scale = sanitize_scale(imu_position_residual_scale);
  imu_rotation_residual_scale = sanitize_scale(imu_rotation_residual_scale);
  imu_velocity_residual_scale = sanitize_scale(imu_velocity_residual_scale);
  imu_bias_residual_scale     = sanitize_scale(imu_bias_residual_scale);

  if (SVIOConfig::debug) {
    Logger::info("SVIOConfig.max_inertial_states: {}", max_inertial_states);
    Logger::info("SVIOConfig.marginalizer_initial_bias_weight: {}",
                 marginalizer_initial_bias_weight);
    Logger::info("SVIOConfig.imu_residual_scale: {}", imu_residual_scale);
    Logger::info("SVIOConfig.imu_position_residual_scale: {}",
                 imu_position_residual_scale);
    Logger::info("SVIOConfig.imu_rotation_residual_scale: {}",
                 imu_rotation_residual_scale);
    Logger::info("SVIOConfig.imu_velocity_residual_scale: {}",
                 imu_velocity_residual_scale);
    Logger::info("SVIOConfig.imu_bias_residual_scale: {}",
                 imu_bias_residual_scale);
    Logger::info("SVIOConfig.imu_acc_noise_density: {}", acc_noise_density);
    Logger::info("SVIOConfig.imu_gyr_noise_density: {}", gyr_noise_density);
    Logger::info("SVIOConfig.imu_acc_random_walk: {}", acc_random_walk);
    Logger::info("SVIOConfig.imu_gyr_random_walk: {}", gyr_random_walk);
    Logger::info("SVIOConfig.imu_min_integration_dt_s: {}",
                 imu_min_integration_dt_s);
  }
}

}  // namespace omni_slam
