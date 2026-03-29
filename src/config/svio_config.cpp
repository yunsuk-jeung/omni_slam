#include "config/svio_config.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <vector>

#include <nlohmann/json.hpp>

#include "utils/logger.hpp"

namespace omni_slam {

double                SVIOConfig::marginalizer_initial_bias_weight = 100.0;
double                SVIOConfig::acc_noise_density                = 0.08;
double                SVIOConfig::gyr_noise_density                = 0.004;
double                SVIOConfig::acc_random_walk                  = 0.0002;
double                SVIOConfig::gyr_random_walk                  = 0.00002;
double                SVIOConfig::imu_min_integration_dt_s         = 1e-6;
const Eigen::Vector3d SVIOConfig::g_w = Eigen::Vector3d(0.0, 0.0, -9.81);

namespace {

double ReadDoubleWithAliases(const nlohmann::json&           node,
                             const std::vector<std::string>& aliases,
                             double                          default_value) {
  for (const auto& key : aliases) {
    if (node.contains(key) && node[key].is_number()) {
      return node[key].get<double>();
    }
  }
  return default_value;
}

size_t ReadSizeTWithAliases(const nlohmann::json&           node,
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

void ReadVec3WithAliases(const nlohmann::json&           node,
                         const std::vector<std::string>& aliases,
                         Eigen::Vector3d*                out) {
  if (!out) {
    return;
  }
  for (const auto& key : aliases) {
    if (!node.contains(key) || !node[key].is_array() || node[key].size() != 3) {
      continue;
    }
    (*out)(0) = node[key][0].get<double>();
    (*out)(1) = node[key][1].get<double>();
    (*out)(2) = node[key][2].get<double>();
    return;
  }
}

}  // namespace

void SVIOConfig::ParseConfig(const std::string& file) {
  // Parse visual/stereo parameters first.
  SVOConfig::ParseConfig(file);

  std::ifstream input(file);
  if (!input.is_open()) {
    Logger::Warn("Failed to open SVIO config: {}", file);
    return;
  }

  nlohmann::json config;
  try {
    input >> config;
  } catch (const std::exception&) {
    Logger::Warn("SVIO config JSON parse error: {}", file);
    return;
  }

  const nlohmann::json* imu_node = &config;
  if (config.contains("imu") && config["imu"].is_object()) {
    imu_node = &config["imu"];
  }

  marginalizer_initial_bias_weight = config.value("marginalizer_initial_bias_weight",
                                                  marginalizer_initial_bias_weight);
  acc_noise_density                = ReadDoubleWithAliases(*imu_node,
                                                           {"acc_noise_density",
                                                            "accelerometer_noise_density",
                                                            "acc_noise_sigma"},
                                            acc_noise_density);
  gyr_noise_density                = ReadDoubleWithAliases(*imu_node,
                                                           {"gyr_noise_density",
                                                            "gyro_noise_density",
                                                            "gyroscope_noise_density",
                                                            "gyr_noise_sigma"},
                                            gyr_noise_density);
  acc_random_walk                  = ReadDoubleWithAliases(*imu_node,
                                                           {"acc_random_walk",
                                                            "accelerometer_random_walk",
                                                            "acc_bias_rw_sigma"},
                                          acc_random_walk);
  gyr_random_walk                  = ReadDoubleWithAliases(*imu_node,
                                                           {"gyr_random_walk",
                                                            "gyro_random_walk",
                                                            "gyroscope_random_walk",
                                                            "gyr_bias_rw_sigma"},
                                          gyr_random_walk);
  imu_min_integration_dt_s         = ReadDoubleWithAliases(*imu_node,
                                                           {"min_integration_dt_s",
                                                            "integration_min_dt_s"},
                                                   imu_min_integration_dt_s);

  acc_noise_density        = std::max(acc_noise_density, 1e-12);
  gyr_noise_density        = std::max(gyr_noise_density, 1e-12);
  acc_random_walk          = std::max(acc_random_walk, 1e-12);
  gyr_random_walk          = std::max(gyr_random_walk, 1e-12);
  imu_min_integration_dt_s = std::max(imu_min_integration_dt_s, 1e-9);
  if (!std::isfinite(marginalizer_initial_bias_weight)
      || marginalizer_initial_bias_weight <= 0.0) {
    marginalizer_initial_bias_weight = 100.0;
  }

  if (SVIOConfig::debug) {
    Logger::Info("SVIOConfig.marginalizer_initial_bias_weight: {}",
                 marginalizer_initial_bias_weight);
    Logger::Info("SVIOConfig.imu_acc_noise_density: {}", acc_noise_density);
    Logger::Info("SVIOConfig.imu_gyr_noise_density: {}", gyr_noise_density);
    Logger::Info("SVIOConfig.imu_acc_random_walk: {}", acc_random_walk);
    Logger::Info("SVIOConfig.imu_gyr_random_walk: {}", gyr_random_walk);
    Logger::Info("SVIOConfig.imu_min_integration_dt_s: {}", imu_min_integration_dt_s);
  }
}

}  // namespace omni_slam
