#include "config/svio_config.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <vector>

#include <nlohmann/json.hpp>

#include "utils/logger.hpp"

namespace omni_slam {

double          SVIOConfig::imu_acc_noise_density   = 0.08;
double          SVIOConfig::imu_gyr_noise_density   = 0.004;
double          SVIOConfig::imu_acc_random_walk     = 0.0002;
double          SVIOConfig::imu_gyr_random_walk     = 0.00002;
double          SVIOConfig::imu_min_integration_dt_s = 1e-6;
Eigen::Vector3d SVIOConfig::imu_init_bias_acc       = Eigen::Vector3d::Zero();
Eigen::Vector3d SVIOConfig::imu_init_bias_gyr       = Eigen::Vector3d::Zero();
Eigen::Vector3d SVIOConfig::gravity_vector_w        = Eigen::Vector3d(0.0, 0.0, -9.81);

namespace {

double ReadDoubleWithAliases(const nlohmann::json&        node,
                             const std::vector<std::string>& aliases,
                             double                         default_value) {
  for (const auto& key : aliases) {
    if (node.contains(key) && node[key].is_number()) {
      return node[key].get<double>();
    }
  }
  return default_value;
}

void ReadVec3WithAliases(const nlohmann::json&        node,
                         const std::vector<std::string>& aliases,
                         Eigen::Vector3d*              out) {
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

  imu_acc_noise_density = ReadDoubleWithAliases(*imu_node,
                                                {"acc_noise_density",
                                                 "accelerometer_noise_density",
                                                 "acc_noise_sigma"},
                                                imu_acc_noise_density);
  imu_gyr_noise_density = ReadDoubleWithAliases(*imu_node,
                                                {"gyr_noise_density",
                                                 "gyro_noise_density",
                                                 "gyroscope_noise_density",
                                                 "gyr_noise_sigma"},
                                                imu_gyr_noise_density);
  imu_acc_random_walk = ReadDoubleWithAliases(*imu_node,
                                              {"acc_random_walk",
                                               "accelerometer_random_walk",
                                               "acc_bias_rw_sigma"},
                                              imu_acc_random_walk);
  imu_gyr_random_walk = ReadDoubleWithAliases(*imu_node,
                                              {"gyr_random_walk",
                                               "gyro_random_walk",
                                               "gyroscope_random_walk",
                                               "gyr_bias_rw_sigma"},
                                              imu_gyr_random_walk);
  imu_min_integration_dt_s = ReadDoubleWithAliases(*imu_node,
                                                   {"min_integration_dt_s",
                                                    "integration_min_dt_s"},
                                                   imu_min_integration_dt_s);

  ReadVec3WithAliases(*imu_node,
                      {"init_bias_acc", "initial_bias_acc", "acc_bias"},
                      &imu_init_bias_acc);
  ReadVec3WithAliases(*imu_node,
                      {"init_bias_gyr", "initial_bias_gyr", "gyr_bias", "gyro_bias"},
                      &imu_init_bias_gyr);

  ReadVec3WithAliases(*imu_node, {"gravity_vector_w", "gravity_vector"}, &gravity_vector_w);
  if (gravity_vector_w.norm() <= 0.0) {
    const double g_mag = ReadDoubleWithAliases(*imu_node,
                                               {"gravity_magnitude", "gravity_norm", "g_norm"},
                                               9.81);
    gravity_vector_w = Eigen::Vector3d(0.0, 0.0, -std::abs(g_mag));
  }

  imu_acc_noise_density   = std::max(imu_acc_noise_density, 1e-12);
  imu_gyr_noise_density   = std::max(imu_gyr_noise_density, 1e-12);
  imu_acc_random_walk     = std::max(imu_acc_random_walk, 1e-12);
  imu_gyr_random_walk     = std::max(imu_gyr_random_walk, 1e-12);
  imu_min_integration_dt_s = std::max(imu_min_integration_dt_s, 1e-9);

  if (SVIOConfig::debug) {
    Logger::Info("SVIOConfig.imu_acc_noise_density: {}", imu_acc_noise_density);
    Logger::Info("SVIOConfig.imu_gyr_noise_density: {}", imu_gyr_noise_density);
    Logger::Info("SVIOConfig.imu_acc_random_walk: {}", imu_acc_random_walk);
    Logger::Info("SVIOConfig.imu_gyr_random_walk: {}", imu_gyr_random_walk);
    Logger::Info("SVIOConfig.imu_min_integration_dt_s: {}", imu_min_integration_dt_s);
    Logger::Info("SVIOConfig.imu_init_bias_acc: [{}, {}, {}]",
                 imu_init_bias_acc.x(),
                 imu_init_bias_acc.y(),
                 imu_init_bias_acc.z());
    Logger::Info("SVIOConfig.imu_init_bias_gyr: [{}, {}, {}]",
                 imu_init_bias_gyr.x(),
                 imu_init_bias_gyr.y(),
                 imu_init_bias_gyr.z());
    Logger::Info("SVIOConfig.gravity_vector_w: [{}, {}, {}]",
                 gravity_vector_w.x(),
                 gravity_vector_w.y(),
                 gravity_vector_w.z());
  }
}

}  // namespace omni_slam
