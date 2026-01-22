#include "svo_config.hpp"

#include <fstream>

#include <nlohmann/json.hpp>

#include "utils/logger.hpp"
#include "utils/types.hpp"
#include "camera_model/camera_model.hpp"

namespace omni_slam {
bool                             SVOConfig::debug                       = false;
bool                             SVOConfig::tbb                         = true;
bool                             SVOConfig::equalize_histogram          = false;
double                           SVOConfig::clahe_clip_limit            = 3.0;
int                              SVOConfig::clahe_tile_size             = 8;
int                              SVOConfig::optical_flow_patch_size     = 21;
float                            SVOConfig::optical_flow_dist_threshold = 5.0;
int                              SVOConfig::fast_threshold              = 20;
int                              SVOConfig::feature_grid_rows           = 4;
int                              SVOConfig::feature_grid_cols           = 4;
int                              SVOConfig::max_pyramid_level           = 3;
size_t                           SVOConfig::max_window                  = 0;
double                           SVOConfig::triangulation_dist_threshold = 0.0025;
std::vector<int>                 SVOConfig::camera_models;
std::vector<std::vector<double>> SVOConfig::camera_intrinsics;
std::vector<std::vector<double>> SVOConfig::camera_distortions;
std::vector<std::vector<int>>    SVOConfig::camera_resolutions;
std::vector<Sophus::SE3d>        SVOConfig::camera_T_bc;

void ParseCameraParams(const nlohmann::json&             node,
                       std::vector<int>*                 models,
                       std::vector<std::vector<double>>* intrinsics,
                       std::vector<std::vector<double>>* distortions,
                       std::vector<std::vector<int>>*    resolutions,
                       std::vector<Sophus::SE3d>*        T_bc) {
  if (!node.is_object()) {
    return;
  }

  if (models) {
    models->push_back(
      node.value("camera_model", static_cast<int>(CameraModel::PINHOLE_RAD_TAN)));
  }

  if (intrinsics) {
    std::vector<double> vals;
    if (node.contains("intrinsics") && node["intrinsics"].is_array()) {
      vals.reserve(node["intrinsics"].size());
      for (const auto& v : node["intrinsics"]) {
        vals.push_back(v.get<double>());
      }
    }
    intrinsics->push_back(std::move(vals));
  }

  if (distortions) {
    std::vector<double> vals;
    if (node.contains("distortions") && node["distortions"].is_array()) {
      vals.reserve(node["distortions"].size());
      for (const auto& v : node["distortions"]) {
        vals.push_back(v.get<double>());
      }
    }
    distortions->push_back(std::move(vals));
  }

  if (resolutions) {
    std::vector<int> vals;
    if (node.contains("resolution") && node["resolution"].is_array()) {
      vals.reserve(node["resolution"].size());
      for (const auto& v : node["resolution"]) {
        vals.push_back(v.get<int>());
      }
    }
    resolutions->push_back(std::move(vals));
  }
  if (T_bc && node.contains("Mbc") && node["Mbc"].is_array()
      && node["Mbc"].size() == 16) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (int i = 0; i < 16; ++i) {
      T(i / 4, i % 4) = node["Mbc"][i].get<double>();
    }
    T_bc->push_back(Sophus::SE3d(T));
  }
  else if (T_bc) {
    T_bc->push_back(Sophus::SE3d());
  }
}

void SVOConfig::ParseConfig(const std::string& file) {
  std::ifstream input(file);
  if (!input.is_open()) {
    return;
  }

  nlohmann::json config;
  try {
    input >> config;
  } catch (const std::exception&) {
    LogE("json parsing error");
    return;
  }

  debug                       = config.value("debug", debug);
  if (debug) {
    Logger::Init();
    spdlog::set_level(spdlog::level::debug);
  }
  tbb                         = config.value("tbb", tbb);
  equalize_histogram          = config.value("equalize_histogram", equalize_histogram);
  clahe_clip_limit            = config.value("clahe_clip_limit", clahe_clip_limit);
  clahe_tile_size             = config.value("clahe_tile_size", clahe_tile_size);
  optical_flow_patch_size     = config.value("optical_flow_patch_size",
                                         optical_flow_patch_size);
  optical_flow_dist_threshold = config.value("optical_flow_dist_threshold",
                                             optical_flow_dist_threshold);
  fast_threshold              = config.value("fast_threshold", fast_threshold);
  feature_grid_rows           = config.value("feature_grid_rows", feature_grid_rows);
  feature_grid_cols           = config.value("feature_grid_cols", feature_grid_cols);
  max_pyramid_level           = config.value("max_pyramid_level", max_pyramid_level);
  max_window                  = config.value("max_window", max_window);
  triangulation_dist_threshold =
    config.value("triangulation_dist_threshold", triangulation_dist_threshold);

  camera_models.clear();
  camera_intrinsics.clear();
  camera_distortions.clear();
  camera_resolutions.clear();
  camera_T_bc.clear();
  if (config.contains("cam0")) {
    ParseCameraParams(config["cam0"],
                      &camera_models,
                      &camera_intrinsics,
                      &camera_distortions,
                      &camera_resolutions,
                      &camera_T_bc);
  }
  if (config.contains("cam1")) {
    ParseCameraParams(config["cam1"],
                      &camera_models,
                      &camera_intrinsics,
                      &camera_distortions,
                      &camera_resolutions,
                      &camera_T_bc);
  }

  if (debug) {
    Logger::Info("SVOConfig.debug: {}", debug);
    Logger::Info("SVOConfig.tbb: {}", tbb);
    Logger::Info("SVOConfig.equalize_histogram: {}", equalize_histogram);
    Logger::Info("SVOConfig.clahe_clip_limit: {}", clahe_clip_limit);
    Logger::Info("SVOConfig.clahe_tile_size: {}", clahe_tile_size);
    Logger::Info("SVOConfig.optical_flow_patch_size: {}", optical_flow_patch_size);
    Logger::Info("SVOConfig.optical_flow_dist_threshold: {}",
                 optical_flow_dist_threshold);
    Logger::Info("SVOConfig.fast_threshold: {}", fast_threshold);
    Logger::Info("SVOConfig.feature_grid_rows: {}", feature_grid_rows);
    Logger::Info("SVOConfig.feature_grid_cols: {}", feature_grid_cols);
    Logger::Info("SVOConfig.max_pyramid_level: {}", max_pyramid_level);
    Logger::Info("SVOConfig.max_window: {}", max_window);
    Logger::Info("SVOConfig.triangulation_dist_threshold: {}",
                 triangulation_dist_threshold);
    Logger::Info("SVOConfig.camera_models: {}", camera_models.size());
  }
}
}  // namespace omni_slam
