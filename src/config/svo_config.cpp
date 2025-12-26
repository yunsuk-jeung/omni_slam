#include "svo_config.hpp"

#include <fstream>

#include <nlohmann/json.hpp>

#include "utils/logger.hpp"

namespace omni_slam {
bool   SVOConfig::debug                       = false;
bool   SVOConfig::tbb                         = true;
bool   SVOConfig::equalize_histogram          = false;
double SVOConfig::clahe_clip_limit            = 3.0;
int    SVOConfig::clahe_tile_size             = 8;
int    SVOConfig::optical_flow_patch_size     = 21;
float  SVOConfig::optical_flow_dist_threshold = 5.0;
int    SVOConfig::fast_threshold              = 20;
int    SVOConfig::feature_grid_rows           = 4;
int    SVOConfig::feature_grid_cols           = 4;
int    SVOConfig::max_pyramid_level           = 3;

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
  }
}
}  // namespace omni_slam
