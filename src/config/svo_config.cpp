#include "svo_config.hpp"

#include <fstream>

#include <nlohmann/json.hpp>

#include "utils/logger.hpp"

namespace omni_slam {
bool   SVOConfig::debug                   = false;
bool   SVOConfig::tbb                     = true;
bool   SVOConfig::equalize_histogram      = false;
double SVOConfig::clahe_clip_limit        = 3.0;
int    SVOConfig::clahe_tile_size         = 8;
int    SVOConfig::optical_flow_patch_size = 21;
int    SVOConfig::max_pyramid_level       = 3;

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

  debug                   = config.value("debug", debug);
  tbb                     = config.value("tbb", tbb);
  equalize_histogram      = config.value("equalize_histogram", equalize_histogram);
  clahe_clip_limit        = config.value("clahe_clip_limit", clahe_clip_limit);
  clahe_tile_size         = config.value("clahe_tile_size", clahe_tile_size);
  optical_flow_patch_size = config.value("optical_flow_patch_size",
                                         optical_flow_patch_size);
  max_pyramid_level       = config.value("max_pyramid_level", max_pyramid_level);

  if (debug) {
    Logger::Info("SVOConfig.debug: {}", debug);
    Logger::Info("SVOConfig.tbb: {}", tbb);
    Logger::Info("SVOConfig.equalize_histogram: {}", equalize_histogram);
    Logger::Info("SVOConfig.clahe_clip_limit: {}", clahe_clip_limit);
    Logger::Info("SVOConfig.clahe_tile_size: {}", clahe_tile_size);
    Logger::Info("SVOConfig.optical_flow_patch_size: {}", optical_flow_patch_size);
    Logger::Info("SVOConfig.max_pyramid_level: {}", max_pyramid_level);
  }
}
}  // namespace omni_slam
