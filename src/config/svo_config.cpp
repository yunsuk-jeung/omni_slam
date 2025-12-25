#include "svo_config.hpp"

#include <fstream>

#include <nlohmann/json.hpp>

#include "utils/logger.hpp"

namespace omni_slam {
bool SVOConfig::debug = false;
bool SVOConfig::tbb = true;
bool SVOConfig::equalize_histogram = false;
int SVOConfig::max_pyramid_level = 3;

void SVOConfig::ParseConfig(const std::string& file) {
  std::ifstream input(file);
  if (!input.is_open()) {
    return;
  }

  nlohmann::json config;
  try {
    input >> config;
  }
  catch (const std::exception&) {
    return;
  }

  debug = config.value("debug", debug);
  tbb = config.value("tbb", tbb);
  equalize_histogram = config.value("equalize_histogram", equalize_histogram);
  max_pyramid_level = config.value("max_pyramid_level", max_pyramid_level);

  if (debug) {
    Logger::Info("SVOConfig.debug: {}", debug);
    Logger::Info("SVOConfig.tbb: {}", tbb);
    Logger::Info("SVOConfig.equalize_histogram: {}", equalize_histogram);
    Logger::Info("SVOConfig.max_pyramid_level: {}", max_pyramid_level);
  }
}
}  // namespace omni_slam
