#pragma once
#include <string>

namespace omni_slam {
class SVOConfig {
public:
  static void ParseConfig(const std::string& file);
  static bool debug;
  static bool tbb;
  static bool equalize_histogram;
  static int  max_pyramid_level;
};
}  // namespace omni_slam