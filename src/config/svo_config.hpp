#pragma once
#include <string>

namespace omni_slam {
class SVOConfig {
public:
  static void   ParseConfig(const std::string& file);
  static bool   debug;
  static bool   tbb;
  static bool   equalize_histogram;
  static double clahe_clip_limit;
  static int    clahe_tile_size;
  static int    optical_flow_patch_size;
  static float  optical_flow_dist_threshold;
  static int    fast_threshold;
  static int    feature_grid_rows;
  static int    feature_grid_cols;
  static int    max_pyramid_level;
};
}  // namespace omni_slam
