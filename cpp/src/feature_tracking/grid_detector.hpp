#pragma once

#include <vector>

#include <opencv2/core.hpp>

namespace omni_slam {

std::vector<cv::KeyPoint> detect_grid_features(
  const cv::Mat&                  image,
  int                             grid_rows,
  int                             grid_cols,
  int                             fast_threshold,
  const std::vector<cv::Point2f>& occupied     = {},
  int                             max_per_cell = 1);

}  // namespace omni_slam
