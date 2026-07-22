#include "feature_tracking/grid_detector.hpp"

#include <algorithm>

#include <opencv2/features2d.hpp>

namespace omni_slam {

std::vector<cv::KeyPoint> detect_grid_features(
  const cv::Mat&                  image,
  int                             grid_rows,
  int                             grid_cols,
  int                             fast_threshold,
  const std::vector<cv::Point2f>& occupied,
  int                             max_per_cell) {
  grid_rows    = std::max(1, grid_rows);
  grid_cols    = std::max(1, grid_cols);
  max_per_cell = std::max(1, max_per_cell);

  const int cell_w = std::max(1, image.cols / grid_cols);
  const int cell_h = std::max(1, image.rows / grid_rows);

  std::vector<bool> cell_has_feature(grid_rows * grid_cols, false);
  for (const auto& uv : occupied) {
    const int col = std::min(static_cast<int>(uv.x / cell_w), grid_cols - 1);
    const int row = std::min(static_cast<int>(uv.y / cell_h), grid_rows - 1);
    cell_has_feature[row * grid_cols + col] = true;
  }

  std::vector<cv::KeyPoint> detected;
  detected.reserve(static_cast<size_t>(grid_rows) * grid_cols * max_per_cell);

  for (int row = 0; row < grid_rows; ++row) {
    for (int col = 0; col < grid_cols; ++col) {
      if (cell_has_feature[row * grid_cols + col]) {
        continue;
      }
      const int x0 = col * cell_w;
      const int y0 = row * cell_h;
      const int x1 = (col == grid_cols - 1) ? image.cols : (col + 1) * cell_w;
      const int y1 = (row == grid_rows - 1) ? image.rows : (row + 1) * cell_h;
      if (x1 <= x0 || y1 <= y0) {
        continue;
      }

      const cv::Rect            roi(x0, y0, x1 - x0, y1 - y0);
      std::vector<cv::KeyPoint> keypoints;
      cv::FAST(image(roi), keypoints, fast_threshold, true);
      if (keypoints.empty()) {
        continue;
      }

      const size_t keep =
        std::min(static_cast<size_t>(max_per_cell), keypoints.size());
      std::partial_sort(keypoints.begin(),
                        keypoints.begin() + keep,
                        keypoints.end(),
                        [](const cv::KeyPoint& a, const cv::KeyPoint& b) {
                          return a.response > b.response;
                        });
      for (size_t i = 0; i < keep; ++i) {
        cv::KeyPoint kp = keypoints[i];
        kp.pt.x += static_cast<float>(x0);
        kp.pt.y += static_cast<float>(y0);
        detected.push_back(kp);
      }
    }
  }
  return detected;
}

}  // namespace omni_slam
