#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>

#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <sophus/se3.hpp>

namespace omni_slam {

struct OdometryResult {
  struct TrackingData {
    std::vector<std::vector<size_t>>      ids;
    std::vector<std::vector<cv::Point2f>> uvs;
  };

  int64_t                      timestamp_ns = 0;
  std::vector<cv::Mat>         images;
  std::vector<Sophus::SE3d>    T_w_b_window;
  std::vector<Sophus::SE3d>    T_b_c;
  std::vector<uint64_t>        window_frame_ids;
  TrackingData                 tracking;
  std::vector<Eigen::Vector4f> map_points;
};

}  // namespace omni_slam
