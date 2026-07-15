#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

namespace omni_slam {

struct OdometryResult {
  struct TrackingData {
    std::vector<std::vector<size_t>>      ids;
    std::vector<std::vector<cv::Point2f>> uvs;
  };

  uint64_t                              frame_id     = 0;
  int64_t                               timestamp_ns = 0;
  std::vector<cv::Mat>                  images;
  std::vector<Sophus::SE3d>             T_w_b_window;
  std::vector<Sophus::SE3d>             T_b_c;
  std::vector<uint64_t>                 window_frame_ids;
  TrackingData                          tracking;
  std::vector<Eigen::Vector4f>          map_points;
  std::vector<std::vector<cv::Point2f>> map_point_uvs;

  Eigen::Vector3d acc_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyr_bias = Eigen::Vector3d::Zero();
};

}  // namespace omni_slam
