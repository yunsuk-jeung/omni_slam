#pragma once

#include <cstdint>
#include <map>
#include <vector>
#include <opencv2/core.hpp>

namespace omni_slam {

class TrackingResult {
public:
  TrackingResult();
  TrackingResult(const size_t& cam_num);
  ~TrackingResult();

  size_t Size(size_t cam_idx) const;

  void Clear();
  void Reserve(size_t cam_idx, size_t size);

  void AddFeature(size_t cam_idx, const cv::Point2f& uv, int64_t id);

  std::vector<size_t>&       Ids(size_t cam_idx) { return ids_[cam_idx]; }
  const std::vector<size_t>& Ids(size_t cam_idx) const { return ids_[cam_idx]; }

  std::map<size_t, size_t>&       IdIndex(size_t cam_idx) { return id_idx_[cam_idx]; }
  const std::map<size_t, size_t>& IdIndex(size_t cam_idx) const {
    return id_idx_[cam_idx];
  }

  std::vector<cv::Point2f>&       Uvs(size_t cam_idx) { return uvs_[cam_idx]; }
  const std::vector<cv::Point2f>& Uvs(size_t cam_idx) const { return uvs_[cam_idx]; }

private:
  const size_t                          kCamNum;
  std::vector<std::vector<size_t>>      ids_;
  std::vector<std::map<size_t, size_t>> id_idx_;
  std::vector<std::vector<cv::Point2f>> uvs_;
};

}  // namespace omni_slam
