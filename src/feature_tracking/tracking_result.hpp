#pragma once

#include <cstdint>
#include <map>
#include <vector>
#include <opencv2/core.hpp>

namespace omni_slam {

class TrackingResult {
public:
  TrackingResult() = delete;
  TrackingResult(const size_t& cam_num)
    : kCamNum{cam_num}
    , ids_(cam_num)
    , uvs_(cam_num){};

  ~TrackingResult() = default;

  size_t GetSize(size_t cam_idx) const { return ids_[cam_idx].size(); }

  void Clear() {
    for (size_t i = 0; i < kCamNum; ++i) {
      ids_[i].clear();
      uvs_[i].clear();
    }
  }

  void Reserve(size_t cam_idx, size_t size) {
    const auto idx = cam_idx;
    ids_[idx].reserve(size);
    uvs_[idx].reserve(size);
  }

  void AddFeature(size_t cam_idx, const cv::Point2f& uv, int64_t id) {
    ids_[cam_idx].push_back(id);
    uvs_[cam_idx].push_back(uv);
  }

  std::vector<size_t>&       GetIds(size_t cam_idx) { return ids_[cam_idx]; }
  const std::vector<size_t>& GetIds(size_t cam_idx) const { return ids_[cam_idx]; }

  std::vector<cv::Point2f>&       GetUvs(size_t cam_idx) { return uvs_[cam_idx]; }
  const std::vector<cv::Point2f>& GetUvs(size_t cam_idx) const { return uvs_[cam_idx]; }

private:
  const size_t                          kCamNum;
  std::vector<std::vector<size_t>>      ids_;
  std::vector<std::vector<cv::Point2f>> uvs_;
};

}  // namespace omni_slam
