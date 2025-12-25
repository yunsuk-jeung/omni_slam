#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <vector>
#include <tbb/concurrent_queue.h>
#include <opencv2/core.hpp>

namespace omni_slam {

class TrackingResult {
public:
  static constexpr size_t kCamNum   = 2;
  static constexpr size_t kLeftCam  = 0;
  static constexpr size_t kRightCam = 1;

  TrackingResult()  = default;
  ~TrackingResult() = default;
  TrackingResult(const TrackingResult& src) {
    ids_          = src.ids_;
    uvs_          = src.uvs_;
    track_counts_ = src.track_counts_;
    id_idx_       = src.id_idx_;
  }

  size_t Size(size_t cam_idx) const { return ids_[cam_idx].size(); }

  void Clear() {
    for (size_t i = 0; i < kCamNum; ++i) {
      ids_[i].clear();
      uvs_[i].clear();
      track_counts_[i].clear();
      id_idx_[i].clear();
    }
  }

  void Reserve(size_t cam_idx, size_t size) {
    const auto idx = cam_idx;
    ids_[idx].reserve(size);
    uvs_[idx].reserve(size);
    track_counts_[idx].reserve(size);
  }

  void PushBack(size_t cam_idx, const TrackingResult& kpts) {
    auto idx = ids_[cam_idx].size();
    ids_[cam_idx].insert(ids_[cam_idx].end(),
                         kpts.ids_[cam_idx].begin(),
                         kpts.ids_[cam_idx].end());

    for (; idx < ids_[cam_idx].size(); ++idx) {
      id_idx_[cam_idx][ids_[cam_idx][idx]] = idx;
    }

    uvs_[cam_idx].insert(uvs_[cam_idx].end(),
                         kpts.uvs_[cam_idx].begin(),
                         kpts.uvs_[cam_idx].end());

    track_counts_[cam_idx].insert(track_counts_[cam_idx].end(),
                                  kpts.track_counts_[cam_idx].begin(),
                                  kpts.track_counts_[cam_idx].end());
  }

  std::vector<int64_t>&           Ids(size_t cam_idx) { return ids_[cam_idx]; }
  const std::vector<int64_t>&     Ids(size_t cam_idx) const { return ids_[cam_idx]; }
  std::vector<cv::Point2f>&       Uvs(size_t cam_idx) { return uvs_[cam_idx]; }
  const std::vector<cv::Point2f>& Uvs(size_t cam_idx) const { return uvs_[cam_idx]; }
  std::vector<uint32_t>& TrackCounts(size_t cam_idx) { return track_counts_[cam_idx]; }
  const std::vector<uint32_t>& TrackCounts(size_t cam_idx) const {
    return track_counts_[cam_idx];
  }
  std::map<size_t, size_t>&       IdIndex(size_t cam_idx) { return id_idx_[cam_idx]; }
  const std::map<size_t, size_t>& IdIndex(size_t cam_idx) const {
    return id_idx_[cam_idx];
  }

private:
  std::array<std::vector<int64_t>, kCamNum>     ids_;
  std::array<std::vector<cv::Point2f>, kCamNum> uvs_;
  std::array<std::vector<uint32_t>, kCamNum>    track_counts_;
  std::array<std::map<size_t, size_t>, kCamNum> id_idx_;
  std::array<std::vector<cv::Mat>, kCamNum>     image_pyramids_;
};

class OpticalFlow {
public:
  OpticalFlow(tbb::concurrent_queue<std::array<cv::Mat, 2>>&          in_queue,
              tbb::concurrent_queue<std::shared_ptr<TrackingResult>>& out_queue)
    : in_queue_(in_queue)
    , out_queue_(out_queue) {}
  ~OpticalFlow() = default;

  void Run(std::atomic<bool>& running);

private:
  void                                                    Match();
  tbb::concurrent_queue<std::array<cv::Mat, 2>>&          in_queue_;
  tbb::concurrent_queue<std::shared_ptr<TrackingResult>>& out_queue_;
  cv::Mat                                                 prev_left_gray_;
  cv::Mat                                                 prev_right_gray_;
  bool                                                    has_prev_{false};
  std::shared_ptr<TrackingResult>                         prev_result_;
  uint64_t                                                next_feature_id_{0};
};

}  // namespace omni_slam
