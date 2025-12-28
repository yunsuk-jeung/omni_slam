#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <vector>
#include <tbb/concurrent_queue.h>
#include <opencv2/core.hpp>

namespace cv {
class CLAHE;
}

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

class Frame;
class OpticalFlow {
public:
  OpticalFlow() = delete;
  OpticalFlow(const size_t                                   cam_num,
              tbb::concurrent_queue<std::shared_ptr<Frame>>& in_queue,
              tbb::concurrent_queue<std::shared_ptr<Frame>>& out_queue);

  ~OpticalFlow() = default;

  void Run(std::atomic<bool>& running);

private:
  void PrepareImagesAndPyramids(std::shared_ptr<Frame>& curr_frame);

  void TrackMono(const std::shared_ptr<Frame>& curr_frame);
  void TrackStereo(const std::shared_ptr<Frame>& curr_frame);
  void DetectFeatures(const std::shared_ptr<Frame>& curr_frame);

private:
  const size_t                                   kCamNum;
  tbb::concurrent_queue<std::shared_ptr<Frame>>& in_queue_;
  tbb::concurrent_queue<std::shared_ptr<Frame>>& out_queue_;

  cv::Ptr<cv::CLAHE>     clahe_;
  std::shared_ptr<Frame> prev_frame_;
  uint64_t               next_feature_id_;
};

}  // namespace omni_slam
