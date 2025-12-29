#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <tbb/concurrent_queue.h>
#include <opencv2/core.hpp>
#include "feature_tracking/tracking_result.hpp"

namespace cv {
class CLAHE;
}

namespace omni_slam {

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
