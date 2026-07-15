#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>

#include <opencv2/core.hpp>
#include <tbb/concurrent_queue.h>

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

  void run(std::atomic<bool>& running);

 private:
  void prepare_images_and_pyramids(std::shared_ptr<Frame>& curr_frame);
  void process(std::shared_ptr<Frame>& curr_frame);

  void track_mono(const std::shared_ptr<Frame>& curr_frame);
  void track_stereo(const std::shared_ptr<Frame>& curr_frame);
  void detect_features(const std::shared_ptr<Frame>& curr_frame);

 private:
  const size_t                                   kCamNum;
  tbb::concurrent_queue<std::shared_ptr<Frame>>& in_queue_;
  tbb::concurrent_queue<std::shared_ptr<Frame>>& out_queue_;

  cv::Ptr<cv::CLAHE>     clahe_;
  std::shared_ptr<Frame> prev_frame_;
  uint64_t               next_feature_id_;
};

}  // namespace omni_slam
