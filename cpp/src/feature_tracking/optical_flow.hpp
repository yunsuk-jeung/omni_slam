#pragma once

#include <array>
#include <cstdint>
#include <memory>

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
  explicit OpticalFlow(const size_t cam_num);

  ~OpticalFlow() = default;

  void process(std::shared_ptr<Frame>& curr_frame);

 private:
  void prepare_images_and_pyramids(std::shared_ptr<Frame>& curr_frame);

  void track_mono(const std::shared_ptr<Frame>& curr_frame);
  void track_stereo(const std::shared_ptr<Frame>& curr_frame);
  void detect_features(const std::shared_ptr<Frame>& curr_frame);

 private:
  const size_t kCamNum;

  cv::Ptr<cv::CLAHE>     clahe_;
  std::shared_ptr<Frame> prev_frame_;
  uint64_t               next_feature_id_;
};

}  // namespace omni_slam
