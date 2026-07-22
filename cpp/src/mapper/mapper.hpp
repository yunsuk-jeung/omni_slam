#pragma once

#include <atomic>
#include <cstdint>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

#include "utils/concurrent_queue.hpp"

namespace omni_slam {

struct KeyframeWithPrior {
  uint64_t             keyframe_id  = 0;
  int64_t              timestamp_ns = 0;
  std::vector<cv::Mat> images;
  Sophus::SE3d         T_w_b_lin;
  Eigen::MatrixXd      pose_information;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class Mapper {
 public:
  Mapper();
  ~Mapper();

  Mapper(const Mapper&)            = delete;
  Mapper& operator=(const Mapper&) = delete;

  void run();
  void shutdown();

  ConcurrentQueue<KeyframeWithPrior>* input_queue() { return &input_queue_; }

 private:
  void mapper_loop();
  void process(const KeyframeWithPrior& input);

  std::atomic<bool> running_{false};
  std::thread       mapper_thread_;

  ConcurrentQueue<KeyframeWithPrior> input_queue_;
};

}  // namespace omni_slam
