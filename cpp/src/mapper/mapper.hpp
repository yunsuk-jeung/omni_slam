#pragma once

#include <atomic>
#include <cstdint>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>

#include "mapper/descriptor_extractor.hpp"
#include "mapper/hash_bow.hpp"
#include "mapper/nfr_optimizer.hpp"
#include "utils/concurrent_queue.hpp"

namespace omni_slam {

struct KeyframeImages {
  uint64_t             keyframe_id  = 0;
  int64_t              timestamp_ns = 0;
  std::vector<cv::Mat> images;
};

struct KeyframeMargData {
  MargPosePrior               prior;
  std::vector<KeyframeImages> keyframes;

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

  ConcurrentQueue<KeyframeMargData>* input_queue() { return &input_queue_; }

 private:
  void mapper_loop();
  void process(const KeyframeMargData& input);

  std::atomic<bool> running_{false};
  std::thread       mapper_thread_;

  ConcurrentQueue<KeyframeMargData> input_queue_;

  DescriptorExtractor      descriptor_extractor_;
  HashBow<kDescriptorBits> hash_bow_;
  NfrOptimizer             nfr_optimizer_;
};

}  // namespace omni_slam
