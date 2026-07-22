#include "mapper/mapper.hpp"

#include "config/svo_config.hpp"
#include "feature_tracking/grid_detector.hpp"
#include "utils/logger.hpp"

namespace omni_slam {

Mapper::Mapper()  = default;

Mapper::~Mapper() {
  shutdown();
}

void Mapper::run() {
  Logger::info("Running Mapper");
  running_.store(true, std::memory_order_release);
  mapper_thread_ = std::thread(&Mapper::mapper_loop, this);
}

void Mapper::shutdown() {
  if (!running_.exchange(false, std::memory_order_acq_rel)) {
    return;
  }
  Logger::info("Shutting down Mapper");
  input_queue_.close();
  if (mapper_thread_.joinable()) {
    mapper_thread_.join();
  }
}

void Mapper::mapper_loop() {
  KeyframeWithPrior input;
  while (running_.load(std::memory_order_acquire)) {
    if (!input_queue_.wait()) {
      break;
    }
    if (!input_queue_.try_pop(input)) {
      continue;
    }
    process(input);
  }
}

void Mapper::process(const KeyframeWithPrior& input) {
  // TODO(A): descriptor extraction, loop detection + verification,
  // global BA/pose-graph with the recovered prior.
  std::vector<cv::KeyPoint> keypoints;
  if (!input.images.empty()) {
    keypoints = detect_grid_features(input.images.front(),
                                     SVOConfig::feature_grid_rows,
                                     SVOConfig::feature_grid_cols,
                                     SVOConfig::fast_threshold);
  }
  Logger::info("Mapper received keyframe {} ({} keypoints)",
               input.keyframe_id,
               keypoints.size());
}

}  // namespace omni_slam
