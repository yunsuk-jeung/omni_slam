#include "odometry/stereo_vo.hpp"

#include <chrono>

#include "config/svo_config.hpp"
#include "utils/logger.hpp"

namespace omni_slam {

bool StereoVO::Initialize(const std::string& config_path) {
  Logger::Info("Initializing VO Pipeline");
  if (config_path.empty()) {
    Logger::Warn("Empty config path for VO pipeline");
    return false;
  }

  SVOConfig::ParseConfig(config_path);
  Logger::Info("Loaded VO config: {}", config_path.c_str());
  return true;
}

void StereoVO::Run() {
  Logger::Info("Running VO Pipeline");
  running_.store(true, std::memory_order_release);

  optical_flow_thread_ = std::thread(&StereoVO::OpticalFlowLoop, this);
  estimator_thread_    = std::thread(&StereoVO::EstimatorLoop, this);
}

void StereoVO::Shutdown() {
  Logger::Info("Shutting down VO Pipeline");
  running_.store(false, std::memory_order_release);

  if (optical_flow_thread_.joinable()) {
    optical_flow_thread_.join();
  }
  if (estimator_thread_.joinable()) {
    estimator_thread_.join();
  }
}

void StereoVO::OnCameraFrame(const std::array<cv::Mat, 2>& images) {
  if (images[0].empty()) {
    Logger::Warn("Received camera frame with empty left image");
    return;
  }

  image_queue_.push(images);
}

void StereoVO::OpticalFlowLoop() {
  optical_flow_.Run(running_);
}

void StereoVO::EstimatorLoop() {
  std::shared_ptr<TrackingResult> keypoint;
  while (running_.load(std::memory_order_acquire)) {
    if (!keypoint_queue_.try_pop(keypoint)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    // TODO: Replace with actual odometry estimation.
  }
}

}  // namespace omni_slam
