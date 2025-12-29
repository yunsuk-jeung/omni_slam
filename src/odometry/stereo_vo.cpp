#include "odometry/stereo_vo.hpp"

#include <chrono>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "utils/logger.hpp"
#include "config/svo_config.hpp"
#include "database/Frame.hpp"
#include "feature_tracking/optical_flow.hpp"
#include "odometry/sliding_window.hpp"

namespace omni_slam {
StereoVO::StereoVO()
  : frame_queue_{}
  , result_queue_{}
  , optical_flow_{nullptr}
  , running_{false} {
  sliding_window_ = std::make_unique<SlidingWindow>();
}

StereoVO::~StereoVO() {}

bool StereoVO::Initialize(const std::string& config_path) {
  Logger::Info("Initializing VO Pipeline");
  if (config_path.empty()) {
    Logger::Warn("Empty config path for VO pipeline");
    return false;
  }

  SVOConfig::ParseConfig(config_path);
  Logger::Info("Loaded VO config: {}", config_path.c_str());

  sliding_window_->SetMaxSize(SVOConfig::max_window);
  optical_flow_ = std::make_unique<OpticalFlow>(kCamNum, frame_queue_, result_queue_);

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

void StereoVO::OnCameraFrame(const std::vector<cv::Mat>&         images,
                             const std::vector<CameraParameter>& camera_parameters) {
  if (images.empty() || images[0].empty()) {
    Logger::Warn("Received camera frame with empty left image");
    return;
  }

  auto frame = std::make_shared<Frame>(images, camera_parameters);
  frame_queue_.push(frame);
}

void StereoVO::OpticalFlowLoop() {
  optical_flow_->Run(running_);
}

void StereoVO::EstimatorLoop() {
  std::shared_ptr<Frame> frame;
  while (running_.load(std::memory_order_acquire)) {
    if (!result_queue_.try_pop(frame)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    if (!frame || !frame->TrackingResultPtr()) {
      continue;
    }

    sliding_window_->AddFrame(frame);
  }
}

}  // namespace omni_slam
