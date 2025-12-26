#include "odometry/stereo_vo.hpp"

#include <chrono>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "config/svo_config.hpp"
#include "optical_flow/stereo_optical_flow.hpp"
#include "utils/logger.hpp"
#include "stereo_vo.hpp"

namespace omni_slam {
StereoVO::StereoVO() {}

StereoVO::~StereoVO() {}

bool StereoVO::Initialize(const std::string& config_path) {
  Logger::Info("Initializing VO Pipeline");
  if (config_path.empty()) {
    Logger::Warn("Empty config path for VO pipeline");
    return false;
  }

  SVOConfig::ParseConfig(config_path);
  Logger::Info("Loaded VO config: {}", config_path.c_str());

  optical_flow_ = std::make_unique<StereoOpticalFlow>(image_queue_, keypoint_queue_);

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
  optical_flow_->Run(running_);
}

void StereoVO::EstimatorLoop() {
  std::shared_ptr<TrackingResult> tracking_result;
  while (running_.load(std::memory_order_acquire)) {
    if (!keypoint_queue_.try_pop(tracking_result)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    const cv::Mat& left  = tracking_result->Image(0);
    const cv::Mat& right = tracking_result->Image(1);
    if (!left.empty()) {
      cv::Mat left_vis;
      if (left.channels() == 1) {
        cv::cvtColor(left, left_vis, cv::COLOR_GRAY2BGR);
      }
      else {
        left.copyTo(left_vis);
      }
      for (const auto& uv : tracking_result->Uvs(0)) {
        cv::circle(left_vis, uv, 2, cv::Scalar(0, 255, 0), -1);
      }
      cv::imshow("SVO Left Features", left_vis);
    }

    if (!right.empty()) {
      cv::Mat right_vis;
      if (right.channels() == 1) {
        cv::cvtColor(right, right_vis, cv::COLOR_GRAY2BGR);
      }
      else {
        right.copyTo(right_vis);
      }
      for (const auto& uv : tracking_result->Uvs(1)) {
        cv::circle(right_vis, uv, 2, cv::Scalar(0, 255, 0), -1);
      }
      cv::imshow("SVO Right Features", right_vis);
    }
    cv::waitKey(1);
  }
}

}  // namespace omni_slam
