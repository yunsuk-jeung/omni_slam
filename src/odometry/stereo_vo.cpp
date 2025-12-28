#include "odometry/stereo_vo.hpp"

#include <chrono>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "config/svo_config.hpp"
#include "database/Frame.hpp"
#include "optical_flow/optical_flow.hpp"
#include "utils/logger.hpp"

namespace omni_slam {
StereoVO::StereoVO()
  : frame_queue_{}
  , result_queue_{}
  , optical_flow_{nullptr}
  , running_{false} {}

StereoVO::~StereoVO() {}

bool StereoVO::Initialize(const std::string& config_path) {
  Logger::Info("Initializing VO Pipeline");
  if (config_path.empty()) {
    Logger::Warn("Empty config path for VO pipeline");
    return false;
  }

  SVOConfig::ParseConfig(config_path);
  Logger::Info("Loaded VO config: {}", config_path.c_str());

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

  (void)camera_parameters;

  // camera_models_      = models;
  // camera_intrinsics_  = intrinsics;
  // camera_distortions_ = distortions;
  // camera_resolutions_ = resolutions;
  // camera_T_bc_        = T_bc;
  // if (camera_models_.empty()) {
  //   camera_models_.assign(images.size(),
  //   static_cast<int>(CameraModel::PINHOLE_RAD_TAN));
  // }
  // frames_.clear();
  // const size_t cam_count = camera_models_.size();
  // frames_.reserve(cam_count);
  // for (size_t i = 0; i < cam_count; ++i) {
  //   const Eigen::Matrix4d T = (i < camera_T_bc_.size()) ? camera_T_bc_[i]
  //                                                       : Eigen::Matrix4d::Identity();
  //   frames_.push_back(
  //     std::make_unique<Frame>(static_cast<CameraModel>(models[i]),
  //                             i < camera_intrinsics_.size() ? camera_intrinsics_[i]
  //                                                           : std::vector<double>{},
  //                             i < camera_distortions_.size() ? camera_distortions_[i]
  //                                                            : std::vector<double>{},
  //                             i < camera_resolutions_.size() ? camera_resolutions_[i]
  //                                                            : std::vector<int>{},
  //                             T));
  // }

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

    const auto* tracking_result = frame->TrackingResultPtr();
    const cv::Mat& left          = frame->Image(0);
    const cv::Mat& right         = frame->Image(1);
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
