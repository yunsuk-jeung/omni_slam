#pragma once

#include <array>
#include <atomic>
#include <string>
#include <thread>
#include <tbb/concurrent_queue.h>
#include <opencv2/core.hpp>
#include "optical_flow/stereo_optical_flow.hpp"
#include "odometry/odometry.hpp"

namespace omni_slam {

class StereoVO : public Odometry {
public:
  StereoVO() = default;

  bool Initialize(const std::string& config_path) override;
  void Run() override;
  void Shutdown() override;
  void OnCameraFrame(const std::array<cv::Mat, 2>& images);

private:
  void OpticalFlowLoop();
  void EstimatorLoop();

  std::thread optical_flow_thread_;
  std::thread estimator_thread_;

  tbb::concurrent_queue<std::array<cv::Mat, 2>>          image_queue_;
  tbb::concurrent_queue<std::shared_ptr<TrackingResult>> keypoint_queue_;
  StereoOpticalFlow optical_flow_{image_queue_, keypoint_queue_};
  std::atomic<bool> running_{false};
};

}  // namespace omni_slam
