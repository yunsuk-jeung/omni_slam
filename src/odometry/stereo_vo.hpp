#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <tbb/concurrent_queue.h>
#include <opencv2/core.hpp>

#include "odometry/odometry.hpp"

namespace omni_slam {
class TrackingResult;
class StereoOpticalFlow;
class Frame;
class StereoVO : public Odometry {
public:
  StereoVO();
  ~StereoVO() override;

  bool Initialize(const std::string& config_path) override;
  void Run() override;
  void Shutdown() override;
  void OnCameraFrame(const std::vector<cv::Mat>&             images,
                     const std::vector<int>&                 models,
                     const std::vector<std::vector<double>>& intrinsics,
                     const std::vector<std::vector<double>>& distortions,
                     const std::vector<std::vector<int>>&    resolutions);

private:
  void                    OpticalFlowLoop();
  void                    EstimatorLoop();
  static constexpr size_t kCamNum = 2;

  std::thread optical_flow_thread_;
  std::thread estimator_thread_;

  tbb::concurrent_queue<std::array<cv::Mat, kCamNum>>    image_queue_;
  tbb::concurrent_queue<std::shared_ptr<TrackingResult>> keypoint_queue_;
  std::unique_ptr<StereoOpticalFlow>                     optical_flow_;
  std::vector<std::unique_ptr<Frame>>                    frames_;

  std::atomic<bool> running_;
};

}  // namespace omni_slam
