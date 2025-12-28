#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <tbb/concurrent_queue.h>
#include <opencv2/core.hpp>

#include "utils/types.hpp"
#include "odometry/odometry.hpp"

namespace omni_slam {
class TrackingResult;
class OpticalFlow;
class Frame;
class SlidingWindow;
class StereoVO : public Odometry {
public:
  StereoVO();
  ~StereoVO() override;

  bool Initialize(const std::string& config_path) override;
  void Run() override;
  void Shutdown() override;
  void OnCameraFrame(const std::vector<cv::Mat>&         images,
                     const std::vector<CameraParameter>& camera_parameters);

private:
  void                    OpticalFlowLoop();
  void                    EstimatorLoop();
  static constexpr size_t kCamNum = 2;

  std::thread optical_flow_thread_;
  std::thread estimator_thread_;

  tbb::concurrent_queue<std::shared_ptr<Frame>> frame_queue_;
  tbb::concurrent_queue<std::shared_ptr<Frame>> result_queue_;
  std::unique_ptr<OpticalFlow>                  optical_flow_;

  std::unique_ptr<SlidingWindow> sliding_window_;
  std::atomic<bool>              running_;
};

}  // namespace omni_slam
