#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <map>
#include <mutex>

#include <tbb/concurrent_queue.h>
#include <opencv2/core.hpp>

#include "utils/types.hpp"
#include "odometry/odometry.hpp"
#include "odometry/odometry_result.hpp"

namespace omni_slam {
class TrackingResult;
class OpticalFlow;
class Frame;
class MapPoint;
class SlidingWindow;
class StereoVO : public Odometry {
public:
  StereoVO();
  ~StereoVO() override;

  bool Initialize(const std::string& config_path) override;
  void Run() override;
  void Shutdown() override;
  void OnCameraFrame(int64_t                             timestamp_ns,
                     const std::vector<cv::Mat>&         images,
                     const std::vector<CameraParameter>& camera_parameters);

  bool FetchResult(OdometryResult& out);

private:
  void OpticalFlowLoop();
  void EstimatorLoop();

  OdometryResult BuildOdometryResult(const std::shared_ptr<Frame>& frame,
                                     TrackingResult*              tracking_result);
  int InitializeMapPoints(std::shared_ptr<Frame>& frame);

private:
  static constexpr size_t kCamNum = 2;

  std::atomic<bool> running_;
  std::thread       optical_flow_thread_;
  std::thread       estimator_thread_;

  tbb::concurrent_queue<std::shared_ptr<Frame>> frame_queue_;
  tbb::concurrent_queue<std::shared_ptr<Frame>> result_queue_;
  std::unique_ptr<OpticalFlow>                  optical_flow_;

  std::unique_ptr<SlidingWindow> sliding_window_;

  bool                    make_keyframe_;
  int                     new_keyframe_after_;
  std::map<uint64_t, int> created_map_point_nums_;

  std::mutex      result_mutex_;
  bool            has_result_;
  OdometryResult  latest_result_;
};

}  // namespace omni_slam
