#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <map>
#include <set>
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
class VOEstimator;
class StereoVO : public Odometry {
public:
  enum class Status { Initializing, Tracking };

  StereoVO();
  ~StereoVO() override;

  bool Setup(const std::string& config_path) override;
  void Run() override;
  void Shutdown() override;
  void OnCameraFrame(int64_t                             timestamp_ns,
                     const std::vector<cv::Mat>&         images,
                     const std::vector<CameraParameter>& camera_parameters);

  bool FetchResult(OdometryResult& out);

private:
  void            OpticalFlowLoop();
  void            EstimatorLoop();
  void            Process(std::shared_ptr<Frame>& frame);
  void            ProcessInitialize(std::shared_ptr<Frame>& frame);
  void            ProcessTracking(std::shared_ptr<Frame>& frame);
  TrackingResult* UpdateFrameObservations(std::shared_ptr<Frame>& frame,
                                          size_t&                 connected);
  void            UpdateKeyframeStatus(std::shared_ptr<Frame>& frame, size_t connected);
  void            BuildAndStoreResult(const std::shared_ptr<Frame>& frame,
                                      TrackingResult*               tracking_result);

  OdometryResult BuildOdometryResult(const std::shared_ptr<Frame>& frame,
                                     TrackingResult*               tracking_result);
  int            InitializeMapPoints(std::shared_ptr<Frame>& frame);

  void SelectMarginalFrames(std::set<uint64_t>& marginal_none_frame_ids,
                            std::set<uint64_t>& marginal_keyframe_ids);

private:
  static constexpr size_t kCamNum = 2;
  Status                  status_;

  std::atomic<bool> running_;
  std::thread       optical_flow_thread_;
  std::thread       estimator_thread_;

  tbb::concurrent_queue<std::shared_ptr<Frame>> frame_queue_;
  tbb::concurrent_queue<std::shared_ptr<Frame>> result_queue_;
  std::unique_ptr<OpticalFlow>                  optical_flow_;

  std::unique_ptr<SlidingWindow> sliding_window_;

  std::unique_ptr<VOEstimator> estimator_;

  bool                    make_keyframe_;
  int                     new_keyframe_after_;
  std::map<uint64_t, int> created_map_point_nums_;

  std::mutex     result_mutex_;
  bool           has_result_;
  OdometryResult latest_result_;
};

}  // namespace omni_slam
