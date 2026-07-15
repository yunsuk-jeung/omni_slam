#pragma once

#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>
#include <tbb/concurrent_queue.h>

#include "odometry/odometry.hpp"
#include "odometry/odometry_result.hpp"
#include "utils/types.hpp"

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

  bool setup(const std::string& config_path) override;
  void run() override;
  void shutdown() override;
  void on_camera_frame(int64_t                             timestamp_ns,
                       const std::vector<cv::Mat>&         images,
                       const std::vector<CameraParameter>& camera_parameters);

  bool fetch_result(OdometryResult& out);

 private:
  void  optical_flow_loop();
  void  estimator_loop();
  void  process(std::shared_ptr<Frame>& frame);
  bool  initialize(std::shared_ptr<Frame>& frame);
  void  track(std::shared_ptr<Frame>& frame);
  float update_frame_observations(std::shared_ptr<Frame>& frame);

  OdometryResult build_odometry_result(const std::shared_ptr<Frame>& frame);
  int            initialize_map_points(std::shared_ptr<Frame>& frame);

  void select_marginal_frames(std::set<uint64_t>& marginal_none_frame_ids,
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
