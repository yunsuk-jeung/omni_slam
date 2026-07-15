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

#include "odometry/imu_preintegration.hpp"
#include "omni_slam/odometry.hpp"
#include "omni_slam/odometry_result.hpp"
#include "omni_slam/types.hpp"
#include "utils/eigen_utils.hpp"

namespace omni_slam {
class TrackingResult;
class OpticalFlow;
class Frame;
class MapPoint;
class SlidingWindow;
class VIOEstimator;
class StereoVIO : public Odometry {
 public:
  enum class Status { Initializing, Tracking };

  StereoVIO();
  ~StereoVIO() override;

  bool setup(const std::string& config_path) override;
  void run() override;
  void shutdown() override;
  void on_camera_frame(int64_t                             timestamp_ns,
                       const std::vector<cv::Mat>&         images,
                       const std::vector<CameraParameter>& camera_parameters);
  void on_imu_data(const ImuData& imu_data);

  bool fetch_result(OdometryResult& out);

 private:
  void optical_flow_loop();
  void estimator_loop();
  void process(std::shared_ptr<Frame>&     frame,
               const std::vector<ImuData>& imu_data);
  bool initialize(std::shared_ptr<Frame>&     frame,
                  const std::vector<ImuData>& imu_data);
  void track(std::shared_ptr<Frame>&     frame,
             const std::vector<ImuData>& imu_data);
  void pop_imu_data_until(int64_t timestamp_ns, std::vector<ImuData>& imu_data);
  float update_frame_observations(std::shared_ptr<Frame>& frame);

  OdometryResult build_odometry_result(const std::shared_ptr<Frame>& frame);
  int            initialize_map_points(std::shared_ptr<Frame>& frame);

  void select_marginal_frames(std::set<uint64_t>& marginal_frame_ids,
                              std::set<uint64_t>& marginal_inertial_state_ids);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

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
  std::map<uint64_t, InertialState>
    inertial_states_;  // v_wb, bias_acc, bias_gyr
  std::map<uint64_t, ImuPreintegration> imu_preintegrations_;

  std::unique_ptr<VIOEstimator> estimator_;

  bool                    make_keyframe_;
  int                     new_keyframe_after_;
  std::map<uint64_t, int> created_map_point_nums_;

  std::mutex     result_mutex_;
  bool           has_result_;
  OdometryResult latest_result_;

  tbb::concurrent_queue<ImuData> imu_queue_;
  std::atomic<size_t>            imu_queue_size_;
  std::vector<ImuData>           imu_data_buffer_;
  bool                           has_pending_imu_;
  ImuData                        pending_imu_;
  bool                           has_last_frame_imu_;
  ImuData                        last_frame_imu_;
  ImuPreintegration::Parameters  imu_parameters;
};

}  // namespace omni_slam
