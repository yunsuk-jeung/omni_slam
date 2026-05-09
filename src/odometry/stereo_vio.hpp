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

#include "utils/eigen_utils.hpp"
#include "utils/types.hpp"
#include "odometry/odometry.hpp"
#include "odometry/odometry_result.hpp"
#include "odometry/imu_preintegration.hpp"

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

  bool Setup(const std::string& config_path) override;
  void Run() override;
  void Shutdown() override;
  void OnCameraFrame(int64_t                             timestamp_ns,
                     const std::vector<cv::Mat>&         images,
                     const std::vector<CameraParameter>& camera_parameters);
  void OnImuData(const ImuData& imu_data);

  bool FetchResult(OdometryResult& out);

private:
  void  OpticalFlowLoop();
  void  EstimatorLoop();
  void  Process(std::shared_ptr<Frame>& frame, const std::vector<ImuData>& imu_data);
  bool  Initialize(std::shared_ptr<Frame>& frame, const std::vector<ImuData>& imu_data);
  void  Track(std::shared_ptr<Frame>& frame, const std::vector<ImuData>& imu_data);
  void  PopImuDataUntil(int64_t timestamp_ns, std::vector<ImuData>& imu_data);
  float UpdateFrameObservations(std::shared_ptr<Frame>& frame);

  OdometryResult BuildOdometryResult(const std::shared_ptr<Frame>& frame);
  int            InitializeMapPoints(std::shared_ptr<Frame>& frame);

  void SelectMarginalFrames(std::set<uint64_t>& marginal_frame_ids,
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

  std::unique_ptr<SlidingWindow>        sliding_window_;
  std::map<uint64_t, InertialState>     inertial_states_;  // v_wb, bias_acc, bias_gyr
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
