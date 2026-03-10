#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <set>

#include <Eigen/Core>

#include "odometry/imu_preintegration.hpp"

namespace omni_slam {

class Frame;
class SlidingWindow;
class Marginalizer;

class VIOEstimator {
public:
  struct InertialState {
    int64_t         timestamp_ns = 0;
    Eigen::Vector3d velocity_wb  = Eigen::Vector3d::Zero();
    Eigen::Vector3d bias_acc     = Eigen::Vector3d::Zero();
    Eigen::Vector3d bias_gyr     = Eigen::Vector3d::Zero();
  };

  struct ImuPreintegrationFactor {
    uint64_t          from_frame_id = 0;
    uint64_t          to_frame_id   = 0;
    ImuPreintegration preintegration;
  };

  static void OptimizeSingleFrame(std::shared_ptr<Frame> frame, SlidingWindow* window);

  VIOEstimator();
  ~VIOEstimator();

  void OptimizeWindow(SlidingWindow* window);
  void Marginalize(SlidingWindow* window, std::set<uint64_t> marignal_keyframes);

  void EnsureInertialState(uint64_t frame_id, int64_t timestamp_ns);
  bool GetInertialBias(uint64_t frame_id, Eigen::Vector3d* bias_acc, Eigen::Vector3d* bias_gyr) const;

  void AddImuPreintegration(uint64_t frame_id_i,
                            uint64_t frame_id_j,
                            const ImuPreintegration& preintegration);

  void RemoveFrameStates(const std::set<uint64_t>& frame_ids);
  void ResetStates();

private:
  void OptimizeVisualWindow(SlidingWindow* window);
  void OptimizeImuWindow(SlidingWindow* window);
  void RemoveFrameState(uint64_t frame_id);
  void RemoveImuFactorsWithFrame(uint64_t frame_id);
  void PruneInvalidImuFactors(const std::set<uint64_t>& active_frame_ids);

private:
  std::unique_ptr<Marginalizer> marginalizer_;

  std::map<uint64_t, InertialState>          inertial_states_;
  std::map<uint64_t, ImuPreintegrationFactor> imu_factors_by_to_frame_;
  Eigen::Vector3d                             gravity_vector_w_;
};

}  // namespace omni_slam
