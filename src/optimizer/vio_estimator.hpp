#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <vector>

#include <Eigen/Core>

#include "odometry/imu_preintegration.hpp"

namespace omni_slam {

class Frame;
class SlidingWindow;
struct MarginalizationPrior;

class VIOEstimator {
 public:
  static void OptimizeSingleFrame(std::shared_ptr<Frame> frame,
                                  SlidingWindow*         window);

  VIOEstimator();
  ~VIOEstimator();

  void OptimizeWindow(
    SlidingWindow*                               window,
    std::map<uint64_t, InertialState>&           inertial_states,
    const std::map<uint64_t, ImuPreintegration>& imu_preintegrations);
  void Marginalize(
    SlidingWindow*                               window,
    std::set<uint64_t>                           marginal_frame_ids,
    std::set<uint64_t>                           marginal_inertial_state_ids,
    const std::map<uint64_t, InertialState>&     inertial_states,
    const std::map<uint64_t, ImuPreintegration>& imu_preintegrations);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  void ClearPrior();

 private:
  std::unique_ptr<MarginalizationPrior> marginalization_prior_;
};

}  // namespace omni_slam
