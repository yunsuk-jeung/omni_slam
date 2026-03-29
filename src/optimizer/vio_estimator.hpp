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
  static void OptimizeSingleFrame(std::shared_ptr<Frame> frame, SlidingWindow* window);

  VIOEstimator();
  ~VIOEstimator();

  void OptimizeWindow(SlidingWindow*                               window,
                      std::map<uint64_t, InertialState>*           inertial_states,
                      const std::map<uint64_t, ImuPreintegration>* imu_preintegrations);
  void Marginalize(SlidingWindow* window, std::set<uint64_t> marignal_keyframes);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

private:
  std::unique_ptr<Marginalizer> marginalizer_;

  std::map<uint64_t, InertialState>     inertial_states_by_frame_;
  std::map<uint64_t, ImuPreintegration> imu_factors_by_to_frame_;
};

}  // namespace omni_slam
