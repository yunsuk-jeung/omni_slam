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

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

private:
  std::unique_ptr<Marginalizer> marginalizer_;

  std::map<uint64_t, ImuPreintegrationFactor> imu_factors_by_to_frame_;
  Eigen::Vector3d                             gravity_vector_w_;
};

}  // namespace omni_slam
