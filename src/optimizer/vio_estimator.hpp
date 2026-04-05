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
  static void OptimizeSingleFrame(std::shared_ptr<Frame> frame, SlidingWindow* window);

  VIOEstimator();
  ~VIOEstimator();

  void OptimizeWindow(SlidingWindow*                               window,
                      std::map<uint64_t, InertialState>*           inertial_states,
                      const std::map<uint64_t, ImuPreintegration>* imu_preintegrations);
  void Marginalize(SlidingWindow*     window,
                   std::set<uint64_t> marginal_frame_ids,
                   std::set<uint64_t> marginal_preintegration_ids);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  void ClearPrior();
  void SetPrior(const std::set<uint64_t>& frame_ids,
                const std::set<uint64_t>& preintegration_ids,
                const std::vector<int>&   block_sizes,
                const Eigen::MatrixXd&    A,
                const Eigen::VectorXd&    b,
                const Eigen::VectorXd&    x0);

private:
  std::unique_ptr<MarginalizationPrior> marginalization_prior_;

  std::map<uint64_t, InertialState>     inertial_states_by_frame_;
  std::map<uint64_t, ImuPreintegration> imu_factors_by_to_frame_;
};

}  // namespace omni_slam
