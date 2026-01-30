#include <ceres/ceres.h>

#include "database/Frame.hpp"
#include "odometry/sliding_window.hpp"
#include "utils/eigen_utils.hpp"
#include "utils/logger.hpp"
#include "optimizer/parameterization.hpp"
#include "optimizer/cost_function.hpp"
#include "optimizer/vo_estimator.hpp"

namespace omni_slam {

static constexpr int kPoseSize = 6;

void VOEstimator::OptimizeSingleFrame(std::shared_ptr<Frame> frame,
                                      SlidingWindow*         window) {
  auto&           mp_id_to_uv = frame->GetObservation(0u);
  auto&           T_w_b       = frame->GetTwb();
  Eigen::Vector6d box_w_b     = SE3BoxplusManifold::ToParams(frame->GetTwb());

  ceres::Problem problem;

  problem.AddParameterBlock(box_w_b.data(), kPoseSize);

  // auto cost = PoseOnlyReprojectionCost(Eigen::Vector2d::Zero());
}

}  // namespace omni_slam
