#include <ceres/ceres.h>

#include "database/MapPoint.hpp"
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
  constexpr size_t kCamIdx     = 0;
  auto&            mp_id_to_uv = frame->GetObservation(kCamIdx);
  auto&            T_w_b       = frame->GetTwb();
  auto&            T_b_c       = frame->GetTbc(kCamIdx);

  Eigen::Vector6d box_w_b = SE3BoxplusManifold::ToParams(frame->GetTwb());

  ceres::Problem problem;
  problem.AddParameterBlock(box_w_b.data(), kPoseSize);

  auto& mp_id_to_bearing = frame->GetObservation(kCamIdx);

  for (auto& [mp_id, bearing] : mp_id_to_bearing) {
    std::shared_ptr<MapPoint> mp = window->GetMapPoint(mp_id);

    if (!mp) {
      LogE("Missing map point id {}", mp_id);
      continue;
    }

    std::shared_ptr<Frame> f0  = window->GetFrame(mp->GetHostFrameId());
    Sophus::SE3d           Twc = f0->GetTwc(kCamIdx);

    // world point reconstruction from host frame
    const Sophus::SE3d Twc0 = f0->GetTwc(kCamIdx);

    // bearing * inverse depth  → point in host camera frame
    const Eigen::Vector3d p_c0 = mp->GetBearing() / mp->GetInvDist();

    // world point
    const Eigen::Vector3d p_w = Twc0 * p_c0;

    // bearing residual (pose-only)
    ceres::CostFunction* cost = new PoseOnlyBearingCost(bearing, p_w, T_b_c);

    problem.AddResidualBlock(cost,
                             nullptr,  // no robust loss for now
                             box_w_b.data());
  }
  // solver options
  ceres::Solver::Options options;
  options.linear_solver_type           = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations           = 2;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // update pose
  frame->SetTwb(SE3BoxplusManifold::FromParams(box_w_b.data()));
}

}  // namespace omni_slam
