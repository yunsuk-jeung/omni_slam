#include <ceres/ceres.h>

#include <cmath>
#include <unordered_map>
#include <vector>

#include "database/MapPoint.hpp"
#include "database/Frame.hpp"
#include "odometry/sliding_window.hpp"
#include "utils/eigen_utils.hpp"
#include "utils/logger.hpp"
#include "optimizer/parameterization.hpp"
#include "optimizer/cost_function.hpp"
#include "optimizer/vo_estimator.hpp"
#include "vo_estimator.hpp"

namespace omni_slam {

static constexpr int kPoseSize    = 6;
static constexpr int kBearingSize = 3;

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
      continue;
    }

    std::shared_ptr<Frame> f0  = window->GetFrame(mp->GetHostFrameCamId().frame_id);
    Sophus::SE3d           Twc = f0->GetTwc(kCamIdx);

    // world point reconstruction from host frame
    const Sophus::SE3d Twc0 = f0->GetTwc(kCamIdx);

    // bearing * inverse depth  → point in host camera frame
    const Eigen::Vector3d p_c0 = mp->GetBearing() / mp->GetInvDist();

    // world point
    const Eigen::Vector3d p_w = Twc0 * p_c0;

    // bearing residual (pose-only)
    ceres::CostFunction* cost = new PoseOnlyBearingCost(p_w, bearing, T_b_c);

    // auto* cost = new ceres::AutoDiffCostFunction<PoseOnlyBearingCostAuto,
    //                                              2,  // residual dim
    //                                              6   // pose dim
    //                                              >(
    //   new PoseOnlyBearingCostAuto(p_w, bearing, T_b_c));

    ceres::LossFunction* loss = new ceres::HuberLoss(0.01);
    problem.AddResidualBlock(cost,
                             loss,  // no robust loss for now
                             box_w_b.data());
  }
  // solver options
  ceres::Solver::Options options;
  options.linear_solver_type           = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations           = 10;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  LogD("{}", summary.FullReport());

  // update pose
  frame->SetTwb(SE3BoxplusManifold::FromParams(box_w_b.data()));
}

void VOEstimator::OptimizeWindow(SlidingWindow* window) {
  if (!window) {
    return;
  }

  const auto& frames     = window->GetFrames();
  const auto& map_points = window->GetMapPoints();

  if (frames.empty()) {
    return;
  }

  ceres::Problem problem;

  // add pose parameters to problem

  std::vector<Eigen::Vector6d>         pose_params;
  std::unordered_map<uint64_t, size_t> frame_id_to_index;
  pose_params.reserve(frames.size());

  for (const auto& [frame_id, frame] : frames) {
    if (!frame) {
      continue;
    }
    frame_id_to_index[frame_id] = pose_params.size();
    pose_params.push_back(SE3BoxplusManifold::ToParams(frame->GetTwb()));
  }

  SE3BoxplusManifold* se3_box_plus_manifold = new SE3BoxplusManifold();
  for (auto& param : pose_params) {
    problem.AddParameterBlock(param.data(), kPoseSize, se3_box_plus_manifold);
  }

  std::unordered_map<uint64_t, size_t> mp_id_to_index;
  std::vector<Eigen::Vector3d>         bearing_params;
  bearing_params.reserve(window->GetMapPointCount());

  for (const auto& [mp_id, mp] : map_points) {
    mp_id_to_index[mp_id] = bearing_params.size();
    bearing_params.push_back(mp->GetBearing());
  }

  BearingTangentManifold* bearing_manifold = new BearingTangentManifold();
  for (auto& bearing : bearing_params) {
    problem.AddParameterBlock(bearing.data(), kBearingSize, bearing_manifold);
  }

  for (const auto& [mp_id, mp] : map_points) {
    if (!mp || mp->GetStatus() < MapPoint::Status::TRACKING) {
      continue;
    }
    const double inv_dist = mp->GetInvDist();
    if (!std::isfinite(inv_dist) || inv_dist <= 0.0) {
      continue;
    }

    const FrameCamId       frame_cam_id0 = mp->GetHostFrameCamId();
    std::shared_ptr<Frame> frame0        = window->GetFrame(frame_cam_id0.frame_id);
    const Sophus::SE3d&    T_b_c0        = frame0->GetTbc(frame_cam_id0.cam_id);
    double* pose_param0 = pose_params[frame_id_to_index[frame_cam_id0.frame_id]].data();

    const auto& observations  = mp->GetObservation();
    double*     bearing_param = bearing_params[mp_id_to_index[mp->GetId()]].data();

    for (const auto& [frame_cam_id1, bearing] : observations) {
      if (frame_cam_id0 == frame_cam_id1) {
        continue;
      }

      std::shared_ptr<Frame> frame1 = window->GetFrame(frame_cam_id1.frame_id);
      if (!frame1) {
        continue;
      }
      auto it1 = frame_id_to_index.find(frame_cam_id1.frame_id);
      if (it1 == frame_id_to_index.end()) {
        continue;
      }
      double*             pose_param1 = pose_params[it1->second].data();
      const Sophus::SE3d& T_b_c1      = frame1->GetTbc(frame_cam_id1.cam_id);

      ceres::LossFunction* loss = new ceres::HuberLoss(0.01);
      if (frame_cam_id0.frame_id == frame_cam_id1.frame_id) {
        // Stereo within the same frame: use a single pose parameter block.
        ceres::CostFunction*
          cost = new ceres::AutoDiffCostFunction<StereoBearingCostAuto, 2, 6, 3>(
            new StereoBearingCostAuto(bearing, T_b_c1, T_b_c0, inv_dist));
        problem.AddResidualBlock(cost, loss, pose_param0, bearing_param);
      }
      else {
        ceres::CostFunction*
          cost = new ceres::AutoDiffCostFunction<BearingStereoCostAuto, 2, 6, 6, 3>(
            new BearingStereoCostAuto(bearing, T_b_c1, T_b_c0, inv_dist));
        problem.AddResidualBlock(cost, loss, pose_param1, pose_param0, bearing_param);
      }
    }
  }

  const auto& frame_ids_set = window->GetFrameIds();
  if (!frame_ids_set.empty()) {
    const uint64_t anchor_id = *frame_ids_set.rbegin();
    auto           anchor_it = frame_id_to_index.find(anchor_id);
    if (anchor_it != frame_id_to_index.end()) {
      problem.SetParameterBlockConstant(pose_params[anchor_it->second].data());
    }
  }

  ceres::Solver::Options options;
  options.linear_solver_type           = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations           = 10;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  LogD("OptimizeWindow: {}", summary.BriefReport());

  for (const auto& [frame_id, frame] : frames) {
    if (!frame) {
      continue;
    }
    const auto it = frame_id_to_index.find(frame_id);
    if (it == frame_id_to_index.end()) {
      continue;
    }
    frame->SetTwb(SE3BoxplusManifold::FromParams(pose_params[it->second].data()));
  }

  for (const auto& [mp_id, idx] : mp_id_to_index) {
    std::shared_ptr<MapPoint> mp = window->GetMapPoint(mp_id);
    if (!mp) {
      continue;
    }
    Eigen::Vector3d b = bearing_params[idx];
    if (b.norm() > 0.0) {
      b.normalize();
    }
    mp->GetBearing() = b;
  }
}

}  // namespace omni_slam
