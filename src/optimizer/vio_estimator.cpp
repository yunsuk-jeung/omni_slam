#include "optimizer/vio_estimator.hpp"

#include <ceres/ceres.h>

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <vector>

#include "config/svio_config.hpp"
#include "config/svo_config.hpp"
#include "database/Frame.hpp"
#include "database/MapPoint.hpp"
#include "odometry/sliding_window.hpp"
#include "optimizer/cost_function.hpp"
#include "optimizer/marginalizer.hpp"
#include "optimizer/parameterization.hpp"
#include "utils/ceres_utils.hpp"
#include "utils/eigen_utils.hpp"
#include "utils/logger.hpp"
#include "utils/omni_assert.hpp"
#include "utils/timer.hpp"

namespace omni_slam {
namespace {

static std::string JoinIds(const std::set<uint64_t>& ids) {
  std::string s;
  for (auto id : ids) {
    if (!s.empty())
      s += ",";
    s += std::to_string(id);
  }
  return s;
}

static constexpr int      kPoseSize                   = 6;
static constexpr int      kBiasSize                   = 3;
static constexpr int      kBearingSize                = 3;
static constexpr int      kInertialStateDim           = 3;
static constexpr int      kInertialStateSize          = 3 * kInertialStateDim;
static constexpr uint64_t kMarginalizerInitialFrameId = 0;

static void AddMarginalizationPriorIfAvailable(
  ceres::Problem&                             problem,
  const MarginalizationPrior&                 prior,
  const std::unordered_map<uint64_t, size_t>& frame_id_to_index,
  const std::unordered_map<uint64_t, size_t>& inertial_id_to_index,
  std::vector<Eigen::Vector6d>&               pose_params,
  std::vector<Eigen::Vector3d>&               velocity_params,
  std::vector<Eigen::Vector3d>&               bias_acc_params,
  std::vector<Eigen::Vector3d>&               bias_gyr_params) {
  if (prior.block_sizes_.empty() || prior.J_.rows() == 0 || prior.r_.size() == 0
      || prior.x0_.size() == 0) {
    LogW("Prior skipped: empty data (blocks={}, J_rows={}, r={}, x0={})",
         prior.block_sizes_.size(),
         prior.J_.rows(),
         prior.r_.size(),
         prior.x0_.size());
    return;
  }

  std::vector<double*> prior_pose_blocks;
  const auto&          frame_ids          = prior.frame_ids_;
  const auto&          preintegration_ids = prior.preintegration_ids_;
  const auto&          block_sizes        = prior.block_sizes_;

  if (block_sizes.size() < frame_ids.size()) {
    LogW("Prior skipped: block_sizes({}) < frame_ids({})",
         block_sizes.size(),
         frame_ids.size());
    return;
  }

  const size_t extra_block_count        = block_sizes.size() - frame_ids.size();
  size_t       per_inertial_block_count = 0;
  if (!preintegration_ids.empty()) {
    if (extra_block_count % preintegration_ids.size() != 0) {
      LogW("Prior skipped: extra_block_count({}) not divisible by preint_ids({})",
           extra_block_count,
           preintegration_ids.size());
      return;
    }
    per_inertial_block_count = extra_block_count / preintegration_ids.size();
    if (per_inertial_block_count < 2 || per_inertial_block_count > 3) {
      LogW("Prior skipped: per_inertial_block_count={} out of [2,3]",
           per_inertial_block_count);
      return;
    }
  }
  else if (extra_block_count != 0) {
    LogW("Prior skipped: extra_block_count={} but no preintegration_ids",
         extra_block_count);
    return;
  }

  prior_pose_blocks.reserve(frame_ids.size()
                            + preintegration_ids.size() * per_inertial_block_count);

  for (const uint64_t frame_id : frame_ids) {
    auto it = frame_id_to_index.find(frame_id);
    if (it == frame_id_to_index.end()) {
      LogW("Prior skipped: frame_id {} not found in window", frame_id);
      return;
    }
    const size_t idx = it->second;
    if (idx >= pose_params.size()) {
      LogW("Prior skipped: frame_id {} idx {} >= pose_params size {}",
           frame_id,
           idx,
           pose_params.size());
      return;
    }
    prior_pose_blocks.push_back(pose_params[idx].data());
  }

  for (const uint64_t frame_id : preintegration_ids) {
    auto it = inertial_id_to_index.find(frame_id);
    if (it == inertial_id_to_index.end()) {
      LogW("Prior skipped: preint id {} not found in inertial index", frame_id);
      return;
    }
    const size_t idx = it->second;
    if (idx >= velocity_params.size() || idx >= bias_acc_params.size()
        || idx >= bias_gyr_params.size()) {
      LogW("Prior skipped: preint id {} idx {} out of inertial param bounds",
           frame_id,
           idx);
      return;
    }
    if (per_inertial_block_count == 3) {
      prior_pose_blocks.push_back(velocity_params[idx].data());
    }
    prior_pose_blocks.push_back(bias_acc_params[idx].data());
    prior_pose_blocks.push_back(bias_gyr_params[idx].data());
  }

  LogI("Prior added: frame_ids=[{}], preint_ids=[{}], blocks={}, residuals={}",
       JoinIds(frame_ids),
       JoinIds(preintegration_ids),
       block_sizes.size(),
       prior.r_.size());

  ceres::CostFunction* prior_cost = new MarginalizationCost(prior);
  problem.AddResidualBlock(prior_cost, nullptr, prior_pose_blocks);
}

}  // namespace

void VIOEstimator::OptimizeSingleFrame(std::shared_ptr<Frame> frame,
                                       SlidingWindow*         window) {
  constexpr size_t kCamIdx = 0;
  if (!frame || !window) {
    return;
  }

  Eigen::Vector6d box_w_b = SE3BoxplusManifold::ToParams(frame->GetTwb());

  ceres::Problem problem;
  auto*          se3_box_plus_manifold = new SE3BoxplusManifold();
  problem.AddParameterBlock(box_w_b.data(), kPoseSize, se3_box_plus_manifold);

  auto& mp_id_to_bearing = frame->GetObservation(kCamIdx);

  for (auto& [mp_id, bearing] : mp_id_to_bearing) {
    std::shared_ptr<MapPoint> mp = window->GetMapPoint(mp_id);
    if (!mp || mp->GetStatus() < MapPoint::Status::TRACKING) {
      continue;
    }

    std::shared_ptr<Frame> host_frame = window->GetFrame(
      mp->GetHostFrameCamId().frame_id);
    if (!host_frame) {
      continue;
    }

    const Sophus::SE3d    Twc0  = host_frame->GetTwc(kCamIdx);
    const Eigen::Vector3d p_c0  = mp->GetBearing() / mp->GetInvDist();
    const Eigen::Vector3d p_w   = Twc0 * p_c0;
    const Sophus::SE3d&   T_b_c = frame->GetTbc(kCamIdx);

    ceres::CostFunction* cost = new PoseOnlyBearingCost(p_w,
                                                        bearing,
                                                        T_b_c,
                                                        SVOConfig::bearing_cost_scale);
    ceres::LossFunction* loss = new ceres::HuberLoss(SVOConfig::bearing_huber_const);
    problem.AddResidualBlock(cost, loss, box_w_b.data());
  }

  ceres::Solver::Options options;
  options.linear_solver_type           = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;
  options.num_threads                  = 2;
  options.max_num_iterations           = SVOConfig::single_frame_max_iterations;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  frame->SetTwb(SE3BoxplusManifold::FromParams(box_w_b.data()));
}

VIOEstimator::VIOEstimator()
  : marginalization_prior_(std::make_unique<MarginalizationPrior>()) {
  marginalization_prior_->frame_ids_.insert(kMarginalizerInitialFrameId);
  // marginalization_prior_->preintegration_ids_.insert(kMarginalizerInitialFrameId);

  marginalization_prior_->x0_ = Eigen::VectorXd::Zero(kPoseSize);
  marginalization_prior_->J_  = Eigen::MatrixXd::Zero(marginalization_prior_->x0_.size(),
                                                     marginalization_prior_->x0_.size());
  marginalization_prior_->J_
    .topLeftCorner(3u, 3u) = SVIOConfig::marginalizer_initial_prior_weight
                             * Eigen::MatrixXd::Identity(3u, 3u);
  marginalization_prior_->J_(5, 5) = SVIOConfig::marginalizer_initial_prior_weight;

  marginalization_prior_->J_
    .topLeftCorner(6u, 6u) = SVIOConfig::marginalizer_initial_prior_weight
                             * Eigen::MatrixXd::Identity(6u, 6u);

  // marginalization_prior_->J_
  //   .block(kPoseSize,
  //          kPoseSize,
  //          kBiasSize,
  //          kBiasSize) = SVIOConfig::marginalizer_initial_bias_weight
  //                       * Eigen::MatrixXd::Identity(kBiasSize, kBiasSize);

  // marginalization_prior_->J_
  //   .block(kPoseSize + kBiasSize,
  //          kPoseSize + kBiasSize,
  //          kBiasSize,
  //          kBiasSize) = SVIOConfig::marginalizer_initial_bias_weight
  //                       * Eigen::MatrixXd::Identity(kBiasSize, kBiasSize);
  marginalization_prior_->block_sizes_ = {kPoseSize};
  marginalization_prior_->r_ = Eigen::VectorXd::Zero(marginalization_prior_->x0_.size());
}

VIOEstimator::~VIOEstimator() = default;

void VIOEstimator::ClearPrior() {
  LogE("something is wrong");
  if (!marginalization_prior_) {
    return;
  }
  marginalization_prior_->J_.resize(0, 0);
  marginalization_prior_->r_.resize(0);
  marginalization_prior_->x0_.resize(0);
  marginalization_prior_->frame_ids_.clear();
  marginalization_prior_->preintegration_ids_.clear();
  marginalization_prior_->block_sizes_.clear();
}

void VIOEstimator::OptimizeWindow(
  SlidingWindow*                               window,
  std::map<uint64_t, InertialState>&           inertial_states,
  const std::map<uint64_t, ImuPreintegration>& imu_preintegrations) {
  if (!window) {
    return;
  }

  const auto& frames     = window->GetFrames();
  const auto& map_points = window->GetMapPoints();

  if (frames.size() < 3) {
    return;
  }

  ceres::Problem problem;

  std::vector<Eigen::Vector6d>         pose_params;
  std::unordered_map<uint64_t, size_t> frame_id_to_index;
  pose_params.reserve(frames.size());

  for (const auto& [frame_id, frame] : frames) {
    frame_id_to_index[frame_id] = pose_params.size();
    pose_params.push_back(SE3BoxplusManifold::ToParams(frame->GetTwb()));
  }

  auto* se3_box_plus_manifold = new SE3BoxplusManifold();
  for (auto& param : pose_params) {
    problem.AddParameterBlock(param.data(), kPoseSize, se3_box_plus_manifold);
  }

  std::unordered_map<uint64_t, size_t> inertial_id_to_index;
  std::vector<Eigen::Vector3d>         velocity_params;
  std::vector<Eigen::Vector3d>         bias_acc_params;
  std::vector<Eigen::Vector3d>         bias_gyr_params;

  for (const auto& [frame_id, _] : frames) {
    const auto state_it = inertial_states.find(frame_id);
    if (state_it == inertial_states.end()) {
      continue;
    }
    inertial_id_to_index[frame_id] = velocity_params.size();
    velocity_params.push_back(state_it->second.v_w_b);
    bias_acc_params.push_back(state_it->second.bias_acc);
    bias_gyr_params.push_back(state_it->second.bias_gyr);
  }

  for (size_t i = 0; i < velocity_params.size(); ++i) {
    problem.AddParameterBlock(velocity_params[i].data(), kInertialStateDim);
    problem.AddParameterBlock(bias_acc_params[i].data(), kInertialStateDim);
    problem.AddParameterBlock(bias_gyr_params[i].data(), kInertialStateDim);
  }

  for (const auto& [_, preintegration] : imu_preintegrations) {
    if (preintegration.GetDeltaTimeSec() <= 0.0) {
      continue;
    }
    const uint64_t from_id = preintegration.GetFromFrameId();
    const uint64_t to_id   = preintegration.GetToFrameId();

    const auto from_frame_it    = frame_id_to_index.find(from_id);
    const auto to_frame_it      = frame_id_to_index.find(to_id);
    const auto from_inertial_it = inertial_id_to_index.find(from_id);
    const auto to_inertial_it   = inertial_id_to_index.find(to_id);
    if (from_frame_it == frame_id_to_index.end() || to_frame_it == frame_id_to_index.end()
        || from_inertial_it == inertial_id_to_index.end()
        || to_inertial_it == inertial_id_to_index.end()) {
      continue;
    }

    const size_t from_pose_idx     = from_frame_it->second;
    const size_t to_pose_idx       = to_frame_it->second;
    const size_t from_inertial_idx = from_inertial_it->second;
    const size_t to_inertial_idx   = to_inertial_it->second;

    ceres::CostFunction* imu_cost = new ImuPreintegrationCost(preintegration,
                                                              SVIOConfig::g_w);
    problem.AddResidualBlock(imu_cost,
                             nullptr,
                             pose_params[from_pose_idx].data(),
                             pose_params[to_pose_idx].data(),
                             velocity_params[from_inertial_idx].data(),
                             velocity_params[to_inertial_idx].data(),
                             bias_acc_params[from_inertial_idx].data(),
                             bias_gyr_params[from_inertial_idx].data(),
                             bias_acc_params[to_inertial_idx].data(),
                             bias_gyr_params[to_inertial_idx].data());
  }

  std::unordered_map<uint64_t, size_t> mp_id_to_index;
  std::vector<Eigen::Vector3d>         bearing_params;
  std::vector<double>                  inv_dist_params;
  bearing_params.reserve(window->GetMapPointCount());
  inv_dist_params.reserve(window->GetMapPointCount());

  for (const auto& [mp_id, mp] : map_points) {
    mp_id_to_index[mp_id] = bearing_params.size();
    bearing_params.push_back(mp->GetBearing());
    double inv_dist = mp->GetInvDist();
    if (!std::isfinite(inv_dist) || inv_dist <= SVOConfig::inv_dist_min_value) {
      inv_dist = SVOConfig::inv_dist_initial_value;
    }
    inv_dist_params.push_back(inv_dist);
  }

  auto* bearing_manifold = new BearingTangentManifold();
  for (auto& bearing : bearing_params) {
    problem.AddParameterBlock(bearing.data(), kBearingSize, bearing_manifold);
  }

  for (auto& inv_dist : inv_dist_params) {
    problem.AddParameterBlock(&inv_dist, 1);
    problem.SetParameterLowerBound(&inv_dist, 0, SVOConfig::inv_dist_min_value);
  }
  {
    std::string window_ids_str;
    for (const auto& fid : window->GetFrameIds()) {
      window_ids_str += std::to_string(fid) + ",";
    }
    LogI("OptimizeWindow: window=[{}], prior_frames=[{}], prior_preints=[{}]",
         window_ids_str,
         JoinIds(marginalization_prior_->frame_ids_),
         JoinIds(marginalization_prior_->preintegration_ids_));
  }
  AddMarginalizationPriorIfAvailable(problem,
                                     *marginalization_prior_,
                                     frame_id_to_index,
                                     inertial_id_to_index,
                                     pose_params,
                                     velocity_params,
                                     bias_acc_params,
                                     bias_gyr_params);

  for (const auto& [mp_id, mp] : map_points) {
    if (mp->GetStatus() < MapPoint::Status::TRACKING) {
      continue;
    }
    const double inv_dist = mp->GetInvDist();
    if (!std::isfinite(inv_dist) || inv_dist <= 0.0) {
      continue;
    }

    const FrameCamId frame_cam_id0 = mp->GetHostFrameCamId();
    auto             frame0        = window->GetFrame(frame_cam_id0.frame_id);
    if (!frame0) {
      continue;
    }

    const Sophus::SE3d& T_b_c0 = frame0->GetTbc(frame_cam_id0.cam_id);
    double* pose_param0 = pose_params[frame_id_to_index[frame_cam_id0.frame_id]].data();

    const auto& observations   = mp->GetObservation();
    double*     bearing_param  = bearing_params[mp_id_to_index[mp->GetId()]].data();
    double*     inv_dist_param = &inv_dist_params[mp_id_to_index[mp->GetId()]];

    const auto host_obs_it = observations.find(frame_cam_id0);
    if (host_obs_it != observations.end()) {
      ceres::CostFunction*
        host_bearing_prior_cost = new BearingPriorCost(host_obs_it->second,
                                                       SVOConfig::bearing_cost_scale);
      problem.AddResidualBlock(host_bearing_prior_cost, nullptr, bearing_param);
    }

    for (const auto& [frame_cam_id1, bearing] : observations) {
      if (frame_cam_id0 == frame_cam_id1) {
        continue;
      }

      auto frame1 = window->GetFrame(frame_cam_id1.frame_id);
      if (!frame1) {
        continue;
      }

      double* pose_param1 = pose_params[frame_id_to_index[frame_cam_id1.frame_id]].data();
      const Sophus::SE3d& T_b_c1 = frame1->GetTbc(frame_cam_id1.cam_id);

      ceres::LossFunction* loss = new ceres::HuberLoss(SVOConfig::bearing_huber_const);
      if (frame_cam_id0.frame_id == frame_cam_id1.frame_id) {
        ceres::CostFunction* cost = new BearingStereoCost(bearing,
                                                          T_b_c1,
                                                          T_b_c0,
                                                          SVOConfig::bearing_cost_scale);
        problem.AddResidualBlock(cost, loss, bearing_param, inv_dist_param);
      }
      else {
        ceres::CostFunction* cost = new BearingCost(bearing,
                                                    T_b_c1,
                                                    T_b_c0,
                                                    SVOConfig::bearing_cost_scale);
        problem.AddResidualBlock(cost,
                                 loss,
                                 pose_param1,
                                 pose_param0,
                                 bearing_param,
                                 inv_dist_param);
      }
    }
  }

  ceres::Solver::Options options;
  options.linear_solver_type           = ceres::SPARSE_NORMAL_CHOLESKY;
  options.num_threads                  = SVOConfig::window_num_threads;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations           = SVOConfig::window_max_iterations;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  for (const auto& [frame_id, frame] : frames) {
    auto it = frame_id_to_index.find(frame_id);
    frame->SetTwb(SE3BoxplusManifold::FromParams(pose_params[it->second].data()));
  }

  for (const auto& [frame_id, idx] : inertial_id_to_index) {
    auto state_it = inertial_states.find(frame_id);
    if (state_it == inertial_states.end()) {
      continue;
    }
    state_it->second.v_w_b    = velocity_params[idx];
    state_it->second.bias_acc = bias_acc_params[idx];
    state_it->second.bias_gyr = bias_gyr_params[idx];
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
    mp->SetInvDist(std::max(inv_dist_params[idx], SVOConfig::inv_dist_min_value));
  }
}

void VIOEstimator::Marginalize(
  SlidingWindow*                               window,
  std::set<uint64_t>                           marginal_frame_ids,
  std::set<uint64_t>                           marginal_inertial_state_ids,
  const std::map<uint64_t, InertialState>&     inertial_states,
  const std::map<uint64_t, ImuPreintegration>& imu_preintegrations) {
  if (!window || (marginal_frame_ids.empty() && marginal_inertial_state_ids.empty())) {
    return;
  }

  LogI("=== Marginalize START ===");
  LogI("  marginal_frame_ids: [{}]", JoinIds(marginal_frame_ids));
  LogI("  marginal_inertial_ids: [{}]", JoinIds(marginal_inertial_state_ids));
  LogI("  prior frame_ids: [{}]", JoinIds(marginalization_prior_->frame_ids_));
  LogI("  prior preint_ids: [{}]", JoinIds(marginalization_prior_->preintegration_ids_));
  {
    std::string window_ids_str;
    for (const auto& fid : window->GetFrameIds()) {
      window_ids_str += std::to_string(fid) + ",";
    }
    LogI("  window frame_ids: [{}]", window_ids_str);
  }

  // Contract check: preintegration map key must be from_id.
  for (const auto& [from_id, preintegration] : imu_preintegrations) {
    OMNI_ASSERT_MESSAGE(preintegration.GetFromFrameId() == from_id,
                        "preintegration key must be from_id");
  }

  const auto& prev_frame_ids = marginalization_prior_->frame_ids_;

  std::set<uint64_t> remain_frame_ids;
  for (const uint64_t f_id : prev_frame_ids) {
    if (marginal_frame_ids.count(f_id) == 0) {
      remain_frame_ids.insert(f_id);
    }
  }

  auto& mp_id_to_mp = window->GetMapPoints();

  std::vector<std::shared_ptr<MapPoint>> marginal_map_points;
  marginal_map_points.reserve(mp_id_to_mp.size());
  for (auto& [_, mp] : mp_id_to_mp) {
    if (marginal_frame_ids.count(mp->GetHostFrameCamId().frame_id) > 0) {
      marginal_map_points.push_back(mp);
    }
  }

  for (const auto& mp : marginal_map_points) {
    auto& frame_cam_id_to_bearing = mp->GetObservation();
    for (const auto& [frame_cam_id, _] : frame_cam_id_to_bearing) {
      if (marginal_frame_ids.count(frame_cam_id.frame_id) > 0) {
        continue;
      }
      if (remain_frame_ids.count(frame_cam_id.frame_id) == 0) {
        remain_frame_ids.insert(frame_cam_id.frame_id);
      }
    }
  }

  // NOTE: Inertial states and preintegration factors are excluded from
  // marginalization to avoid degrading performance.

  ceres::Problem problem;

  std::vector<Eigen::Vector6d>         pose_params;
  std::unordered_map<uint64_t, size_t> frame_id_to_index;
  auto*                                box_plus_manifold = new SE3BoxplusManifold();

  pose_params.reserve(marginal_frame_ids.size() + remain_frame_ids.size());
  for (const auto& frame_id : marginal_frame_ids) {
    auto frame = window->GetFrame(frame_id);
    if (!frame) {
      continue;
    }
    frame_id_to_index[frame_id] = pose_params.size();
    pose_params.push_back(SE3BoxplusManifold::ToParams(frame->GetTwb()));
    problem.AddParameterBlock(pose_params.back().data(), kPoseSize, box_plus_manifold);
  }

  for (const auto& frame_id : remain_frame_ids) {
    auto frame = window->GetFrame(frame_id);
    if (!frame) {
      continue;
    }
    frame_id_to_index[frame_id] = pose_params.size();
    pose_params.push_back(SE3BoxplusManifold::ToParams(frame->GetTwb()));
  }

  LogI("  remain_frame_ids: [{}]", JoinIds(remain_frame_ids));

  // Empty inertial maps - needed for AddMarginalizationPriorIfAvailable signature
  std::unordered_map<uint64_t, size_t> inertial_id_to_index;
  std::vector<Eigen::Vector3d>         velocity_params;
  std::vector<Eigen::Vector3d>         bias_acc_params;
  std::vector<Eigen::Vector3d>         bias_gyr_params;

  std::vector<Eigen::Vector3d> bearing_params;
  std::vector<double>          inv_dist_params;
  bearing_params.reserve(window->GetMapPointCount());
  inv_dist_params.reserve(window->GetMapPointCount());

  for (const auto& map_point : marginal_map_points) {
    bearing_params.push_back(map_point->GetBearing());
    double inv_dist = map_point->GetInvDist();
    if (!std::isfinite(inv_dist) || inv_dist <= SVOConfig::inv_dist_min_value) {
      inv_dist = SVOConfig::inv_dist_initial_value;
    }
    inv_dist_params.push_back(inv_dist);
  }

  auto* bearing_manifold = new BearingTangentManifold();
  for (auto& bearing : bearing_params) {
    problem.AddParameterBlock(bearing.data(), kBearingSize, bearing_manifold);
  }

  for (auto& inv_dist : inv_dist_params) {
    problem.AddParameterBlock(&inv_dist, 1);
    problem.SetParameterLowerBound(&inv_dist, 0, SVOConfig::inv_dist_min_value);
  }

  for (const auto& frame_id : remain_frame_ids) {
    const auto it = frame_id_to_index.find(frame_id);
    if (it == frame_id_to_index.end()) {
      continue;
    }
    problem.AddParameterBlock(pose_params[it->second].data(),
                              kPoseSize,
                              box_plus_manifold);
  }

  AddMarginalizationPriorIfAvailable(problem,
                                     *marginalization_prior_,
                                     frame_id_to_index,
                                     inertial_id_to_index,
                                     pose_params,
                                     velocity_params,
                                     bias_acc_params,
                                     bias_gyr_params);

  for (size_t i = 0; i < marginal_map_points.size(); ++i) {
    std::shared_ptr<MapPoint> mp = marginal_map_points[i];
    if (!mp || mp->GetStatus() < MapPoint::Status::TRACKING) {
      continue;
    }

    const FrameCamId frame_cam_id0 = mp->GetHostFrameCamId();
    auto             frame0        = window->GetFrame(frame_cam_id0.frame_id);
    if (!frame0 || !frame_id_to_index.contains(frame_cam_id0.frame_id)) {
      continue;
    }

    const Sophus::SE3d& T_b_c0 = frame0->GetTbc(frame_cam_id0.cam_id);
    double* pose_param0 = pose_params[frame_id_to_index[frame_cam_id0.frame_id]].data();
    const auto& observations   = mp->GetObservation();
    double*     bearing_param  = bearing_params[i].data();
    double*     inv_dist_param = &inv_dist_params[i];

    const auto host_obs_it = observations.find(frame_cam_id0);
    if (host_obs_it != observations.end()) {
      ceres::CostFunction*
        host_bearing_prior_cost = new BearingPriorCost(host_obs_it->second,
                                                       SVOConfig::bearing_cost_scale);
      ceres::LossFunction* loss = new ceres::HuberLoss(SVOConfig::bearing_huber_const);
      problem.AddResidualBlock(host_bearing_prior_cost, loss, bearing_param);
    }

    for (const auto& [frame_cam_id1, bearing] : observations) {
      if (frame_cam_id0 == frame_cam_id1) {
        continue;
      }
      if (!frame_id_to_index.contains(frame_cam_id1.frame_id)) {
        continue;
      }

      auto frame1 = window->GetFrame(frame_cam_id1.frame_id);
      if (!frame1) {
        continue;
      }

      double* pose_param1 = pose_params[frame_id_to_index[frame_cam_id1.frame_id]].data();
      const Sophus::SE3d& T_b_c1 = frame1->GetTbc(frame_cam_id1.cam_id);

      ceres::LossFunction* loss = new ceres::HuberLoss(SVOConfig::bearing_huber_const);
      if (frame_cam_id0.frame_id == frame_cam_id1.frame_id) {
        ceres::CostFunction* cost = new BearingStereoCost(bearing,
                                                          T_b_c1,
                                                          T_b_c0,
                                                          SVOConfig::bearing_cost_scale);
        problem.AddResidualBlock(cost, loss, bearing_param, inv_dist_param);
      }
      else {
        ceres::CostFunction* cost = new BearingCost(bearing,
                                                    T_b_c1,
                                                    T_b_c0,
                                                    SVOConfig::bearing_cost_scale);
        problem.AddResidualBlock(cost,
                                 loss,
                                 pose_param1,
                                 pose_param0,
                                 bearing_param,
                                 inv_dist_param);
      }
    }
  }

  // IMU preintegration factors are excluded from marginalization.

  size_t remain_pose_count = 0;
  for (const auto& frame_id : remain_frame_ids) {
    if (frame_id_to_index.contains(frame_id)) {
      ++remain_pose_count;
    }
  }
  Statistics::startTimer("marginalize eval");
  ceres::Problem::EvaluateOptions eval_opts;
  eval_opts.apply_loss_function = true;

  // Explicit parameter block ordering: [marginalize | keep]
  // Marginalize partition - poses
  for (const auto& frame_id : marginal_frame_ids) {
    if (!frame_id_to_index.contains(frame_id)) {
      continue;
    }
    eval_opts.parameter_blocks.push_back(pose_params[frame_id_to_index[frame_id]].data());
  }
  // Marginalize partition - map points
  for (auto& bearing : bearing_params) {
    eval_opts.parameter_blocks.push_back(bearing.data());
  }
  for (auto& inv_dist : inv_dist_params) {
    eval_opts.parameter_blocks.push_back(&inv_dist);
  }
  // Keep partition - poses
  for (const auto& frame_id : remain_frame_ids) {
    if (!frame_id_to_index.contains(frame_id)) {
      continue;
    }
    eval_opts.parameter_blocks.push_back(pose_params[frame_id_to_index[frame_id]].data());
  }
  ceres::CRSMatrix    ceres_J;
  std::vector<double> residuals;
  problem.Evaluate(eval_opts, nullptr, &residuals, nullptr, &ceres_J);

  Eigen::MatrixXd H;
  Eigen::VectorXd Jt_R;
  if (!CeresUtil::CreateHessianFromCRSMatrix(ceres_J, residuals, H, Jt_R)) {
    ClearPrior();
    Statistics::stopTimer("marginalize eval");
    return;
  }
  Statistics::stopTimer("marginalize eval");

  H = 0.5 * (H + H.transpose());

  const Eigen::Index r = static_cast<Eigen::Index>(remain_pose_count * kPoseSize);
  if (r <= 0 || H.cols() < r || H.rows() < r || Jt_R.size() < r) {
    ClearPrior();
    return;
  }
  const Eigen::Index total_dim = H.cols();
  const Eigen::Index m         = total_dim - r;
  LogI("  Hessian: total_dim={}, m(marginal)={}, r(remain)={}, "
       "remain_pose_count={}",
       total_dim,
       m,
       r,
       remain_pose_count);
  if (m < 0) {
    ClearPrior();
    return;
  }
  if (m == 0) {
    LogW("Skip marginalization update because marginal block is empty (frames: {}, "
         "points: {})",
         marginal_frame_ids.size(),
         marginal_map_points.size());
    return;
  }

  Eigen::MatrixXd A = H.block(m, m, r, r);
  Eigen::VectorXd b = Jt_R.segment(m, r);

  const Eigen::MatrixXd Amm = H.block(0, 0, m, m);
  const Eigen::MatrixXd Amr = H.block(0, m, m, r);
  const Eigen::MatrixXd Arm = H.block(m, 0, r, m);
  const Eigen::VectorXd bmm = Jt_R.segment(0, m);
  const Eigen::VectorXd brr = Jt_R.segment(m, r);

  Statistics::startTimer("marginalize saes");
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(Amm);
  if (saes.info() != Eigen::Success) {
    ClearPrior();
    return;
  }
  Statistics::stopTimer("marginalize saes");

  constexpr double eps         = 1e-8;
  Eigen::VectorXd  inv_eigvals = (saes.eigenvalues().array() > eps)
                                  .select(saes.eigenvalues().array().inverse(), 0.0);
  const Eigen::MatrixXd Amm_inv = saes.eigenvectors() * inv_eigvals.asDiagonal()
                                  * saes.eigenvectors().transpose();

  A = A - Arm * Amm_inv * Amr;
  b = brr - Arm * Amm_inv * bmm;
  A = 0.5 * (A + A.transpose());

  Eigen::VectorXd    x0(r);
  std::vector<int>   prior_block_sizes;
  std::set<uint64_t> prior_frame_ids;
  std::set<uint64_t> prior_preintegration_ids;
  Eigen::Index       offset = 0;
  prior_block_sizes.reserve(remain_pose_count);
  for (const uint64_t& frame_id : remain_frame_ids) {
    if (!frame_id_to_index.contains(frame_id)) {
      continue;
    }
    prior_frame_ids.insert(frame_id);
    prior_block_sizes.push_back(kPoseSize);
    size_t idx                    = frame_id_to_index[frame_id];
    x0.segment<kPoseSize>(offset) = pose_params[idx];
    offset += kPoseSize;
  }
  if (offset != r) {
    ClearPrior();
    return;
  }

  Statistics::startTimer("marginalize saes2");
  if (!marginalization_prior_) {
    marginalization_prior_ = std::make_unique<MarginalizationPrior>();
  }

  marginalization_prior_->frame_ids_          = prior_frame_ids;
  marginalization_prior_->preintegration_ids_ = prior_preintegration_ids;
  marginalization_prior_->block_sizes_        = prior_block_sizes;
  marginalization_prior_->x0_                 = x0;

  if (marginalization_prior_->x0_.size() == 0) {
    marginalization_prior_->J_.resize(0, 0);
    marginalization_prior_->r_.resize(0);
  }
  else {
    int total_block_size = 0;
    for (const int block_size : marginalization_prior_->block_sizes_) {
      total_block_size += block_size;
    }
    if (total_block_size != marginalization_prior_->x0_.size()) {
      ClearPrior();
    }
    else {
      Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes_prior(A);
      if (saes_prior.info() != Eigen::Success) {
        ClearPrior();
      }
      else {
        constexpr double      eps      = 1e-8;
        const Eigen::VectorXd eig_vals = saes_prior.eigenvalues();
        const Eigen::MatrixXd eig_vecs = saes_prior.eigenvectors();

        Eigen::VectorXd sqrt_vals(eig_vals.size());
        Eigen::VectorXd inv_sqrt_vals(eig_vals.size());
        for (int i = 0; i < eig_vals.size(); ++i) {
          if (eig_vals[i] > eps) {
            sqrt_vals[i]     = std::sqrt(eig_vals[i]);
            inv_sqrt_vals[i] = 1.0 / sqrt_vals[i];
          }
          else {
            sqrt_vals[i]     = 0.0;
            inv_sqrt_vals[i] = 0.0;
          }
        }

        marginalization_prior_->J_ = sqrt_vals.asDiagonal() * eig_vecs.transpose();
        marginalization_prior_->r_ = inv_sqrt_vals.asDiagonal() * eig_vecs.transpose()
                                     * b;
      }
    }
  }
  LogI("=== Marginalize END ===");
  LogI("  new prior frame_ids: [{}]", JoinIds(marginalization_prior_->frame_ids_));
  LogI("  new prior preint_ids: [{}]",
       JoinIds(marginalization_prior_->preintegration_ids_));
  LogI("  new prior J: {}x{}, r: {}, x0: {}, blocks: {}",
       marginalization_prior_->J_.rows(),
       marginalization_prior_->J_.cols(),
       marginalization_prior_->r_.size(),
       marginalization_prior_->x0_.size(),
       marginalization_prior_->block_sizes_.size());
  Statistics::stopTimer("marginalize saes2");
}

}  // namespace omni_slam
