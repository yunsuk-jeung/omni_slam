#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

#include <ceres/ceres.h>

#include "config/svio_config.hpp"
#include "config/svo_config.hpp"
#include "database/Frame.hpp"
#include "database/MapPoint.hpp"
#include "odometry/sliding_window.hpp"
#include "optimizer/cost_function.hpp"
#include "optimizer/marginalizer.hpp"
#include "optimizer/parameterization.hpp"
#include "optimizer/vio_estimator.hpp"
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
static constexpr int      kImuResidualSize            = 15;
static constexpr uint64_t kMarginalizerInitialFrameId = 0;

static Eigen::Matrix<double, kImuResidualSize, 1> MakeImuResidualSqrtScale() {
  Eigen::Matrix<double, kImuResidualSize, 1> scale;
  scale.segment<3>(0).setConstant(SVIOConfig::imu_position_residual_scale);
  scale.segment<3>(3).setConstant(SVIOConfig::imu_rotation_residual_scale);
  scale.segment<3>(6).setConstant(SVIOConfig::imu_velocity_residual_scale);
  scale.segment<3>(9).setConstant(SVIOConfig::imu_bias_residual_scale);
  scale.segment<3>(12).setConstant(SVIOConfig::imu_bias_residual_scale);
  scale *= SVIOConfig::imu_residual_scale;
  return scale;
}

// Ceres parameter blocks shared by window optimization and marginalization:
// camera poses plus per-frame inertial states (velocity, accel bias, gyro
// bias), each indexed by frame id. Vectors must be reserved (or fully built)
// before handing block pointers to ceres so they stay stable.
struct WindowBlocks {
  std::vector<Eigen::Vector6d>         pose_params;
  std::unordered_map<uint64_t, size_t> frame_id_to_index;

  std::vector<Eigen::Vector3d>         velocity_params;
  std::vector<Eigen::Vector3d>         bias_acc_params;
  std::vector<Eigen::Vector3d>         bias_gyr_params;
  std::unordered_map<uint64_t, size_t> inertial_id_to_index;

  void AddPose(uint64_t frame_id, const Sophus::SE3d& T_w_b) {
    frame_id_to_index[frame_id] = pose_params.size();
    pose_params.push_back(SE3BoxplusManifold::ToParams(T_w_b));
  }

  void AddInertialState(uint64_t frame_id, const InertialState& state) {
    inertial_id_to_index[frame_id] = velocity_params.size();
    velocity_params.push_back(state.v_w_b);
    bias_acc_params.push_back(state.bias_acc);
    bias_gyr_params.push_back(state.bias_gyr);
  }

  bool HasFrame(uint64_t frame_id) const {
    return frame_id_to_index.contains(frame_id);
  }

  bool HasInertialState(uint64_t frame_id) const {
    return inertial_id_to_index.contains(frame_id);
  }

  double* PoseParam(uint64_t frame_id) {
    return pose_params[frame_id_to_index.at(frame_id)].data();
  }
};

static double SanitizeInvDist(double inv_dist) {
  if (!std::isfinite(inv_dist) || inv_dist <= SVOConfig::inv_dist_min_value) {
    return SVOConfig::inv_dist_initial_value;
  }
  return inv_dist;
}

static void RegisterPoseBlocks(ceres::Problem& problem, WindowBlocks& blocks) {
  auto* se3_box_plus_manifold = new SE3BoxplusManifold();
  for (auto& param : blocks.pose_params) {
    problem.AddParameterBlock(param.data(), kPoseSize, se3_box_plus_manifold);
  }
}

static void RegisterInertialBlocks(ceres::Problem& problem,
                                   WindowBlocks&   blocks) {
  for (size_t i = 0; i < blocks.velocity_params.size(); ++i) {
    problem.AddParameterBlock(blocks.velocity_params[i].data(),
                              kInertialStateDim);
    problem.AddParameterBlock(blocks.bias_acc_params[i].data(),
                              kInertialStateDim);
    problem.AddParameterBlock(blocks.bias_gyr_params[i].data(),
                              kInertialStateDim);
  }
}

static void RegisterMapPointBlocks(ceres::Problem&               problem,
                                   std::vector<Eigen::Vector3d>& bearing_params,
                                   std::vector<double>& inv_dist_params) {
  auto* bearing_manifold = new BearingTangentManifold();
  for (auto& bearing : bearing_params) {
    problem.AddParameterBlock(bearing.data(), kBearingSize, bearing_manifold);
  }
  for (auto& inv_dist : inv_dist_params) {
    problem.AddParameterBlock(&inv_dist, 1);
    problem.SetParameterLowerBound(&inv_dist, 0, SVOConfig::inv_dist_min_value);
  }
}

enum class ImuCostType { kAnalytic, kAutoDiff };
enum class AddImuFactorResult { kAdded, kSkippedDt, kSkippedMissingBlock };

static AddImuFactorResult AddImuFactor(
  ceres::Problem&                                   problem,
  WindowBlocks&                                     blocks,
  const ImuPreintegration&                          preintegration,
  const Eigen::Matrix<double, kImuResidualSize, 1>& residual_sqrt_scale,
  ImuCostType                                       cost_type) {
  if (preintegration.GetDeltaTimeSec() <= 0.0) {
    return AddImuFactorResult::kSkippedDt;
  }

  const uint64_t from_id = preintegration.GetFromFrameId();
  const uint64_t to_id   = preintegration.GetToFrameId();
  if (!blocks.HasFrame(from_id) || !blocks.HasFrame(to_id)
      || !blocks.HasInertialState(from_id) || !blocks.HasInertialState(to_id)) {
    return AddImuFactorResult::kSkippedMissingBlock;
  }

  ceres::CostFunction* imu_cost = nullptr;
  switch (cost_type) {
  case ImuCostType::kAnalytic:
    imu_cost = new ImuPreintegrationCost(preintegration,
                                         SVIOConfig::g_w,
                                         residual_sqrt_scale);
    break;
  case ImuCostType::kAutoDiff:
    imu_cost = new ImuPreintegrationAutoDiffCost(
      new ImuPreintegrationCostAuto(preintegration,
                                    SVIOConfig::g_w,
                                    residual_sqrt_scale));
    break;
  }

  const size_t from_inertial_idx = blocks.inertial_id_to_index.at(from_id);
  const size_t to_inertial_idx   = blocks.inertial_id_to_index.at(to_id);
  problem.AddResidualBlock(imu_cost,
                           nullptr,
                           blocks.PoseParam(from_id),
                           blocks.PoseParam(to_id),
                           blocks.velocity_params[from_inertial_idx].data(),
                           blocks.velocity_params[to_inertial_idx].data(),
                           blocks.bias_acc_params[from_inertial_idx].data(),
                           blocks.bias_gyr_params[from_inertial_idx].data(),
                           blocks.bias_acc_params[to_inertial_idx].data(),
                           blocks.bias_gyr_params[to_inertial_idx].data());
  return AddImuFactorResult::kAdded;
}

// Adds the host-frame bearing prior plus one stereo/cross-frame bearing
// residual per observation of `mp` whose frame has a pose block.
static void AddBearingResidualsForMapPoint(ceres::Problem& problem,
                                           SlidingWindow*  window,
                                           const std::shared_ptr<MapPoint>& mp,
                                           WindowBlocks& blocks,
                                           double*       bearing_param,
                                           double*       inv_dist_param,
                                           bool          robustify_host_prior) {
  const FrameCamId frame_cam_id0 = mp->GetHostFrameCamId();
  auto             frame0        = window->GetFrame(frame_cam_id0.frame_id);
  if (!frame0 || !blocks.HasFrame(frame_cam_id0.frame_id)) {
    return;
  }

  const Sophus::SE3d& T_b_c0       = frame0->GetTbc(frame_cam_id0.cam_id);
  double*             pose_param0  = blocks.PoseParam(frame_cam_id0.frame_id);
  const auto&         observations = mp->GetObservation();

  const auto host_obs_it = observations.find(frame_cam_id0);
  if (host_obs_it != observations.end()) {
    ceres::CostFunction* host_bearing_prior_cost =
      new BearingPriorCost(host_obs_it->second, SVOConfig::bearing_cost_scale);
    ceres::LossFunction* loss =
      robustify_host_prior
        ? new ceres::HuberLoss(SVOConfig::bearing_huber_const)
        : nullptr;
    problem.AddResidualBlock(host_bearing_prior_cost, loss, bearing_param);
  }

  for (const auto& [frame_cam_id1, bearing] : observations) {
    if (frame_cam_id0 == frame_cam_id1) {
      continue;
    }
    if (!blocks.HasFrame(frame_cam_id1.frame_id)) {
      continue;
    }

    auto frame1 = window->GetFrame(frame_cam_id1.frame_id);
    if (!frame1) {
      continue;
    }

    const Sophus::SE3d& T_b_c1 = frame1->GetTbc(frame_cam_id1.cam_id);

    ceres::LossFunction* loss =
      new ceres::HuberLoss(SVOConfig::bearing_huber_const);
    if (frame_cam_id0.frame_id == frame_cam_id1.frame_id) {
      ceres::CostFunction* cost =
        new BearingStereoCost(bearing,
                              T_b_c1,
                              T_b_c0,
                              SVOConfig::bearing_cost_scale);
      problem.AddResidualBlock(cost, loss, bearing_param, inv_dist_param);
    }
    else {
      double* pose_param1 = blocks.PoseParam(frame_cam_id1.frame_id);
      ceres::CostFunction* cost =
        new BearingCost(bearing, T_b_c1, T_b_c0, SVOConfig::bearing_cost_scale);
      problem.AddResidualBlock(cost,
                               loss,
                               pose_param1,
                               pose_param0,
                               bearing_param,
                               inv_dist_param);
    }
  }
}

static void AddMarginalizationPriorIfAvailable(
  ceres::Problem&             problem,
  const MarginalizationPrior& prior,
  WindowBlocks&               blocks) {
  if (prior.block_sizes_.empty() || prior.J_.rows() == 0 || prior.r_.size() == 0
      || prior.x0_.size() == 0) {
    LogW("Prior skipped: empty data (blocks={}, J_rows={}, r={}, x0={})",
         prior.block_sizes_.size(),
         prior.J_.rows(),
         prior.r_.size(),
         prior.x0_.size());
    return;
  }

  std::vector<double*> prior_param_blocks;
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
      LogW(
        "Prior skipped: extra_block_count({}) not divisible by preint_ids({})",
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

  prior_param_blocks.reserve(
    frame_ids.size() + preintegration_ids.size() * per_inertial_block_count);

  for (const uint64_t frame_id : frame_ids) {
    auto it = blocks.frame_id_to_index.find(frame_id);
    if (it == blocks.frame_id_to_index.end()) {
      LogW("Prior skipped: frame_id {} not found in window", frame_id);
      return;
    }
    const size_t idx = it->second;
    if (idx >= blocks.pose_params.size()) {
      LogW("Prior skipped: frame_id {} idx {} >= pose_params size {}",
           frame_id,
           idx,
           blocks.pose_params.size());
      return;
    }
    prior_param_blocks.push_back(blocks.pose_params[idx].data());
  }

  for (const uint64_t frame_id : preintegration_ids) {
    auto it = blocks.inertial_id_to_index.find(frame_id);
    if (it == blocks.inertial_id_to_index.end()) {
      LogW("Prior skipped: preint id {} not found in inertial index", frame_id);
      return;
    }
    const size_t idx = it->second;
    if (idx >= blocks.velocity_params.size()
        || idx >= blocks.bias_acc_params.size()
        || idx >= blocks.bias_gyr_params.size()) {
      LogW("Prior skipped: preint id {} idx {} out of inertial param bounds",
           frame_id,
           idx);
      return;
    }
    if (per_inertial_block_count == 3) {
      prior_param_blocks.push_back(blocks.velocity_params[idx].data());
    }
    prior_param_blocks.push_back(blocks.bias_acc_params[idx].data());
    prior_param_blocks.push_back(blocks.bias_gyr_params[idx].data());
  }

  LogI("Prior added: frame_ids=[{}], preint_ids=[{}], blocks={}, residuals={}",
       JoinIds(frame_ids),
       JoinIds(preintegration_ids),
       block_sizes.size(),
       prior.r_.size());

  ceres::CostFunction* prior_cost = new MarginalizationCost(prior);
  problem.AddResidualBlock(prior_cost, nullptr, prior_param_blocks);
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

    std::shared_ptr<Frame> host_frame =
      window->GetFrame(mp->GetHostFrameCamId().frame_id);
    if (!host_frame) {
      continue;
    }

    const Sophus::SE3d    Twc0  = host_frame->GetTwc(kCamIdx);
    const Eigen::Vector3d p_c0  = mp->GetBearing() / mp->GetInvDist();
    const Eigen::Vector3d p_w   = Twc0 * p_c0;
    const Sophus::SE3d&   T_b_c = frame->GetTbc(kCamIdx);

    ceres::CostFunction* cost =
      new PoseOnlyBearingCost(p_w,
                              bearing,
                              T_b_c,
                              SVOConfig::bearing_cost_scale);
    ceres::LossFunction* loss =
      new ceres::HuberLoss(SVOConfig::bearing_huber_const);
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
  marginalization_prior_->preintegration_ids_.insert(
    kMarginalizerInitialFrameId);

  constexpr int kInitialPriorSize = kPoseSize + 2 * kBiasSize;
  marginalization_prior_->x0_     = Eigen::VectorXd::Zero(kInitialPriorSize);
  marginalization_prior_->J_      = Eigen::MatrixXd::Zero(kInitialPriorSize,
                                                     kInitialPriorSize);
  marginalization_prior_->J_.topLeftCorner(3u, 3u) =
    SVIOConfig::marginalizer_initial_prior_weight
    * Eigen::MatrixXd::Identity(3u, 3u);
  marginalization_prior_->J_(5,
                             5) = SVIOConfig::marginalizer_initial_prior_weight;

  marginalization_prior_->J_.block(kPoseSize, kPoseSize, kBiasSize, kBiasSize) =
    SVIOConfig::marginalizer_initial_bias_weight
    * Eigen::MatrixXd::Identity(kBiasSize, kBiasSize);

  marginalization_prior_->J_.block(kPoseSize + kBiasSize,
                                   kPoseSize + kBiasSize,
                                   kBiasSize,
                                   kBiasSize) =
    SVIOConfig::marginalizer_initial_bias_weight
    * Eigen::MatrixXd::Identity(kBiasSize, kBiasSize);
  marginalization_prior_->block_sizes_ = {kPoseSize, kBiasSize, kBiasSize};
  marginalization_prior_->r_ =
    Eigen::VectorXd::Zero(marginalization_prior_->x0_.size());
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
  const auto     imu_residual_sqrt_scale = MakeImuResidualSqrtScale();

  WindowBlocks blocks;
  blocks.pose_params.reserve(frames.size());
  for (const auto& [frame_id, frame] : frames) {
    blocks.AddPose(frame_id, frame->GetTwb());
  }
  RegisterPoseBlocks(problem, blocks);

  for (const auto& [frame_id, _] : frames) {
    const auto state_it = inertial_states.find(frame_id);
    if (state_it == inertial_states.end()) {
      OMNI_ASSERT_MESSAGE(true, "inertial state missing");
      continue;
    }
    blocks.AddInertialState(frame_id, state_it->second);
  }
  RegisterInertialBlocks(problem, blocks);

  for (const auto& [_, preintegration] : imu_preintegrations) {
    AddImuFactor(problem,
                 blocks,
                 preintegration,
                 imu_residual_sqrt_scale,
                 ImuCostType::kAnalytic);
  }

  std::unordered_map<uint64_t, size_t> mp_id_to_index;
  std::vector<Eigen::Vector3d>         bearing_params;
  std::vector<double>                  inv_dist_params;
  bearing_params.reserve(window->GetMapPointCount());
  inv_dist_params.reserve(window->GetMapPointCount());

  for (const auto& [mp_id, mp] : map_points) {
    mp_id_to_index[mp_id] = bearing_params.size();
    bearing_params.push_back(mp->GetBearing());
    inv_dist_params.push_back(SanitizeInvDist(mp->GetInvDist()));
  }
  RegisterMapPointBlocks(problem, bearing_params, inv_dist_params);
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
  AddMarginalizationPriorIfAvailable(problem, *marginalization_prior_, blocks);

  for (const auto& [mp_id, mp] : map_points) {
    if (mp->GetStatus() < MapPoint::Status::TRACKING) {
      continue;
    }
    const double inv_dist = mp->GetInvDist();
    if (!std::isfinite(inv_dist) || inv_dist <= 0.0) {
      continue;
    }

    const size_t mp_idx = mp_id_to_index[mp_id];
    AddBearingResidualsForMapPoint(problem,
                                   window,
                                   mp,
                                   blocks,
                                   bearing_params[mp_idx].data(),
                                   &inv_dist_params[mp_idx],
                                   /*robustify_host_prior=*/false);
  }

  ceres::Solver::Options options;
  options.linear_solver_type           = ceres::SPARSE_NORMAL_CHOLESKY;
  options.num_threads                  = SVOConfig::window_num_threads;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations           = SVOConfig::window_max_iterations;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  for (const auto& [frame_id, frame] : frames) {
    frame->SetTwb(SE3BoxplusManifold::FromParams(blocks.PoseParam(frame_id)));
  }

  for (const auto& [frame_id, idx] : blocks.inertial_id_to_index) {
    auto state_it = inertial_states.find(frame_id);
    if (state_it == inertial_states.end()) {
      continue;
    }
    state_it->second.v_w_b    = blocks.velocity_params[idx];
    state_it->second.bias_acc = blocks.bias_acc_params[idx];
    state_it->second.bias_gyr = blocks.bias_gyr_params[idx];
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
    mp->SetInvDist(
      std::max(inv_dist_params[idx], SVOConfig::inv_dist_min_value));
  }
}

void VIOEstimator::Marginalize(
  SlidingWindow*                               window,
  std::set<uint64_t>                           marginal_frame_ids,
  std::set<uint64_t>                           marginal_inertial_state_ids,
  const std::map<uint64_t, InertialState>&     inertial_states,
  const std::map<uint64_t, ImuPreintegration>& imu_preintegrations) {
  if (!window
      || (marginal_frame_ids.empty() && marginal_inertial_state_ids.empty())) {
    return;
  }

  LogI("=== Marginalize START ===");
  LogI("  marginal_frame_ids: [{}]", JoinIds(marginal_frame_ids));
  LogI("  marginal_inertial_ids: [{}]", JoinIds(marginal_inertial_state_ids));
  LogI("  prior frame_ids: [{}]", JoinIds(marginalization_prior_->frame_ids_));
  LogI("  prior preint_ids: [{}]",
       JoinIds(marginalization_prior_->preintegration_ids_));
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

  // ========================================
  // Build inertial state sets
  // ========================================
  std::set<uint64_t> marginal_inertial_ids;
  std::set<uint64_t> remain_inertial_ids;

  for (const uint64_t id : marginal_inertial_state_ids) {
    if (inertial_states.count(id) > 0) {
      marginal_inertial_ids.insert(id);
    }
  }

  auto imu_factor_touches_marginal =
    [&](const ImuPreintegration& preintegration) {
      const uint64_t from_id = preintegration.GetFromFrameId();
      const uint64_t to_id   = preintegration.GetToFrameId();
      return marginal_inertial_ids.count(from_id) > 0
             || marginal_inertial_ids.count(to_id) > 0
             || marginal_frame_ids.count(from_id) > 0
             || marginal_frame_ids.count(to_id) > 0;
    };

  auto add_remain_inertial_if_active = [&](uint64_t id) {
    if (marginal_inertial_ids.count(id) > 0 || inertial_states.count(id) == 0) {
      return;
    }
    remain_inertial_ids.insert(id);
  };

  for (const uint64_t id : marginalization_prior_->preintegration_ids_) {
    add_remain_inertial_if_active(id);
  }

  for (const auto& [_, preintegration] : imu_preintegrations) {
    if (!imu_factor_touches_marginal(preintegration)) {
      continue;
    }
    const uint64_t from_id = preintegration.GetFromFrameId();
    const uint64_t to_id   = preintegration.GetToFrameId();
    add_remain_inertial_if_active(from_id);
    add_remain_inertial_if_active(to_id);
  }

  // Ensure pose blocks exist for all inertial states.
  for (const uint64_t id : marginal_inertial_ids) {
    if (marginal_frame_ids.count(id) == 0 && remain_frame_ids.count(id) == 0
        && window->GetFrame(id)) {
      remain_frame_ids.insert(id);
    }
  }
  for (const uint64_t id : remain_inertial_ids) {
    if (marginal_frame_ids.count(id) == 0 && remain_frame_ids.count(id) == 0
        && window->GetFrame(id)) {
      remain_frame_ids.insert(id);
    }
  }

  ceres::Problem problem;
  const auto     imu_residual_sqrt_scale = MakeImuResidualSqrtScale();

  // Parameter blocks in [marginal | remain] order. The Schur complement below
  // relies on the explicit eval ordering, not on registration order.
  WindowBlocks blocks;
  blocks.pose_params.reserve(marginal_frame_ids.size()
                             + remain_frame_ids.size());
  for (const auto& frame_id : marginal_frame_ids) {
    if (auto frame = window->GetFrame(frame_id)) {
      blocks.AddPose(frame_id, frame->GetTwb());
    }
  }
  for (const auto& frame_id : remain_frame_ids) {
    if (auto frame = window->GetFrame(frame_id)) {
      blocks.AddPose(frame_id, frame->GetTwb());
    }
  }
  RegisterPoseBlocks(problem, blocks);

  LogI("  remain_frame_ids: [{}]", JoinIds(remain_frame_ids));
  LogI("  marginal_inertial_ids: [{}]", JoinIds(marginal_inertial_ids));
  LogI("  remain_inertial_ids: [{}]", JoinIds(remain_inertial_ids));

  auto add_inertial_entry = [&](uint64_t id) {
    if (blocks.HasInertialState(id)) {
      return;
    }
    const auto it = inertial_states.find(id);
    if (it == inertial_states.end()) {
      LogW("Skip inertial state {} in marginalization: state missing", id);
      return;
    }
    blocks.AddInertialState(id, it->second);
  };

  for (const uint64_t id : marginal_inertial_ids) {
    add_inertial_entry(id);
  }
  for (const uint64_t id : remain_inertial_ids) {
    add_inertial_entry(id);
  }
  RegisterInertialBlocks(problem, blocks);

  std::vector<Eigen::Vector3d> bearing_params;
  std::vector<double>          inv_dist_params;
  bearing_params.reserve(marginal_map_points.size());
  inv_dist_params.reserve(marginal_map_points.size());

  for (const auto& map_point : marginal_map_points) {
    bearing_params.push_back(map_point->GetBearing());
    inv_dist_params.push_back(SanitizeInvDist(map_point->GetInvDist()));
  }
  RegisterMapPointBlocks(problem, bearing_params, inv_dist_params);

  AddMarginalizationPriorIfAvailable(problem, *marginalization_prior_, blocks);

  for (size_t i = 0; i < marginal_map_points.size(); ++i) {
    const std::shared_ptr<MapPoint>& mp = marginal_map_points[i];
    if (!mp || mp->GetStatus() < MapPoint::Status::TRACKING) {
      continue;
    }
    AddBearingResidualsForMapPoint(problem,
                                   window,
                                   mp,
                                   blocks,
                                   bearing_params[i].data(),
                                   &inv_dist_params[i],
                                   /*robustify_host_prior=*/true);
  }

  size_t marginal_imu_total        = 0;
  size_t marginal_imu_added        = 0;
  size_t marginal_imu_skip_dt      = 0;
  size_t marginal_imu_skip_missing = 0;
  size_t marginal_imu_skip_keep    = 0;
  for (const auto& [_, preintegration] : imu_preintegrations) {
    ++marginal_imu_total;
    if (preintegration.GetDeltaTimeSec() <= 0.0) {
      ++marginal_imu_skip_dt;
      continue;
    }
    if (!imu_factor_touches_marginal(preintegration)) {
      ++marginal_imu_skip_keep;
      continue;
    }

    switch (AddImuFactor(problem,
                         blocks,
                         preintegration,
                         imu_residual_sqrt_scale,
                         ImuCostType::kAutoDiff)) {
    case AddImuFactorResult::kAdded:
      ++marginal_imu_added;
      break;
    case AddImuFactorResult::kSkippedMissingBlock:
      ++marginal_imu_skip_missing;
      break;
    case AddImuFactorResult::kSkippedDt:
      ++marginal_imu_skip_dt;
      break;
    }
  }
  LogI(
    "Marginalize IMU factors: total={}, added={}, skip_dt={}, skip_missing={}, "
    "skip_keep={}, "
    "marginal_inertial={}, remain_inertial={}, preints={}",
    marginal_imu_total,
    marginal_imu_added,
    marginal_imu_skip_dt,
    marginal_imu_skip_missing,
    marginal_imu_skip_keep,
    marginal_inertial_ids.size(),
    remain_inertial_ids.size(),
    imu_preintegrations.size());

  // The eval-ordering / Schur-complement code below predates WindowBlocks;
  // alias its members to keep that section unchanged.
  auto& pose_params          = blocks.pose_params;
  auto& frame_id_to_index    = blocks.frame_id_to_index;
  auto& velocity_params      = blocks.velocity_params;
  auto& bias_acc_params      = blocks.bias_acc_params;
  auto& bias_gyr_params      = blocks.bias_gyr_params;
  auto& inertial_id_to_index = blocks.inertial_id_to_index;

  size_t remain_pose_count = 0;
  for (const auto& frame_id : remain_frame_ids) {
    if (frame_id_to_index.contains(frame_id)) {
      ++remain_pose_count;
    }
  }
  size_t remain_inertial_count = 0;
  for (const auto& id : remain_inertial_ids) {
    if (inertial_id_to_index.contains(id)) {
      ++remain_inertial_count;
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
    eval_opts.parameter_blocks.push_back(
      pose_params[frame_id_to_index[frame_id]].data());
  }
  // Marginalize partition - inertial states
  for (const auto& id : marginal_inertial_ids) {
    if (!inertial_id_to_index.contains(id)) {
      continue;
    }
    const size_t idx = inertial_id_to_index[id];
    eval_opts.parameter_blocks.push_back(velocity_params[idx].data());
    eval_opts.parameter_blocks.push_back(bias_acc_params[idx].data());
    eval_opts.parameter_blocks.push_back(bias_gyr_params[idx].data());
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
    eval_opts.parameter_blocks.push_back(
      pose_params[frame_id_to_index[frame_id]].data());
  }
  // Keep partition - inertial states
  for (const auto& id : remain_inertial_ids) {
    if (!inertial_id_to_index.contains(id)) {
      continue;
    }
    const size_t idx = inertial_id_to_index[id];
    eval_opts.parameter_blocks.push_back(velocity_params[idx].data());
    eval_opts.parameter_blocks.push_back(bias_acc_params[idx].data());
    eval_opts.parameter_blocks.push_back(bias_gyr_params[idx].data());
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

  const Eigen::Index r = static_cast<Eigen::Index>(
    remain_pose_count * kPoseSize + remain_inertial_count * kInertialStateSize);
  if (r <= 0 || H.cols() < r || H.rows() < r || Jt_R.size() < r) {
    ClearPrior();
    return;
  }
  const Eigen::Index total_dim = H.cols();
  const Eigen::Index m         = total_dim - r;
  LogI("  Hessian: total_dim={}, m(marginal)={}, r(remain)={}, "
       "remain_pose_count={}, remain_inertial_count={}",
       total_dim,
       m,
       r,
       remain_pose_count,
       remain_inertial_count);
  if (m < 0) {
    ClearPrior();
    return;
  }
  if (m == 0) {
    LogW("Skip marginalization update because marginal block is empty (frames: "
         "{}, "
         "inertial: {}, points: {})",
         marginal_frame_ids.size(),
         marginal_inertial_ids.size(),
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
                                  .select(saes.eigenvalues().array().inverse(),
                                          0.0);
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
  prior_block_sizes.reserve(remain_pose_count + remain_inertial_count * 3);
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
  for (const uint64_t& id : remain_inertial_ids) {
    if (!inertial_id_to_index.contains(id)) {
      continue;
    }
    const size_t idx = inertial_id_to_index[id];
    prior_preintegration_ids.insert(id);
    prior_block_sizes.push_back(kInertialStateDim);
    prior_block_sizes.push_back(kInertialStateDim);
    prior_block_sizes.push_back(kInertialStateDim);
    x0.segment<kInertialStateDim>(offset) = velocity_params[idx];
    offset += kInertialStateDim;
    x0.segment<kInertialStateDim>(offset) = bias_acc_params[idx];
    offset += kInertialStateDim;
    x0.segment<kInertialStateDim>(offset) = bias_gyr_params[idx];
    offset += kInertialStateDim;
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

        marginalization_prior_->J_ = sqrt_vals.asDiagonal()
                                     * eig_vecs.transpose();
        marginalization_prior_->r_ = inv_sqrt_vals.asDiagonal()
                                     * eig_vecs.transpose() * b;
      }
    }
  }
  LogI("=== Marginalize END ===");
  LogI("  new prior frame_ids: [{}]",
       JoinIds(marginalization_prior_->frame_ids_));
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
