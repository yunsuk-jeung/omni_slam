#include "optimizer/vio_estimator.hpp"

#include <ceres/ceres.h>

#include <algorithm>
#include <cstdint>
#include <cmath>
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
#include "utils/timer.hpp"

namespace omni_slam {
namespace {

static constexpr int      kPoseSize                   = 6;
static constexpr int      kBearingSize                = 3;
static constexpr int      kInertialStateDim           = 3;
static constexpr uint64_t kMarginalizerInitialFrameId = 0;

static void AddMarginalizationPriorIfAvailable(
  ceres::Problem&                             problem,
  Marginalizer*                               marginalizer,
  const std::unordered_map<uint64_t, size_t>& frame_id_to_index,
  std::vector<Eigen::Vector6d>&               pose_params) {
  if (!marginalizer) {
    LogE("Marginalizer doesn't exist");
    return;
  }

  std::vector<double*> prior_pose_blocks;
  prior_pose_blocks.reserve(marginalizer->GetFrameIds().size());

  for (const uint64_t frame_id : marginalizer->GetFrameIds()) {
    auto it = frame_id_to_index.find(frame_id);
    if (it == frame_id_to_index.end()) {
      return;
    }
    prior_pose_blocks.push_back(pose_params[it->second].data());
  }

  ceres::CostFunction* prior_cost = marginalizer->CreateCost();
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
  : marginalizer_(
      std::make_unique<Marginalizer>(kMarginalizerInitialFrameId,
                                     SVIOConfig::marginalizer_initial_prior_weight))
  , imu_factors_by_to_frame_{} {}

VIOEstimator::~VIOEstimator() = default;

void VIOEstimator::OptimizeWindow(
  SlidingWindow*                               window,
  std::map<uint64_t, InertialState>*           inertial_states,
  const std::map<uint64_t, ImuPreintegration>* imu_preintegrations) {
  if (!window) {
    return;
  }

  const auto& frames     = window->GetFrames();
  const auto& map_points = window->GetMapPoints();
  // if (frames.size() < 3) {
  //   return;
  // }

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

  std::vector<Eigen::Vector3d> velocity_params;
  std::vector<Eigen::Vector3d> bias_acc_params;
  std::vector<Eigen::Vector3d> bias_gyr_params;
  velocity_params.reserve(frames.size());
  bias_acc_params.reserve(frames.size());
  bias_gyr_params.reserve(frames.size());

  for (const auto& [frame_id, _] : frames) {
    InertialState state;
    if (inertial_states) {
      const auto it = inertial_states->find(frame_id);
      if (it != inertial_states->end()) {
        state = it->second;
      }
    }

    velocity_params.push_back(state.v_w_b);
    bias_acc_params.push_back(state.bias_acc);
    bias_gyr_params.push_back(state.bias_gyr);
  }

  for (size_t i = 0; i < frames.size(); ++i) {
    problem.AddParameterBlock(velocity_params[i].data(), kInertialStateDim);
    problem.AddParameterBlock(bias_acc_params[i].data(), kInertialStateDim);
    problem.AddParameterBlock(bias_gyr_params[i].data(), kInertialStateDim);
  }

  imu_factors_by_to_frame_.clear();
  if (imu_preintegrations && !frames.empty()) {
    for (const auto& [_, preintegration] : *imu_preintegrations) {
      if (preintegration.GetDeltaTimeSec() <= 0.0) {
        continue;
      }
      imu_factors_by_to_frame_.insert_or_assign(preintegration.GetToFrameId(),
                                                preintegration);
    }
  }

  for (const auto& [_, preintegration] : imu_factors_by_to_frame_) {
    const uint64_t from_frame_id = preintegration.GetFromFrameId();
    const uint64_t to_factor_id  = preintegration.GetToFrameId();
    const auto     from_it       = frame_id_to_index.find(from_frame_id);
    const auto     to_it         = frame_id_to_index.find(to_factor_id);
    if (from_it == frame_id_to_index.end() || to_it == frame_id_to_index.end()) {
      continue;
    }

    const size_t from_idx = from_it->second;
    const size_t to_idx   = to_it->second;

    ceres::CostFunction* imu_cost = new ImuPreintegrationCost(preintegration,
                                                              SVIOConfig::g_w);
    problem.AddResidualBlock(imu_cost,
                             nullptr,
                             pose_params[from_idx].data(),
                             pose_params[to_idx].data(),
                             velocity_params[from_idx].data(),
                             velocity_params[to_idx].data(),
                             bias_acc_params[from_idx].data(),
                             bias_gyr_params[from_idx].data(),
                             bias_acc_params[to_idx].data(),
                             bias_gyr_params[to_idx].data());
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

  AddMarginalizationPriorIfAvailable(problem,
                                     marginalizer_.get(),
                                     frame_id_to_index,
                                     pose_params);

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

  if (inertial_states) {
    for (const auto& [frame_id, idx] : frame_id_to_index) {
      InertialState& state = (*inertial_states)[frame_id];
      state.v_w_b          = velocity_params[idx];
      state.bias_acc       = bias_acc_params[idx];
      state.bias_gyr       = bias_gyr_params[idx];
    }
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

void VIOEstimator::Marginalize(SlidingWindow*     window,
                               std::set<uint64_t> marginal_kf_ids) {
  if (!window || marginal_kf_ids.empty()) {
    return;
  }

  const auto& prev_frame_ids = marginalizer_->GetFrameIds();

  std::set<uint64_t> remain_frame_ids;
  for (const uint64_t f_id : prev_frame_ids) {
    if (marginal_kf_ids.count(f_id) == 0) {
      remain_frame_ids.insert(f_id);
    }
  }

  auto& mp_id_to_mp = window->GetMapPoints();

  std::vector<std::shared_ptr<MapPoint>> marginal_map_points;
  marginal_map_points.reserve(mp_id_to_mp.size());
  for (auto& [_, mp] : mp_id_to_mp) {
    if (marginal_kf_ids.count(mp->GetHostFrameCamId().frame_id) > 0) {
      marginal_map_points.push_back(mp);
    }
  }

  for (const auto& mp : marginal_map_points) {
    auto& frame_cam_id_to_bearing = mp->GetObservation();
    for (const auto& [frame_cam_id, _] : frame_cam_id_to_bearing) {
      if (marginal_kf_ids.count(frame_cam_id.frame_id) > 0) {
        continue;
      }
      if (remain_frame_ids.count(frame_cam_id.frame_id) == 0) {
        remain_frame_ids.insert(frame_cam_id.frame_id);
      }
    }
  }

  ceres::Problem problem;

  std::vector<Eigen::Vector6d>         pose_params;
  std::unordered_map<uint64_t, size_t> frame_id_to_index;
  auto*                                box_plus_manifold = new SE3BoxplusManifold();

  pose_params.reserve(marginal_kf_ids.size() + remain_frame_ids.size());
  for (const auto& frame_id : marginal_kf_ids) {
    auto frame = window->GetFrame(frame_id);
    if (!frame) {
      continue;
    }
    frame_id_to_index[frame_id] = pose_params.size();
    pose_params.push_back(SE3BoxplusManifold::ToParams(frame->GetTwb()));
    problem.AddParameterBlock(pose_params.back().data(), kPoseSize, box_plus_manifold);
  }

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
    auto frame = window->GetFrame(frame_id);
    if (!frame) {
      continue;
    }
    frame_id_to_index[frame_id] = pose_params.size();
    pose_params.push_back(SE3BoxplusManifold::ToParams(frame->GetTwb()));
    problem.AddParameterBlock(pose_params.back().data(), kPoseSize, box_plus_manifold);
  }

  AddMarginalizationPriorIfAvailable(problem,
                                     marginalizer_.get(),
                                     frame_id_to_index,
                                     pose_params);

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

    auto* bearing_loss = new ceres::HuberLoss(SVOConfig::bearing_huber_const);

    const auto host_obs_it = observations.find(frame_cam_id0);
    if (host_obs_it != observations.end()) {
      ceres::CostFunction*
        host_bearing_prior_cost = new BearingPriorCost(host_obs_it->second,
                                                       SVOConfig::bearing_cost_scale);
      problem.AddResidualBlock(host_bearing_prior_cost, bearing_loss, bearing_param);
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

      if (frame_cam_id0.frame_id == frame_cam_id1.frame_id) {
        ceres::CostFunction* cost = new BearingStereoCost(bearing,
                                                          T_b_c1,
                                                          T_b_c0,
                                                          SVOConfig::bearing_cost_scale);
        problem.AddResidualBlock(cost, bearing_loss, bearing_param, inv_dist_param);
      }
      else {
        ceres::CostFunction* cost = new BearingCost(bearing,
                                                    T_b_c1,
                                                    T_b_c0,
                                                    SVOConfig::bearing_cost_scale);
        problem.AddResidualBlock(cost,
                                 bearing_loss,
                                 pose_param1,
                                 pose_param0,
                                 bearing_param,
                                 inv_dist_param);
      }
    }
  }

  Statistics::startTimer("marginalize eval");
  ceres::Problem::EvaluateOptions eval_opts;
  eval_opts.apply_loss_function = true;
  ceres::CRSMatrix    ceres_J;
  std::vector<double> residuals;
  problem.Evaluate(eval_opts, nullptr, &residuals, nullptr, &ceres_J);

  Eigen::MatrixXd H;
  Eigen::VectorXd Jt_R;
  CeresUtil::CreateHessianFromCRSMatrix(ceres_J, residuals, H, Jt_R);
  Statistics::stopTimer("marginalize eval");

  H = 0.5 * (H + H.transpose());

  const Eigen::Index r = static_cast<Eigen::Index>(remain_frame_ids.size() * kPoseSize);
  const Eigen::Index total_dim = H.cols();
  const Eigen::Index m         = total_dim - r;

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
    marginalizer_->Clear();
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

  Eigen::VectorXd x0(r);
  Eigen::Index    offset = 0;
  for (const uint64_t& frame_id : remain_frame_ids) {
    if (!frame_id_to_index.contains(frame_id)) {
      x0.segment<kPoseSize>(offset).setZero();
      offset += kPoseSize;
      continue;
    }
    size_t idx                    = frame_id_to_index[frame_id];
    x0.segment<kPoseSize>(offset) = pose_params[idx];
    offset += kPoseSize;
  }

  Statistics::startTimer("marginalize saes2");
  marginalizer_->SetPrior(remain_frame_ids, A, b, x0);
  Statistics::stopTimer("marginalize saes2");
}

}  // namespace omni_slam
