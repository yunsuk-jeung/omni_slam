#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include <ceres/ceres.h>

#include "config/svo_config.hpp"
#include "database/Frame.hpp"
#include "database/MapPoint.hpp"
#include "odometry/sliding_window.hpp"
#include "optimizer/cost_function.hpp"
#include "optimizer/marginalizer.hpp"
#include "optimizer/parameterization.hpp"
#include "optimizer/vo_estimator.hpp"
#include "utils/ceres_utils.hpp"
#include "utils/eigen_utils.hpp"
#include "utils/logger.hpp"
#include "utils/timer.hpp"

namespace omni_slam {

static constexpr int      kPoseSize                   = 6;
static constexpr int      kBearingSize                = 3;
static constexpr uint64_t kMarginalizerInitialFrameId = 0;

static void AddMarginalizationPriorIfAvailable(
  ceres::Problem&                             problem,
  const MarginalizationPrior&                 prior,
  const std::unordered_map<uint64_t, size_t>& frame_id_to_index,
  std::vector<Eigen::Vector6d>&               pose_params) {
  if (prior.frame_ids_.empty() || prior.block_sizes_.empty()
      || prior.J_.rows() == 0 || prior.r_.size() == 0
      || prior.x0_.size() == 0) {
    return;
  }

  std::vector<double*> prior_pose_blocks;
  prior_pose_blocks.reserve(prior.frame_ids_.size());

  for (const uint64_t frame_id : prior.frame_ids_) {
    auto it = frame_id_to_index.find(frame_id);
    if (it == frame_id_to_index.end() || it->second >= pose_params.size()) {
      return;
    }
    prior_pose_blocks.push_back(pose_params[it->second].data());
  }

  ceres::CostFunction* prior_cost = new MarginalizationCost(prior);
  problem.AddResidualBlock(prior_cost, nullptr, prior_pose_blocks);
}

void VOEstimator::OptimizeSingleFrame(std::shared_ptr<Frame> frame,
                                      SlidingWindow*         window) {
  constexpr size_t kCamIdx     = 0;
  auto&            mp_id_to_uv = frame->GetObservation(kCamIdx);
  auto&            T_w_b       = frame->GetTwb();
  auto&            T_b_c       = frame->GetTbc(kCamIdx);

  Eigen::Vector6d box_w_b = SE3BoxplusManifold::ToParams(frame->GetTwb());

  ceres::Problem      problem;
  SE3BoxplusManifold* se3_box_plus_manifold = new SE3BoxplusManifold();
  problem.AddParameterBlock(box_w_b.data(), kPoseSize, se3_box_plus_manifold);

  auto& mp_id_to_bearing = frame->GetObservation(kCamIdx);

  for (auto& [mp_id, bearing] : mp_id_to_bearing) {
    std::shared_ptr<MapPoint> mp = window->GetMapPoint(mp_id);

    if (!mp || mp->GetStatus() < MapPoint::Status::TRACKING) {
      continue;
    }

    std::shared_ptr<Frame> f0 =
      window->GetFrame(mp->GetHostFrameCamId().frame_id);
    Sophus::SE3d Twc = f0->GetTwc(kCamIdx);

    // world point reconstruction from host frame
    const Sophus::SE3d Twc0 = f0->GetTwc(kCamIdx);

    // bearing * inverse depth  → point in host camera frame
    const Eigen::Vector3d p_c0 = mp->GetBearing() / mp->GetInvDist();

    // world point
    const Eigen::Vector3d p_w = Twc0 * p_c0;

    // bearing residual (pose-only)
    ceres::CostFunction* cost =
      new PoseOnlyBearingCost(p_w,
                              bearing,
                              T_b_c,
                              SVOConfig::bearing_cost_scale);

    // auto* cost = new ceres::AutoDiffCostFunction<PoseOnlyBearingCostAuto,
    //                                              2,  // residual dim
    //                                              6   // pose dim
    //                                              >(
    //   new PoseOnlyBearingCostAuto(p_w, bearing, T_b_c));

    ceres::LossFunction* loss =
      new ceres::HuberLoss(SVOConfig::bearing_huber_const);
    problem.AddResidualBlock(cost,
                             loss,  // no robust loss for now
                             box_w_b.data());
  }
  // solver options
  ceres::Solver::Options options;
  options.linear_solver_type           = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;
  options.num_threads                  = 2;
  options.max_num_iterations           = SVOConfig::single_frame_max_iterations;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // LogD("{}", summary.FullReport());

  // update pose
  frame->SetTwb(SE3BoxplusManifold::FromParams(box_w_b.data()));
}

VOEstimator::VOEstimator()
  : marginalization_prior_(std::make_unique<MarginalizationPrior>()) {
  marginalization_prior_->frame_ids_.insert(kMarginalizerInitialFrameId);
  marginalization_prior_->J_ = SVOConfig::marginalizer_initial_prior_weight
                               * Eigen::MatrixXd::Identity(kPoseSize,
                                                           kPoseSize);
  marginalization_prior_->r_           = Eigen::VectorXd::Zero(kPoseSize);
  marginalization_prior_->x0_          = Eigen::VectorXd::Zero(kPoseSize);
  marginalization_prior_->block_sizes_ = {kPoseSize};
}

VOEstimator::~VOEstimator() = default;

void VOEstimator::ClearPrior() {
  if (!marginalization_prior_) {
    return;
  }
  marginalization_prior_->J_.resize(0, 0);
  marginalization_prior_->r_.resize(0);
  marginalization_prior_->x0_.resize(0);
  marginalization_prior_->frame_ids_.clear();
  marginalization_prior_->block_sizes_.clear();
}

void VOEstimator::OptimizeWindow(SlidingWindow* window) {
  if (!window) {
    return;
  }

  const auto& frames     = window->GetFrames();
  const auto& map_points = window->GetMapPoints();

  if (frames.size() < 2) {
    return;
  }

  ceres::Problem problem;

  // add pose parameters to problem

  std::vector<Eigen::Vector6d>         pose_params;
  std::unordered_map<uint64_t, size_t> frame_id_to_index;
  pose_params.reserve(frames.size());

  for (const auto& [frame_id, frame] : frames) {
    frame_id_to_index[frame_id] = pose_params.size();
    pose_params.push_back(SE3BoxplusManifold::ToParams(frame->GetTwb()));
  }

  SE3BoxplusManifold* se3_box_plus_manifold = new SE3BoxplusManifold();
  for (auto& param : pose_params) {
    problem.AddParameterBlock(param.data(), kPoseSize, se3_box_plus_manifold);
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

  BearingTangentManifold* bearing_manifold = new BearingTangentManifold();
  for (auto& bearing : bearing_params) {
    problem.AddParameterBlock(bearing.data(), kBearingSize, bearing_manifold);
  }

  for (auto& inv_dist : inv_dist_params) {
    problem.AddParameterBlock(&inv_dist, 1);
    problem.SetParameterLowerBound(&inv_dist, 0, SVOConfig::inv_dist_min_value);
  }

  AddMarginalizationPriorIfAvailable(problem,
                                     *marginalization_prior_,
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

    const FrameCamId       frame_cam_id0 = mp->GetHostFrameCamId();
    std::shared_ptr<Frame> frame0 = window->GetFrame(frame_cam_id0.frame_id);
    const Sophus::SE3d&    T_b_c0 = frame0->GetTbc(frame_cam_id0.cam_id);
    double*                pose_param0 =
      pose_params[frame_id_to_index[frame_cam_id0.frame_id]].data();

    const auto& observations = mp->GetObservation();
    double* bearing_param  = bearing_params[mp_id_to_index[mp->GetId()]].data();
    double* inv_dist_param = &inv_dist_params[mp_id_to_index[mp->GetId()]];

    const auto host_obs_it = observations.find(frame_cam_id0);
    if (host_obs_it != observations.end()) {
      ceres::CostFunction* host_bearing_prior_cost =
        new BearingPriorCost(host_obs_it->second,
                             SVOConfig::bearing_cost_scale);
      problem.AddResidualBlock(host_bearing_prior_cost, nullptr, bearing_param);
    }

    for (const auto& [frame_cam_id1, bearing] : observations) {
      if (frame_cam_id0 == frame_cam_id1) {
        continue;
      }

      std::shared_ptr<Frame> frame1 = window->GetFrame(frame_cam_id1.frame_id);
      double*                pose_param1 =
        pose_params[frame_id_to_index[frame_cam_id1.frame_id]].data();
      const Sophus::SE3d& T_b_c1 = frame1->GetTbc(frame_cam_id1.cam_id);

      ceres::LossFunction* loss =
        new ceres::HuberLoss(SVOConfig::bearing_huber_const);
      if (frame_cam_id0.frame_id == frame_cam_id1.frame_id) {
        // Stereo within the same frame: use a single pose parameter block.
        ceres::CostFunction* cost =
          new BearingStereoCost(bearing,
                                T_b_c1,
                                T_b_c0,
                                SVOConfig::bearing_cost_scale);
        problem.AddResidualBlock(cost, loss, bearing_param, inv_dist_param);
      }
      else {
        ceres::CostFunction* cost =
          new BearingCost(bearing,
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
    const auto it = frame_id_to_index.find(frame_id);
    frame->SetTwb(
      SE3BoxplusManifold::FromParams(pose_params[it->second].data()));
  }

  for (const auto& [mp_id, idx] : mp_id_to_index) {
    std::shared_ptr<MapPoint> mp = window->GetMapPoint(mp_id);
    Eigen::Vector3d           b  = bearing_params[idx];
    if (b.norm() > 0.0) {
      b.normalize();
    }
    mp->GetBearing() = b;
    mp->SetInvDist(
      std::max(inv_dist_params[idx], SVOConfig::inv_dist_min_value));
  }
}

void VOEstimator::Marginalize(SlidingWindow*     window,
                              std::set<uint64_t> marginal_kf_ids) {
  if (!window || marginal_kf_ids.empty()) {
    return;
  }

  const auto& prev_frame_ids = marginalization_prior_->frame_ids_;

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
  SE3BoxplusManifold* box_plus_manifold = new SE3BoxplusManifold();

  pose_params.reserve(marginal_kf_ids.size() + remain_frame_ids.size());
  for (const auto& frame_id : marginal_kf_ids) {
    std::shared_ptr<Frame> frame = window->GetFrame(frame_id);
    frame_id_to_index[frame_id]  = pose_params.size();
    pose_params.push_back(SE3BoxplusManifold::ToParams(frame->GetTwb()));
    problem.AddParameterBlock(pose_params.back().data(),
                              kPoseSize,
                              box_plus_manifold);
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

  BearingTangentManifold* bearing_manifold = new BearingTangentManifold();
  for (auto& bearing : bearing_params) {
    problem.AddParameterBlock(bearing.data(), kBearingSize, bearing_manifold);
  }

  for (auto& inv_dist : inv_dist_params) {
    problem.AddParameterBlock(&inv_dist, 1);
    problem.SetParameterLowerBound(&inv_dist, 0, SVOConfig::inv_dist_min_value);
  }

  for (const auto& frame_id : remain_frame_ids) {
    std::shared_ptr<Frame> frame = window->GetFrame(frame_id);
    frame_id_to_index[frame_id]  = pose_params.size();
    pose_params.push_back(SE3BoxplusManifold::ToParams(frame->GetTwb()));
    problem.AddParameterBlock(pose_params.back().data(),
                              kPoseSize,
                              box_plus_manifold);
  }

  AddMarginalizationPriorIfAvailable(problem,
                                     *marginalization_prior_,
                                     frame_id_to_index,
                                     pose_params);

  for (size_t i = 0; i < marginal_map_points.size(); i++) {
    std::shared_ptr<MapPoint> mp = marginal_map_points[i];
    if (mp->GetStatus() < MapPoint::Status::TRACKING) {
      continue;
    }
    const FrameCamId       frame_cam_id0 = mp->GetHostFrameCamId();
    std::shared_ptr<Frame> frame0 = window->GetFrame(frame_cam_id0.frame_id);

    const Sophus::SE3d& T_b_c0 = frame0->GetTbc(frame_cam_id0.cam_id);
    double*             pose_param0 =
      pose_params[frame_id_to_index[frame_cam_id0.frame_id]].data();

    const auto& observations   = mp->GetObservation();
    double*     bearing_param  = bearing_params[i].data();
    double*     inv_dist_param = &inv_dist_params[i];

    auto* bearing_loss = new ceres::HuberLoss(SVOConfig::bearing_huber_const);

    const auto host_obs_it = observations.find(frame_cam_id0);
    if (host_obs_it != observations.end()) {
      ceres::CostFunction* host_bearing_prior_cost =
        new BearingPriorCost(host_obs_it->second,
                             SVOConfig::bearing_cost_scale);
      problem.AddResidualBlock(host_bearing_prior_cost,
                               bearing_loss,
                               bearing_param);
    }

    for (const auto& [frame_cam_id1, bearing] : observations) {
      if (frame_cam_id0 == frame_cam_id1) {
        continue;
      }

      std::shared_ptr<Frame> frame1 = window->GetFrame(frame_cam_id1.frame_id);
      double*                pose_param1 =
        pose_params[frame_id_to_index[frame_cam_id1.frame_id]].data();
      const Sophus::SE3d& T_b_c1 = frame1->GetTbc(frame_cam_id1.cam_id);

      if (frame_cam_id0.frame_id == frame_cam_id1.frame_id) {
        // Stereo within the same frame: use a single pose parameter block.
        ceres::CostFunction* cost =
          new BearingStereoCost(bearing,
                                T_b_c1,
                                T_b_c0,
                                SVOConfig::bearing_cost_scale);
        problem.AddResidualBlock(cost,
                                 bearing_loss,
                                 bearing_param,
                                 inv_dist_param);
      }
      else {
        ceres::CostFunction* cost =
          new BearingCost(bearing,
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
  // Evaluate Jacobian
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

  const Eigen::Index r =
    static_cast<Eigen::Index>(remain_frame_ids.size() * kPoseSize);
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

  Eigen::VectorXd x0(r);
  Eigen::Index    offset = 0;
  for (const uint64_t& frame_id : remain_frame_ids) {
    size_t idx                    = frame_id_to_index[frame_id];
    x0.segment<kPoseSize>(offset) = pose_params[idx];
    offset += kPoseSize;
  }
  Statistics::startTimer("marginalize saes2");

  marginalization_prior_->frame_ids_          = remain_frame_ids;
  marginalization_prior_->preintegration_ids_ = {};
  marginalization_prior_->block_sizes_ =
    std::vector<int>(remain_frame_ids.size(), kPoseSize);
  marginalization_prior_->x0_ = x0;

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes_prior(A);
  if (saes_prior.info() != Eigen::Success) {
    LogE("Marginalize prior decompose fail");
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
    marginalization_prior_->r_ = inv_sqrt_vals.asDiagonal()
                                 * eig_vecs.transpose() * b;
  }
  Statistics::stopTimer("marginalize saes2");

  LogD("ceres_J : {} x {} ", ceres_J.num_rows, ceres_J.num_cols);
  LogD("margin frame size: {}", marginal_kf_ids.size());
  LogD("total frame: {}", marginal_kf_ids.size() + remain_frame_ids.size());
  LogD("margin map poitns size: {}", marginal_map_points.size());
  LogD("Hessian : {} x {} ", H.rows(), H.cols());
}

}  // namespace omni_slam
