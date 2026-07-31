#pragma once

#include <cstdint>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>
#include <sophus/se3.hpp>

#include "mapper/nfr_factors.hpp"

namespace omni_slam {

// One marginalization prior handed over from the VIO backend: a joint,
// pose-only Gaussian over several keyframes (docs/nfr_mapper.md §1-§2).
//
// Blocks in H/b are 6-DoF per keyframe and ordered to match keyframe_ids
// (block k spans rows/cols [6k, 6k+6)). T_w_b_lin are the linearization points
// the prior was built at.
struct MargPrior {
  std::vector<uint64_t>     keyframe_ids;       // block order in H / b
  std::vector<Sophus::SE3d> T_w_b_lin;          // linearization point per block
  Eigen::MatrixXd           H;                  // 6N x 6N information matrix
  Eigen::VectorXd           b;                  // 6N information vector
  std::vector<uint64_t>     keyframes_to_marg;  // anchor candidate(s)

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Non-linear Factor Recovery optimizer. Accumulates marginalization priors
// from the VIO, recovers re-linearizable factors from each, and jointly
// optimizes the global keyframe trajectory (docs/nfr_mapper.md §3, §6).
//
// This is a skeleton: the pipeline stages are stubbed. Fill them in step by
// step following the referenced doc sections.
class NfrOptimizer {
 public:
  NfrOptimizer() = default;

  // Ingest one prior: recover factors and register keyframe poses.
  // docs/nfr_mapper.md §3 (addMargData).
  void add_marg_data(const MargPrior& prior);

  // Jointly optimize registered poses under the accumulated factors (and, once
  // wired, loop-closure visual constraints). docs/nfr_mapper.md §6.
  void optimize(int max_iterations = 10);

  const std::unordered_map<uint64_t, Sophus::SE3d>& frame_poses() const {
    return frame_poses_;
  }

 private:
  // Invert the pose-only prior to Σ = H^-1 and recover one RollPitchFactor on
  // the anchor plus RelPoseFactors anchor<->others. Returns false if the prior
  // is not full-rank. docs/nfr_mapper.md §3-B.
  bool extract_nonlinear_factors(const MargPrior& prior);

  std::unordered_map<uint64_t, Sophus::SE3d> frame_poses_;
  std::vector<RollPitchFactor>               roll_pitch_factors_;
  std::vector<RelPoseFactor>                 rel_pose_factors_;
};

}  // namespace omni_slam
