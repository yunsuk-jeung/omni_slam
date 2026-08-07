#pragma once

#include <cstdint>
#include <unordered_map>
#include <vector>

#include <sophus/se3.hpp>

#include "mapper/nfr_factors.hpp"
#include "optimizer/marg_pose_prior.hpp"  // MargPosePrior

namespace omni_slam {

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
  void add_marg_data(const MargPosePrior& prior);

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
  bool extract_nonlinear_factors(const MargPosePrior& prior);

  std::unordered_map<uint64_t, Sophus::SE3d> frame_poses_;
  std::vector<RollPitchFactor>               roll_pitch_factors_;
  std::vector<RelPoseFactor>                 rel_pose_factors_;
};

}  // namespace omni_slam
