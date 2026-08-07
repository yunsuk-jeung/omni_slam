#include "mapper/nfr_optimizer.hpp"
#include "utils/logger.hpp"

namespace omni_slam {

void NfrOptimizer::add_marg_data(const MargPosePrior& prior) {
  // docs/nfr_mapper.md §3. Our priors arrive pose-only, so processMargData()'s
  // velocity/bias elimination (§3-A) is not needed here.
  if (!extract_nonlinear_factors(prior)) {
    Logger::warn("NfrOptimizer: dropped rank-deficient prior ({} keyframes)",
                 prior.keyframe_ids.size());
    return;
  }

  // Register each keyframe pose as the optimization's initial estimate.
  // docs/nfr_mapper.md §3-C.
  for (size_t i = 0; i < prior.keyframe_ids.size(); ++i) {
    frame_poses_[prior.keyframe_ids[i]] = prior.T_w_b_lin[i];
  }
}

bool NfrOptimizer::extract_nonlinear_factors(const MargPosePrior& prior) {
  // docs/nfr_mapper.md §3-B:
  //   Step 1: Σ = H^-1 (bail if not full-rank).
  //   Step 2: pick anchor keyframe from keyframes_to_marg.
  //   Step 3: recover RollPitchFactor  (J from roll_pitch_error, Σ_rp = J Σ
  //   J^T,
  //           Ω = Σ_rp^-1) -> roll_pitch_factors_.
  //   Step 4: recover RelPoseFactor for anchor<->each other keyframe
  //           (J from rel_pose_error, Ω = (J Σ J^T)^-1) -> rel_pose_factors_.
  // TODO(nfr): implement.
  (void)prior;
  return false;
}

void NfrOptimizer::optimize(int max_iterations) {
  // docs/nfr_mapper.md §6: build a least-squares problem over frame_poses_
  // with roll_pitch_factors_ + rel_pose_factors_ (and, later, loop-closure
  // reprojection residuals), then run LM / Ceres for max_iterations.
  // TODO(nfr): implement.
  (void)max_iterations;
}

}  // namespace omni_slam
