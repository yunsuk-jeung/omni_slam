# Suspected bugs — audit status

Found during the readability analysis. Updated as fixes land. Numbering follows
the original analysis order.

## Resolved

| # | Location | Issue | Resolution |
|---|----------|-------|------------|
| 1 | utils/logger.hpp | `init()` check-then-act race on `logger_` | `std::call_once` guard (commit `0e18300`) |
| 2 | utils/logger.hpp | `std::localtime` not reentrant | `localtime_r` + call_once (commit `0e18300`) |
| 3 | config/svo_config.cpp | duplicate `bearing_huber_const` clamp (no-op) | duplicate removed in config refactor (`6288fc3`); see note under Open for the unverifiable "missing clamp" theory |
| 4 | config/svo_config.cpp | dead misspelled `beraing_cost_scale` fallback key | removed — key never appears in real configs (`c1fe91e`) |
| 5 | camera_model/camera_model.cpp | unsupported model → silent nullptr | log the offending model before returning nullptr (`375bc83`) |
| 6 | database/frame.cpp | null camera skipped → `cams_` index misalignment | push unconditionally to keep `cams_[i]` aligned (`5f11ca5`) |
| 7 | database/frame.hpp | ctor mem-init order vs declaration order (-Wreorder) | init-list reordered to declaration order in database refactor (`1c3eeda`) |
| 8 | database/frame.hpp | `add_observation(size_t)` vs `remove_observation(uint64_t)` | unified to `uint64_t`; observation maps keyed by `uint64_t` (`bce3716`) |
| 20 | omni_slam/types.hpp | `FrameCamId` ctor `size_t frame_id` → `uint64_t` member | ctor param made `uint64_t` (`bce3716`) |
| 11 | device/dataset_simulator.hpp | `sleep_until` not interruptible → `stop()` can block | condition_variable wait keyed on `terminate_`; `stop()` notifies (`277a050`) |
| 13 | optimizer/vio_estimator.cpp | `OMNI_ASSERT_MESSAGE(true, …)` vacuous, no diagnostic | replaced with `LogW` (abort avoided, recovery kept) (`1d9217f`) |
| 14 | optimizer/vio_estimator.cpp | `marginalize saes` timer unbalanced on failure path | added `stop_timer` on the early return (`1d9217f`) |
| 16 | optimizer/vo_estimator.cpp | stale "no robust loss" comment | removed in optimizer refactor (`0fd352e`) |
| 17 | odometry/stereo_vio.cpp | "use first imu" comment but uses `.back()` | comment corrected in odometry refactor (`b30d3fb`) |
| 18 | odometry/stereo_vo.cpp | `Logger::info` one `{}` for two args (count dropped) | completed the format string (`1d9217f`) |
| 19 | omni_slam/types.hpp | missing `<array>`/`<vector>`/`<cstdint>`/`<cstddef>` | added direct includes (`1d9217f`) |

## Open — needs a decision (behavior change / author intent)

### dataset_simulator (demo/sim tool)
- **#10 — device/dataset_simulator.hpp** — `wait_ns = (dt_ns / speed_)` with no
  guard for `speed_ <= 0`; zero produces inf/NaN and the cast to `int64_t` is
  UB. Guard `speed_` (both feed loops).

### Numerically sensitive (core estimator — review carefully before touching)
- **#12 — optimizer/parameterization.hpp:155** — `BearingTangentManifold::Minus`
  small-angle branch (`sin_theta < eps`) only handles θ≈0, not the antipodal
  θ≈π case (sinθ→0 there too), returning a wrong-magnitude tangent. Likely
  benign in practice (bearings rarely flip ~180°) but a real correctness gap.
- **#15 — optimizer/vio_estimator.cpp / vo_estimator.cpp** — `optimize_window`
  skips a map point with non-finite / non-positive `inv_dist`, but the
  structurally identical loop in `marginalize()` has no such check, so a
  degenerate inverse depth can be folded into the marginalization prior. Verify
  whether the asymmetry is intentional; if not, mirror the guard.
- **#3 (residual) — config/svo_config.cpp** — the removed duplicate clamp *might*
  have been a copy-paste slip that meant to clamp a different field. No field is
  obviously missing a clamp, and the intended target can't be inferred — needs
  the original author.

## Open — speculative (confirm intent; do not auto-fix)
- **#9 — feature_tracking/optical_flow.cpp:60** — squared FB error compared
  against `optical_flow_dist_threshold`. If the config value is meant as a pixel
  distance, the effective threshold is `sqrt(5)≈2.24px`, not 5px. Confirm units.
- **#21 — apps/run_svo.cpp:189** — no drain phase after the loader runs dry; if
  StereoVO lags, the tail of the run can be dropped from the visualization.
