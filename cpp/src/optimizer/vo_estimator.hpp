#pragma once

#include <cstdint>
#include <memory>
#include <set>
#include <vector>

#include <Eigen/Core>

namespace omni_slam {
class Frame;
class SlidingWindow;
struct MarginalizationPrior;
class VOEstimator {
 public:
  /**
   * @brief
   * @param frame
   * @param window
   */
  static void optimize_single_frame(std::shared_ptr<Frame> frames,
                                    SlidingWindow*         window);

  VOEstimator();
  ~VOEstimator();

  void optimize_window(SlidingWindow* window);

  void marginalize(SlidingWindow*     window,
                   std::set<uint64_t> marignal_keyframes);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  void clear_prior();

 private:
  std::unique_ptr<MarginalizationPrior> marginalization_prior_;
};
}  // namespace omni_slam
