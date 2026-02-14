#pragma once

#include <cstdint>
#include <memory>
#include <set>

namespace omni_slam {
class Frame;
class SlidingWindow;
class Marginalizer;
class VOEstimator {
public:
  /**
   * @brief
   * @param frame
   * @param window
   */
  static void OptimizeSingleFrame(std::shared_ptr<Frame> frames, SlidingWindow* window);

  VOEstimator();
  ~VOEstimator();

  void OptimizeWindow(SlidingWindow* window);

  void Marginalize(SlidingWindow* window, std::set<uint64_t> marignal_keyframes);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  std::unique_ptr<Marginalizer> marginalizer_;
};
}  // namespace omni_slam
