#pragma once

#include <memory>

namespace omni_slam {
class Frame;
class SlidingWindow;

class VOEstimator {
public:
  /**
   * @brief
   * @param frame
   * @param window
   */
  static void OptimizeSingleFrame(std::shared_ptr<Frame> frame, SlidingWindow* window);

  void OptimizeWindow(SlidingWindow* window);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
};
}  // namespace omni_slam
