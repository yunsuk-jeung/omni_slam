#pragma once

#include <ceres/sized_cost_function.h>

namespace omni_slam {
class PoseOnlyReprojectionCost : ceres::SizedCostFunction<2, 3, 3> {};
}  // namespace omni_slam