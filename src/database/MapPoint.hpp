#pragma once

#include <Eigen/Dense>

namespace omni_slam {
class MapPoint {
public:

private:
  size_t          id_;
  Eigen::Vector3d xyz;
};
}  // namespace omni_slam