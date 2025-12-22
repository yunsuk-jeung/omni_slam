#pragma once

#include <memory>

#include <Eigen/Dense>

namespace omni_slam {
struct ImuData {
  using Ptr = std::shared_ptr<ImuData>;

  int64_t         t_ns;  ///< timestamp in nanoseconds
  Eigen::Vector3d acc;   ///< Accelerometer measurement
  Eigen::Vector3d gyr;   ///< Gyroscope measurement

  /// @brief Default constructor with zero measurements.
  ImuData() {
    t_ns = 0;
    acc.setZero();
    gyr.setZero();
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace omni_slam
