#pragma once

#include <memory>

#include <Eigen/Dense>

namespace omni_slam {
static constexpr size_t kCamLeft  = 0;
static constexpr size_t kCamRight = 1;

enum class CameraModel {
  PINHOLE_RAD_TAN = 0,
};

struct CameraParameter {
  CameraModel           model;
  std::array<double, 4> intrinsics;
  std::vector<double>   distortions;
  int                   w;
  int                   h;
};

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
