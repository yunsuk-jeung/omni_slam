#pragma once

#include <functional>
#include <memory>

#include <Eigen/Dense>

namespace omni_slam {
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

struct FrameCamId {
  FrameCamId()
    : frame_id_(0)
    , cam_id_(0) {}

  FrameCamId(const size_t& frame_id, const size_t& cam_id)
    : frame_id_(frame_id)
    , cam_id_(cam_id) {}

  uint64_t frame_id_;
  size_t   cam_id_;
};
inline bool operator==(const FrameCamId& lhs, const FrameCamId& rhs) {
  return lhs.frame_id_ == rhs.frame_id_ && lhs.cam_id_ == rhs.cam_id_;
}
}  // namespace omni_slam

namespace std {
template <>
struct hash<omni_slam::FrameCamId> {
  size_t operator()(const omni_slam::FrameCamId& value) const noexcept {
    const size_t h1 = std::hash<uint64_t>{}(value.frame_id_);
    const size_t h2 = std::hash<size_t>{}(value.cam_id_);
    return h1 ^ (h2 << 1);
  }
};
}  // namespace std
