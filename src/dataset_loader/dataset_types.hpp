#pragma once

#include <cstdint>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace omni_slam {

// Represents a single camera frame with timestamp and image path(s)
struct CameraFrame {
  int64_t     timestamp_ns;     // Nanosecond timestamp
  std::string cam0_image_path;  // Path to left/cam0 image
  std::string cam1_image_path;  // Path to right/cam1 image (empty if mono)
};

// Represents a single IMU measurement
struct ImuMeasurement {
  int64_t         timestamp_ns;   // Nanosecond timestamp
  Eigen::Vector3d gyroscope;      // Angular velocity (rad/s): [wx, wy, wz]
  Eigen::Vector3d accelerometer;  // Linear acceleration (m/s^2): [ax, ay, az]
};

// Represents a single ground truth pose
struct GroundTruthPose {
  int64_t            timestamp_ns;  // Nanosecond timestamp
  Eigen::Vector3d    position;      // Position: [px, py, pz]
  Eigen::Quaterniond orientation;   // Orientation quaternion: [qw, qx, qy, qz]
};

}  // namespace omni_slam
