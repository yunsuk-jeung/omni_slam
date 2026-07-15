#pragma once

#include <memory>

#include "device/dataset_types.hpp"
#include "utils/types.hpp"

namespace omni_slam {

class VioLoader {
 public:
  virtual ~VioLoader() = default;

  // Initialize the loader with dataset path
  // Returns true on success, false on error
  virtual bool setup(const std::string& dataset_path) = 0;

  // Reset all iterators to beginning
  virtual void reset() = 0;

  // Camera data access
  virtual bool        has_camera_data() const    = 0;
  virtual CameraFrame get_next_camera_frame()    = 0;
  virtual size_t      camera_frame_count() const = 0;

  // IMU data access
  virtual bool    has_imu_data() const          = 0;
  virtual ImuData get_next_imu_measurement()    = 0;
  virtual size_t  imu_measurement_count() const = 0;

  // Ground truth data access (optional - may not exist in all datasets)
  virtual bool            has_ground_truth_data() const   = 0;
  virtual GroundTruthPose get_next_ground_truth_pose()    = 0;
  virtual size_t          ground_truth_pose_count() const = 0;

  // Get timestamp range for synchronization purposes
  virtual int64_t start_timestamp_ns() const = 0;
  virtual int64_t end_timestamp_ns() const   = 0;

  // Dataset metadata
  virtual std::string dataset_name() const = 0;
  virtual bool        is_stereo() const    = 0;
};

// Supported dataset types
enum class DatasetType {
  AUTO,   // Auto-detect dataset type
  EUROC,  // EuRoC MAV dataset
  // Future dataset types can be added here:
  // KITTI,
  // TUM,
  // etc.
};

class VIOLoaderFactory {
 public:
  // Create a dataset loader based on dataset type
  // If type is AUTO, the factory will attempt to auto-detect the dataset type
  // Returns nullptr if the dataset type cannot be determined or created
  static std::unique_ptr<VioLoader> create_loader(
    const std::string& dataset_path,
    DatasetType        type = DatasetType::AUTO);

 private:
  // Auto-detect dataset type from directory structure
  static DatasetType detect_dataset_type(const std::string& dataset_path);

  // Check if path contains EuRoC dataset structure
  static bool is_euroc_dataset(const std::string& dataset_path);
};

}  // namespace omni_slam
