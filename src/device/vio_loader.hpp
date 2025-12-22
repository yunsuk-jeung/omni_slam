#pragma once

#include <memory>
#include "core/types.hpp"
#include "device/dataset_types.hpp"

namespace omni_slam {

class VioLoader {
public:
  virtual ~VioLoader() = default;

  // Initialize the loader with dataset path
  // Returns true on success, false on error
  virtual bool Initialize(const std::string& dataset_path) = 0;

  // Reset all iterators to beginning
  virtual void Reset() = 0;

  // Camera data access
  virtual bool        HasCameraData() const       = 0;
  virtual CameraFrame GetNextCameraFrame()        = 0;
  virtual size_t      GetCameraFrameCount() const = 0;

  // IMU data access
  virtual bool    HasImuData() const             = 0;
  virtual ImuData GetNextImuMeasurement()        = 0;
  virtual size_t  GetImuMeasurementCount() const = 0;

  // Ground truth data access (optional - may not exist in all datasets)
  virtual bool            HasGroundTruthData() const      = 0;
  virtual GroundTruthPose GetNextGroundTruthPose()        = 0;
  virtual size_t          GetGroundTruthPoseCount() const = 0;

  // Get timestamp range for synchronization purposes
  virtual int64_t GetStartTimestampNs() const = 0;
  virtual int64_t GetEndTimestampNs() const   = 0;

  // Dataset metadata
  virtual std::string GetDatasetName() const = 0;
  virtual bool        IsStereo() const       = 0;
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
  static std::unique_ptr<VioLoader> CreateLoader(const std::string& dataset_path,
                                                 DatasetType type = DatasetType::AUTO);

private:
  // Auto-detect dataset type from directory structure
  static DatasetType DetectDatasetType(const std::string& dataset_path);

  // Check if path contains EuRoC dataset structure
  static bool IsEurocDataset(const std::string& dataset_path);
};

}  // namespace omni_slam
