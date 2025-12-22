#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "device/dataset_types.hpp"
#include "device/vio_loader.hpp"

namespace omni_slam {

class EurocLoader : public VioLoader {
public:
  EurocLoader()           = default;
  ~EurocLoader() override = default;

  // VioLoader interface implementation
  bool Initialize(const std::string& dataset_path) override;
  void Reset() override;

  bool        HasCameraData() const override;
  CameraFrame GetNextCameraFrame() override;
  size_t      GetCameraFrameCount() const override;

  bool    HasImuData() const override;
  ImuData GetNextImuMeasurement() override;
  size_t  GetImuMeasurementCount() const override;

  bool            HasGroundTruthData() const override;
  GroundTruthPose GetNextGroundTruthPose() override;
  size_t          GetGroundTruthPoseCount() const override;

  int64_t GetStartTimestampNs() const override;
  int64_t GetEndTimestampNs() const override;

  std::string GetDatasetName() const override;
  bool        IsStereo() const override;

private:
  // Helper methods
  bool ParseCameraCsv(const std::string& csv_path, int cam_id);
  bool ParseImuCsv(const std::string& csv_path);
  bool ParseGroundTruthCsv(const std::string& csv_path);

  void ComputeTimestampRange();
  bool ValidateDatasetStructure();

  // Internal data storage (metadata only, no actual images)
  std::string dataset_path_;
  std::string dataset_name_;

  // Camera data (timestamp, filename pairs)
  std::vector<std::pair<int64_t, std::string>> cam0_data_;
  std::vector<std::pair<int64_t, std::string>> cam1_data_;
  size_t                                       camera_index_ = 0;

  // IMU data
  std::vector<ImuData> imu_data_;
  size_t               imu_index_ = 0;

  // Ground truth data
  std::vector<GroundTruthPose> ground_truth_data_;
  size_t                       ground_truth_index_ = 0;

  // Metadata
  int64_t start_timestamp_ns_ = 0;
  int64_t end_timestamp_ns_   = 0;
  bool    is_stereo_          = false;
  bool    initialized_        = false;
};

}  // namespace omni_slam
