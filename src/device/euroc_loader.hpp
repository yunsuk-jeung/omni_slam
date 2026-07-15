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
  bool setup(const std::string& dataset_path) override;
  void reset() override;

  bool        has_camera_data() const override;
  CameraFrame get_next_camera_frame() override;
  size_t      get_camera_frame_count() const override;

  bool    has_imu_data() const override;
  ImuData get_next_imu_measurement() override;
  size_t  get_imu_measurement_count() const override;

  bool            has_ground_truth_data() const override;
  GroundTruthPose get_next_ground_truth_pose() override;
  size_t          get_ground_truth_pose_count() const override;

  int64_t get_start_timestamp_ns() const override;
  int64_t get_end_timestamp_ns() const override;

  std::string get_dataset_name() const override;
  bool        is_stereo() const override;

 private:
  // Helper methods
  bool parse_camera_csv(const std::string& csv_path, int cam_id);
  bool parse_imu_csv(const std::string& csv_path);
  bool parse_ground_truth_csv(const std::string& csv_path);

  void compute_timestamp_range();
  bool validate_dataset_structure();

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
