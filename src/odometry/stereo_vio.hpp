#pragma once

#include <cstddef>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "odometry/odometry.hpp"
#include "odometry/odometry_result.hpp"
#include "odometry/stereo_vo.hpp"
#include "utils/types.hpp"

namespace omni_slam {

class StereoVIO : public Odometry {
public:
  StereoVIO();
  ~StereoVIO() override;

  bool Setup(const std::string& config_path) override;
  void Run() override;
  void Shutdown() override;

  void OnCameraFrame(int64_t                             timestamp_ns,
                     const std::vector<cv::Mat>&         images,
                     const std::vector<CameraParameter>& camera_parameters);
  void OnImuData(const ImuData& imu_data);

  bool   FetchResult(OdometryResult& out);
  size_t GetBufferedImuCount() const;

private:
  StereoVO            stereo_vo_;
  mutable std::mutex  imu_mutex_;
  std::deque<ImuData> imu_buffer_;
};

}  // namespace omni_slam
