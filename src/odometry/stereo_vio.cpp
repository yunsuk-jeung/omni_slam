#include "odometry/stereo_vio.hpp"

namespace omni_slam {

StereoVIO::StereoVIO() = default;
StereoVIO::~StereoVIO() = default;

bool StereoVIO::Setup(const std::string& config_path) {
  return stereo_vo_.Setup(config_path);
}

void StereoVIO::Run() {
  stereo_vo_.Run();
}

void StereoVIO::Shutdown() {
  stereo_vo_.Shutdown();
}

void StereoVIO::OnCameraFrame(int64_t                             timestamp_ns,
                              const std::vector<cv::Mat>&         images,
                              const std::vector<CameraParameter>& camera_parameters) {
  stereo_vo_.OnCameraFrame(timestamp_ns, images, camera_parameters);
}

void StereoVIO::OnImuData(const ImuData& imu_data) {
  std::lock_guard<std::mutex> lock(imu_mutex_);
  imu_buffer_.push_back(imu_data);

  constexpr size_t kMaxImuBufferSize = 5000;
  while (imu_buffer_.size() > kMaxImuBufferSize) {
    imu_buffer_.pop_front();
  }
}

bool StereoVIO::FetchResult(OdometryResult& out) {
  return stereo_vo_.FetchResult(out);
}

size_t StereoVIO::GetBufferedImuCount() const {
  std::lock_guard<std::mutex> lock(imu_mutex_);
  return imu_buffer_.size();
}

}  // namespace omni_slam
