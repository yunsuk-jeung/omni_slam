#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <thread>

#include <opencv2/imgcodecs.hpp>

#include "device/vio_loader.hpp"
#include "device/device_interface.hpp"

namespace omni_slam {

class DatasetSimulator final : public DeviceInterface {
public:
  explicit DatasetSimulator(VioLoader& loader, double speed = 1.0, bool realtime = true)
    : loader_(loader)
    , speed_(speed)
    , realtime_(realtime) {}

  ~DatasetSimulator() override { Stop(); }

  void Start() override {
    Stop();
    terminate_ = false;
    loader_.Reset();
    image_thread_ = std::thread(&DatasetSimulator::FeedImages, this);
    imu_thread_   = std::thread(&DatasetSimulator::FeedImu, this);
  }

  void Stop() override {
    terminate_ = true;
    if (image_thread_.joinable()) {
      image_thread_.join();
    }
    if (imu_thread_.joinable()) {
      imu_thread_.join();
    }
  }

  void SetCameraCallback(CameraCallback callback) override {
    camera_callback_ = std::move(callback);
  }

  void SetImuCallback(ImuCallback callback) override {
    imu_callback_ = std::move(callback);
  }

  void SetRealtime(bool realtime) { realtime_ = realtime; }
  void SetSpeed(double speed) { speed_ = speed; }

private:
  void FeedImages() {
    if (!camera_callback_) {
      return;
    }

    bool    initialized = false;
    int64_t start_ts    = 0;
    auto    start_time  = std::chrono::steady_clock::now();

    while (!terminate_ && loader_.HasCameraData()) {
      CameraFrame frame = loader_.GetNextCameraFrame();
      if (!initialized) {
        start_ts    = frame.t_ns;
        start_time  = std::chrono::steady_clock::now();
        initialized = true;
      }

      if (realtime_) {
        const auto dt_ns   = frame.t_ns - start_ts;
        const auto wait_ns = static_cast<int64_t>(static_cast<double>(dt_ns) / speed_);
        const auto target  = start_time + std::chrono::nanoseconds(wait_ns);
        std::this_thread::sleep_until(target);
      }

      if (terminate_) {
        break;
      }

      std::array<cv::Mat, 2> images;
      if (!frame.cam0_image_path.empty()) {
        images[0] = cv::imread(frame.cam0_image_path, cv::IMREAD_COLOR);
      }
      if (!frame.cam1_image_path.empty()) {
        images[1] = cv::imread(frame.cam1_image_path, cv::IMREAD_COLOR);
      }

      camera_callback_(images);
    }
  }

  void FeedImu() {
    if (!imu_callback_) {
      return;
    }

    bool    initialized = false;
    int64_t start_ts    = 0;
    auto    start_time  = std::chrono::steady_clock::now();

    while (!terminate_ && loader_.HasImuData()) {
      ImuData imu = loader_.GetNextImuMeasurement();
      if (!initialized) {
        start_ts    = imu.t_ns;
        start_time  = std::chrono::steady_clock::now();
        initialized = true;
      }

      if (realtime_) {
        const auto dt_ns   = imu.t_ns - start_ts;
        const auto wait_ns = static_cast<int64_t>(static_cast<double>(dt_ns) / speed_);
        const auto target  = start_time + std::chrono::nanoseconds(wait_ns);
        std::this_thread::sleep_until(target);
      }

      if (terminate_) {
        break;
      }
      imu_callback_(imu);
    }
  }

  VioLoader&        loader_;
  std::atomic<bool> terminate_{false};
  std::thread       image_thread_;
  std::thread       imu_thread_;

  CameraCallback camera_callback_;
  ImuCallback    imu_callback_;

  double speed_    = 1.0;
  bool   realtime_ = true;
};

}  // namespace omni_slam
