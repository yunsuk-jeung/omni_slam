#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "camera_model/camera_model.hpp"
#include "config/svo_config.hpp"
#include "device/device_interface.hpp"
#include "device/vio_loader.hpp"

namespace omni_slam {

class DatasetSimulator final : public DeviceInterface {
 public:
  explicit DatasetSimulator(VioLoader& loader,
                            double     speed    = 1.0,
                            bool       realtime = true)
    : loader_(loader)
    , speed_(speed)
    , realtime_(realtime) {}

  ~DatasetSimulator() override { stop(); }

  void start() override {
    stop();
    terminate_ = false;
    loader_.reset();
    image_thread_ = std::thread(&DatasetSimulator::feed_images, this);
    imu_thread_   = std::thread(&DatasetSimulator::feed_imu, this);
  }

  void stop() override {
    terminate_ = true;
    if (image_thread_.joinable()) {
      image_thread_.join();
    }
    if (imu_thread_.joinable()) {
      imu_thread_.join();
    }
  }

  void camera_callback(CameraCallback callback) override {
    camera_callback_ = std::move(callback);
  }

  void imu_callback(ImuCallback callback) override {
    imu_callback_ = std::move(callback);
  }

  void realtime(bool realtime) { realtime_ = realtime; }
  void speed(double speed) { speed_ = speed; }

 private:
  void feed_images() {
    if (!camera_callback_) {
      return;
    }

    bool    initialized = false;
    int64_t start_ts    = 0;
    auto    start_time  = std::chrono::steady_clock::now();

    while (!terminate_ && loader_.has_camera_data()) {
      CameraFrame frame = loader_.get_next_camera_frame();
      if (!initialized) {
        start_ts    = frame.t_ns;
        start_time  = std::chrono::steady_clock::now();
        initialized = true;
      }

      if (realtime_) {
        wait_for_playback_time(start_time, start_ts, frame.t_ns);
      }

      if (terminate_) {
        break;
      }

      std::vector<cv::Mat> images;
      if (!frame.cam0_image_path.empty()) {
        images.push_back(cv::imread(frame.cam0_image_path, cv::IMREAD_COLOR));
      }
      if (!frame.cam1_image_path.empty()) {
        images.push_back(cv::imread(frame.cam1_image_path, cv::IMREAD_COLOR));
      }

      const size_t                 cam_count = images.size();
      std::vector<CameraParameter> camera_parameters;
      camera_parameters.reserve(cam_count);

      for (size_t i = 0; i < cam_count; ++i) {
        camera_parameters.push_back(build_camera_parameter(i));
      }

      camera_callback_(frame.t_ns, images, camera_parameters);
    }
  }

  void feed_imu() {
    if (!imu_callback_) {
      return;
    }

    bool    initialized = false;
    int64_t start_ts    = 0;
    auto    start_time  = std::chrono::steady_clock::now();

    while (!terminate_ && loader_.has_imu_data()) {
      ImuData imu = loader_.get_next_imu_measurement();
      if (!initialized) {
        start_ts    = imu.t_ns;
        start_time  = std::chrono::steady_clock::now();
        initialized = true;
      }

      if (realtime_) {
        wait_for_playback_time(start_time, start_ts, imu.t_ns);
      }

      if (terminate_) {
        break;
      }
      imu_callback_(imu);
    }
  }

  // Sleeps until the recorded inter-frame delay has elapsed, scaled by speed_
  // to support faster/slower-than-recorded playback.
  void wait_for_playback_time(std::chrono::steady_clock::time_point start_time,
                              int64_t start_ts,
                              int64_t ts_ns) const {
    const auto dt_ns = ts_ns - start_ts;
    const auto wait_ns =
      static_cast<int64_t>(static_cast<double>(dt_ns) / speed_);
    const auto target = start_time + std::chrono::nanoseconds(wait_ns);
    std::this_thread::sleep_until(target);
  }

  CameraParameter build_camera_parameter(size_t i) const {
    constexpr size_t kIntrinsicsCount = 4;
    constexpr size_t kResolutionCount = 2;
    CameraParameter  params{};
    if (i < SVOConfig::camera_models.size()) {
      params.model = static_cast<CameraModel>(SVOConfig::camera_models[i]);
    }
    else {
      params.model = CameraModel::PINHOLE_RAD_TAN;
    }
    if (i < SVOConfig::camera_intrinsics.size()
        && SVOConfig::camera_intrinsics[i].size() >= kIntrinsicsCount) {
      const auto& intr  = SVOConfig::camera_intrinsics[i];
      params.intrinsics = {intr[0], intr[1], intr[2], intr[3]};
    }
    else {
      params.intrinsics = {0.0, 0.0, 0.0, 0.0};
    }
    if (i < SVOConfig::camera_distortions.size()) {
      params.distortions = SVOConfig::camera_distortions[i];
    }
    if (i < SVOConfig::camera_resolutions.size()
        && SVOConfig::camera_resolutions[i].size() >= kResolutionCount) {
      params.w = SVOConfig::camera_resolutions[i][0];
      params.h = SVOConfig::camera_resolutions[i][1];
    }
    else {
      params.w = 0;
      params.h = 0;
    }
    return params;
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
