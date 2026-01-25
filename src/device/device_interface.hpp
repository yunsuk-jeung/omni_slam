#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <vector>

#include <opencv2/core.hpp>

#include "utils/types.hpp"
#include "device/dataset_types.hpp"

namespace omni_slam {

class DeviceInterface {
public:
  using CameraCallback = std::function<void(int64_t                             timestamp_ns,
                                            const std::vector<cv::Mat>&         images,
                                            const std::vector<CameraParameter>& camera_parameters)>;
  using ImuCallback    = std::function<void(const ImuData&)>;

  virtual ~DeviceInterface() = default;

  virtual void Start() = 0;
  virtual void Stop()  = 0;

  virtual void SetCameraCallback(CameraCallback callback) = 0;
  virtual void SetImuCallback(ImuCallback callback)       = 0;
};

}  // namespace omni_slam
