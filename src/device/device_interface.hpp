#pragma once

#include <functional>

#include "core/types.hpp"
#include "device/dataset_types.hpp"

namespace omni_slam {

class DeviceInterface {
public:
  using CameraCallback = std::function<void(const CameraFrame&)>;
  using ImuCallback    = std::function<void(const ImuData&)>;

  virtual ~DeviceInterface() = default;

  virtual void Start() = 0;
  virtual void Stop()  = 0;

  virtual void SetCameraCallback(CameraCallback callback) = 0;
  virtual void SetImuCallback(ImuCallback callback)       = 0;
};

}  // namespace omni_slam
