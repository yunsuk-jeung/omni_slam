#pragma once
#include <memory>
#include <vector>
#include <Eigen/Dense>

#include "utils/types.hpp"
#include "camera_model/camera_model.hpp"

namespace omni_slam {
class TrackingResult;
class CameraModelBase;
class Frame {
public:
  Frame();
  Frame(const std::vector<CameraParameter>& camera_parameters);
  ~Frame();

  void SetCameraParams(const std::vector<CameraParameter>& camera_parameters);

private:
  std::vector<std::unique_ptr<CameraModelBase>> cams_;
  std::vector<Eigen::Matrix4d>                  T_bcs_;
  std::unique_ptr<TrackingResult>               tracking_result_;
};

}  // namespace omni_slam
