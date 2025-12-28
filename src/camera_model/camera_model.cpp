#include "camera_model/camera_model.hpp"

#include "camera_model/pinhole_radtan.hpp"

namespace omni_slam {

std::unique_ptr<CameraModelBase> CameraModelFactory::Create(
  const CameraParameter& params) {
  std::unique_ptr<CameraModelBase> camera;

  switch (params.model) {
  case CameraModel::PINHOLE_RAD_TAN:
    camera = std::make_unique<PinholeRadialTangential>(params);
    break;
  default:
    return nullptr;
  }
  return camera;
}

}  // namespace omni_slam
