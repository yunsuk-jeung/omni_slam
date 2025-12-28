#include "optical_flow/stereo_optical_flow.hpp"
#include "camera_model/pinhole_radtan.hpp"
#include "database/Frame.hpp"

namespace omni_slam {
Frame::Frame() {}

Frame::Frame(const std::vector<CameraParameter>& camera_parameters) {
  SetCameraParams(camera_parameters);
}

Frame::~Frame() {
  tracking_result_.reset();
}

void Frame::SetCameraParams(const std::vector<CameraParameter>& camera_parameters) {}

}  // namespace omni_slam
