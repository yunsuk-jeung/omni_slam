#include "optical_flow/optical_flow.hpp"
#include "camera_model/pinhole_radtan.hpp"
#include "database/Frame.hpp"

namespace omni_slam {
Frame::Frame() {}

Frame::Frame(const std::vector<cv::Mat>&         images,
             const std::vector<CameraParameter>& camera_parameters)
  : images_(images)
  , image_pyramids_(images.size()) {
  SetCameraParams(camera_parameters);
}

Frame::~Frame() {
  tracking_result_.reset();
}

void Frame::SetCameraParams(const std::vector<CameraParameter>& camera_parameters) {}

}  // namespace omni_slam
