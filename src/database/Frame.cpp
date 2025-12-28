#include "optical_flow/optical_flow.hpp"
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

void Frame::SetCameraParams(const std::vector<CameraParameter>& camera_parameters) {
  cams_.clear();
  cams_.reserve(camera_parameters.size());

  for (const auto& params : camera_parameters) {
    auto camera = CameraModelFactory::Create(params);
    if (camera) {
      cams_.push_back(std::move(camera));
    }
  }
}

}  // namespace omni_slam
