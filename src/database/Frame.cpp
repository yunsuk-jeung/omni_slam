#include <atomic>
#include <utility>

#include "optical_flow/optical_flow.hpp"
#include "database/Frame.hpp"

namespace omni_slam {
namespace {
std::atomic<size_t> g_frame_id{0};
}  // namespace

Frame::Frame(const std::vector<cv::Mat>&         images,
             const std::vector<CameraParameter>& camera_parameters)
  : id_(g_frame_id.fetch_add(1, std::memory_order_relaxed))
  , images_(images)
  , image_pyramids_(images.size()) {
  cams_.clear();
  cams_.reserve(camera_parameters.size());

  for (const auto& params : camera_parameters) {
    auto camera = CameraModelFactory::Create(params);
    if (camera) {
      cams_.push_back(std::move(camera));
    }
  }
  tracking_result_ = std::make_unique<TrackingResult>(images_.size());
}

Frame::~Frame() {
  tracking_result_.reset();
}

}  // namespace omni_slam
