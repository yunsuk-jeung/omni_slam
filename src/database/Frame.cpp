#include <atomic>
#include <utility>

#include "feature_tracking/tracking_result.hpp"
#include "database/Frame.hpp"
#include "config/svo_config.hpp"

namespace omni_slam {
namespace {
std::atomic<size_t> g_frame_id{0};
}  // namespace

Frame::Frame(const std::vector<cv::Mat>&         images,
             const std::vector<CameraParameter>& camera_parameters)
  : id_(g_frame_id.fetch_add(1, std::memory_order_relaxed))
  , images_(images)
  , image_pyramids_(images.size())
  , kCamNum{images.size()} {
  cams_.clear();
  cams_.reserve(images.size());

  for (const auto& params : camera_parameters) {
    auto camera = CameraModelFactory::Create(params);
    if (camera) {
      cams_.push_back(std::move(camera));
    }
  }
  tracking_result_ = std::make_unique<TrackingResult>(images_.size());

  T_bcs_.resize(images.size());
  for (size_t i = 0; i < images.size(); ++i) {
    if (i < SVOConfig::camera_T_b_c.size()) {
      T_bcs_[i] = SVOConfig::camera_T_b_c[i];
    }
  }
}

Frame::~Frame() {
  tracking_result_.reset();
}

}  // namespace omni_slam
