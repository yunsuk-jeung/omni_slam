#include <atomic>
#include <utility>

#include "config/svo_config.hpp"
#include "database/Frame.hpp"
#include "feature_tracking/tracking_result.hpp"

namespace omni_slam {
namespace {
std::atomic<size_t> g_frame_id{0};
}  // namespace

Frame::Frame(int64_t                             timestamp_ns,
             const std::vector<cv::Mat>&         images,
             const std::vector<CameraParameter>& camera_parameters)
  : id_(g_frame_id.fetch_add(1, std::memory_order_relaxed))
  , timestamp_ns_(timestamp_ns)
  , images_(images)
  , image_pyramids_(images.size())
  , kCamNum{images.size()}
  , is_keyframe_{false} {
  cams_.clear();
  cams_.reserve(images.size());

  for (const auto& params : camera_parameters) {
    auto camera = CameraModelFactory::create(params);
    if (camera) {
      cams_.push_back(std::move(camera));
    }
  }
  tracking_result_ = std::make_unique<TrackingResult>(images_.size());

  T_b_cs_.resize(images.size());
  for (size_t i = 0; i < images.size(); ++i) {
    if (i < SVOConfig::camera_T_b_c.size()) {
      T_b_cs_[i] = SVOConfig::camera_T_b_c[i];
    }
  }
  mp_id_to_bearings_.resize(images.size());
}

Frame::~Frame() {
  tracking_result_.reset();
}

void Frame::add_observation(size_t                 cam_idx,
                            size_t                 mp_id,
                            const Eigen::Vector3d& bearing) {
  if (cam_idx >= mp_id_to_bearings_.size()) {
    return;
  }
  mp_id_to_bearings_[cam_idx][mp_id] = bearing;
}

void Frame::remove_observation(const uint64_t& mp_id) {
  for (auto& mp_id_to_bearing : mp_id_to_bearings_) {
    mp_id_to_bearing.erase(mp_id);
  }
}
}  // namespace omni_slam
