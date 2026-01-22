
#include "database/Frame.hpp"
#include "database/MapPoint.hpp"
#include "feature_tracking/tracking_result.hpp"
#include "odometry/sliding_window.hpp"
#include "sliding_window.hpp"

namespace omni_slam {

SlidingWindow::SlidingWindow(size_t max_size)
  : next_map_point_id_(0) {
  SetMaxSize(max_size);
}

void SlidingWindow::SetMaxSize(size_t max_size) {
  max_size_ = max_size;
  frame_ids_.reserve(max_size);
}

size_t SlidingWindow::MaxSize() const {
  return max_size_;
}

size_t SlidingWindow::FrameCount() const {
  return frames_.size();
}

size_t SlidingWindow::MapPointCount() const {
  return map_points_.size();
}

void SlidingWindow::AddFrame(std::shared_ptr<Frame> frame) {
  if (!frame) {
    return;
  }
  const size_t id = frame->Id();
  auto         it = frames_.find(id);

  frame_ids_.push_back(id);
  frames_.emplace(id, frame);
}

std::shared_ptr<Frame> SlidingWindow::GetFrame(const uint64_t& id) {
  const auto it = frames_.find(id);
  return (it == frames_.end()) ? nullptr : it->second;
}

std::shared_ptr<Frame> SlidingWindow::RemoveFrame(uint64_t id) {
  auto it = frames_.find(id);
  if (it == frames_.end()) {
    return nullptr;
  }
  auto removed = it->second;
  frames_.erase(it);

  for (size_t i = 0; i < frame_ids_.size(); ++i) {
    if (frame_ids_[i] == id) {
      frame_ids_.erase(frame_ids_.begin() + static_cast<long>(i));
      break;
    }
  }
  return removed;
}

std::shared_ptr<MapPoint> SlidingWindow::GetMapPoint(const uint64_t& id) const {
  const auto it = map_points_.find(id);
  return (it == map_points_.end()) ? nullptr : it->second;
}

std::shared_ptr<MapPoint> SlidingWindow::GetOrCreateMapPointCandidate(
  const uint64_t& id) {
  auto it = map_point_candidates_.find(id);
  if (it != map_point_candidates_.end()) {
    return it->second;
  }
  auto map_point            = std::make_shared<MapPoint>(id);
  map_point_candidates_[id] = map_point;
  return map_point;
}

bool SlidingWindow::HasMapPoint(const uint64_t& id) const {
  const auto it = map_points_.find(id);
  return !(it == map_points_.end());
}

void SlidingWindow::Clear() {
  frame_ids_.clear();
  frames_.clear();
  map_points_.clear();
  map_point_candidates_.clear();
}

}  // namespace omni_slam
