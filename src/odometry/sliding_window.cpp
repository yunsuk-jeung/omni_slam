
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
}

size_t SlidingWindow::GetMaxSize() const {
  return max_size_;
}

size_t SlidingWindow::GetFrameCount() const {
  return frames_.size();
}

size_t SlidingWindow::GetMapPointCount() const {
  return map_points_.size();
}

void SlidingWindow::AddFrame(std::shared_ptr<Frame>& frame) {
  if (!frame) {
    return;
  }
  const size_t id = frame->GetId();
  auto         it = frames_.find(id);

  frame_ids_.insert(id);
  frames_.emplace(id, frame);
  if (frame->IsKeyframe()) {
    MarkKeyframe(id);
  }
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

  frame_ids_.erase(id);
  RemoveKeyframe(id);
  return removed;
}

void SlidingWindow::MarkKeyframe(uint64_t id) {
  if (frames_.find(id) == frames_.end()) {
    return;
  }
  keyframe_ids_.insert(id);
}

void SlidingWindow::RemoveKeyframe(uint64_t id) {
  keyframe_ids_.erase(id);
}

void SlidingWindow::AddMapPoint(std::shared_ptr<MapPoint>& map_point) {
  if (!map_point) {
    return;
  }
  const size_t id = map_point->GetId();
  auto         it = map_points_.find(id);
  map_points_.emplace(id, map_point);
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

bool SlidingWindow::GetHasMapPoint(const uint64_t& id) const {
  const auto it = map_points_.find(id);
  return !(it == map_points_.end());
}

void SlidingWindow::Clear() {
  frame_ids_.clear();
  keyframe_ids_.clear();
  frames_.clear();
  map_points_.clear();
  map_point_candidates_.clear();
}

}  // namespace omni_slam
