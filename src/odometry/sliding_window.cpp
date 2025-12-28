#include "odometry/sliding_window.hpp"

#include "database/Frame.hpp"
#include "database/MapPoint.hpp"

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

std::shared_ptr<Frame> SlidingWindow::RemoveFrame(size_t id) {
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

const std::vector<size_t>& SlidingWindow::FrameIds() const {
  return frame_ids_;
}

std::shared_ptr<MapPoint> SlidingWindow::GetMapPoint(size_t id) const {
  const auto it = map_points_.find(id);
  return (it == map_points_.end()) ? nullptr : it->second;
}

const std::unordered_map<size_t, std::shared_ptr<MapPoint>>& SlidingWindow::MapPoints()
  const {
  return map_points_;
}

void SlidingWindow::Clear() {
  frame_ids_.clear();
  frames_.clear();
  map_points_.clear();
}

}  // namespace omni_slam
