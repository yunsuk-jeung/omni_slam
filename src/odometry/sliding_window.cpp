
#include <unordered_set>

#include "database/Frame.hpp"
#include "database/MapPoint.hpp"
#include "feature_tracking/tracking_result.hpp"
#include "odometry/sliding_window.hpp"

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
}

std::shared_ptr<Frame> SlidingWindow::GetFrame(const uint64_t& id) {
  const auto it = frames_.find(id);
  return (it == frames_.end()) ? nullptr : it->second;
}

void SlidingWindow::MarkKeyframe(uint64_t id) {
  keyframe_ids_.insert(id);
}

void SlidingWindow::RemoveFrames(const std::set<uint64_t>& frame_ids) {
  if (frame_ids.empty()) {
    return;
  }
  std::unordered_set<uint64_t> to_remove(frame_ids.begin(), frame_ids.end());
  std::unordered_set<uint64_t> removed_map_point_ids;

  auto prune_container = [&](auto& container) {
    for (auto it = container.begin(); it != container.end();) {
      auto& mp = it->second;
      if (!mp) {
        it = container.erase(it);
        continue;
      }

      if (to_remove.find(mp->GetHostFrameCamId().frame_id) != to_remove.end()) {
        removed_map_point_ids.insert(it->first);
        it = container.erase(it);
        continue;
      }

      auto& observations = mp->GetObservation();
      for (auto obs_it = observations.begin(); obs_it != observations.end();) {
        if (to_remove.find(obs_it->first.frame_id) != to_remove.end()) {
          obs_it = observations.erase(obs_it);
        }
        else {
          ++obs_it;
        }
      }

      if (observations.empty()) {
        removed_map_point_ids.insert(it->first);
        it = container.erase(it);
      }
      else {
        ++it;
      }
    }
  };

  prune_container(map_points_);
  prune_container(map_point_candidates_);

  for (const auto id : to_remove) {
    frames_.erase(id);
    frame_ids_.erase(id);
    keyframe_ids_.erase(id);
  }

  if (!removed_map_point_ids.empty()) {
    for (auto& [frame_id, frame] : frames_) {
      for (const auto mp_id : removed_map_point_ids) {
        frame->RemoveObservation(mp_id);
      }
    }
  }
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
