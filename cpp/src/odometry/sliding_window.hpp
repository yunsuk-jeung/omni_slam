#pragma once

#include <cstddef>
#include <memory>
#include <set>
#include <unordered_map>
#include <vector>

namespace omni_slam {

class Frame;
class MapPoint;

class SlidingWindow {
 public:
  explicit SlidingWindow(size_t max_size = 0);

  void   max_size(size_t max_size);
  size_t max_size() const;

  size_t frame_count() const;
  size_t map_point_count() const;

  void                   add_frame(std::shared_ptr<Frame>& frame);
  std::shared_ptr<Frame> frame(const uint64_t& id);
  void                   mark_keyframe(uint64_t id);
  void                   remove_frames(const std::set<uint64_t>& ids);

  void                      add_map_point(std::shared_ptr<MapPoint>& map_point);
  std::shared_ptr<MapPoint> map_point(const uint64_t& id) const;
  std::shared_ptr<MapPoint> get_or_create_map_point_candidate(
    const uint64_t& id);
  bool has_map_point(const uint64_t& id) const;

  const std::unordered_map<uint64_t, std::shared_ptr<Frame>>& frames() const {
    return frames_;
  }
  const std::set<uint64_t>& frame_ids() const { return frame_ids_; };
  const std::set<uint64_t>& keyframe_ids() const { return keyframe_ids_; }
  const std::unordered_map<uint64_t, std::shared_ptr<MapPoint>>& map_points()
    const {
    return map_points_;
  };

  std::unordered_map<uint64_t, std::shared_ptr<MapPoint>>&
  map_point_candidates() {
    return map_point_candidates_;
  };

  void clear();

 private:
  size_t                                                  max_size_;
  uint64_t                                                next_map_point_id_;
  std::set<uint64_t>                                      frame_ids_;
  std::set<uint64_t>                                      keyframe_ids_;
  std::unordered_map<uint64_t, std::shared_ptr<Frame>>    frames_;
  std::unordered_map<uint64_t, std::shared_ptr<MapPoint>> map_points_;
  std::unordered_map<uint64_t, std::shared_ptr<MapPoint>> map_point_candidates_;
};

}  // namespace omni_slam
