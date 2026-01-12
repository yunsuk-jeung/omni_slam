#pragma once

#include <cstddef>
#include <memory>
#include <vector>
#include <unordered_map>

namespace omni_slam {

class Frame;
class MapPoint;

class SlidingWindow {
public:
  explicit SlidingWindow(size_t max_size = 0);

  void   SetMaxSize(size_t max_size);
  size_t MaxSize() const;

  size_t FrameCount() const;
  size_t MapPointCount() const;

  void                   AddFrame(std::shared_ptr<Frame> frame);
  std::shared_ptr<Frame> RemoveFrame(uint64_t id);

  std::shared_ptr<MapPoint> GetMapPoint(const uint64_t& id) const;
  std::shared_ptr<MapPoint> GetOrCreateCandidateMapPoint(const uint64_t& id);
  bool                      HasMapPoint(const uint64_t& id) const;

  const std::vector<uint64_t>& FrameIds() const { return frame_ids_; };
  const std::unordered_map<uint64_t, std::shared_ptr<MapPoint>>& MapPoints() const {
    return map_points_;
  };

  void Clear();

private:
  size_t                                                  max_size_;
  uint64_t                                                next_map_point_id_;
  std::vector<uint64_t>                                   frame_ids_;
  std::unordered_map<uint64_t, std::shared_ptr<Frame>>    frames_;
  std::unordered_map<uint64_t, std::shared_ptr<MapPoint>> map_points_;
  std::unordered_map<uint64_t, std::shared_ptr<MapPoint>> map_point_candidates_;
};

}  // namespace omni_slam
