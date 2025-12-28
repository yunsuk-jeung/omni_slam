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
  std::shared_ptr<Frame> RemoveFrame(size_t id);

  std::shared_ptr<MapPoint> GetMapPoint(size_t id) const;

  const std::vector<size_t>&                                   FrameIds() const;
  const std::unordered_map<size_t, std::shared_ptr<MapPoint>>& MapPoints() const;

  void Clear();

private:
  size_t                                                max_size_;
  size_t                                                next_map_point_id_;
  std::vector<size_t>                                   frame_ids_;
  std::unordered_map<size_t, std::shared_ptr<Frame>>    frames_;
  std::unordered_map<size_t, std::shared_ptr<MapPoint>> map_points_;
};

}  // namespace omni_slam
