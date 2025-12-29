#include "database/Frame.hpp"
#include "database/MapPoint.hpp"
#include "MapPoint.hpp"

namespace omni_slam {

omni_slam::MapPoint::MapPoint(const size_t& id)
  : id_{id} {}

MapPoint::~MapPoint() {}

void MapPoint::AddFactor(ReprojectionFactor factor) {}

void MapPoint::RemoveFactor(const FrameCamId& frame_cam_id) {}
}  // namespace omni_slam
