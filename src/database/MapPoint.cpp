#include "database/Frame.hpp"
#include "database/MapPoint.hpp"
#include "MapPoint.hpp"

namespace omni_slam {

omni_slam::MapPoint::MapPoint(const size_t& id)
  : id_{id}
  , inv_dist_{0.0} {}

MapPoint::~MapPoint() {}

void MapPoint::AddFactor(const FrameCamId& frame_cam_id, const Eigen::Vector2d& uv) {
  reprojection_factor_map_.insert({frame_cam_id, uv});
}

void MapPoint::RemoveFactor(const FrameCamId& frame_cam_id) {}
}  // namespace omni_slam
