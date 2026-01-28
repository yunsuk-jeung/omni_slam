#include "database/Frame.hpp"
#include "database/MapPoint.hpp"
#include "MapPoint.hpp"

namespace omni_slam {

omni_slam::MapPoint::MapPoint(const size_t& id)
  : id_{id}
  , status_{Status::NONE}
  , inv_dist_{0.0} {}

MapPoint::~MapPoint() {}

void MapPoint::AddObservation(const FrameCamId& frame_cam_id, const Eigen::Vector2d& uv) {
  frame_cam_id_to_uv.insert({frame_cam_id, uv});
}

void MapPoint::RemoveObservation(const FrameCamId& frame_cam_id) {}
}  // namespace omni_slam
