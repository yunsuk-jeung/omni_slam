#include "database/Frame.hpp"
#include "database/MapPoint.hpp"
#include "MapPoint.hpp"

namespace omni_slam {

omni_slam::MapPoint::MapPoint(const size_t& id)
  : id_{id}
  , status_{Status::NONE}
  , bearing_{Eigen::Vector3d::Zero()}
  , inv_dist_{0.0} {}

MapPoint::~MapPoint() {}

void MapPoint::AddObservation(const FrameCamId&      frame_cam_id,
                              const Eigen::Vector3d& bearing) {
  frame_cam_id_to_bearing_.insert({frame_cam_id, bearing});
}

void MapPoint::RemoveObservation(const FrameCamId& frame_cam_id) {
  frame_cam_id_to_bearing_.erase(frame_cam_id);
}
}  // namespace omni_slam
