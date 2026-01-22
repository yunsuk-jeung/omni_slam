#pragma once

#include <memory>
#include <unordered_map>
#include <Eigen/Dense>
#include "utils/types.hpp"
namespace omni_slam {

class Frame;
class MapPoint {
public:
  MapPoint() = delete;
  MapPoint(const size_t& id);
  ~MapPoint();

  void AddFactor(const FrameCamId& frame_cam_id, const Eigen::Vector2d& uv);
  void RemoveFactor(const FrameCamId& frame_cam_id);

public:
  const uint64_t   Id() const { return id_; }
  Eigen::Vector2d& Direction() { return direction_; }
  double&          InvDist() { return inv_dist_; }
  void             SetInvDist(const double& inv_dist) { inv_dist_ = inv_dist; };
  auto&            ReprojectionFactorMap() { return reprojection_factor_map_; }
  uint64_t&        HostFrameId() { return host_frame_id_; }

private:
  const uint64_t id_;

  Eigen::Vector2d direction_;
  double          inv_dist_;

  uint64_t host_frame_id_;

  std::unordered_map<FrameCamId, Eigen::Vector2d> reprojection_factor_map_;
};
}  // namespace omni_slam
