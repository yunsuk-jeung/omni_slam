#pragma once

#include <memory>
#include <unordered_map>
#include <Eigen/Dense>
#include "utils/types.hpp"
namespace omni_slam {

class Frame;
class MapPoint {
public:
  enum class Status {
    NONE,
    TRACKING,
  };

  MapPoint() = delete;
  MapPoint(const size_t& id);
  ~MapPoint();

  void AddObservation(const FrameCamId& frame_cam_id, const Eigen::Vector2d& uv);
  void RemoveObservation(const FrameCamId& frame_cam_id);

public:
  const uint64_t   GetId() const { return id_; }
  const Status&    GetStatus() const { return status_; }
  void             SetStatus(const Status& status) { status_ = status; }
  Eigen::Vector2d& GetDirection() { return direction_; }
  double&          GetInvDist() { return inv_dist_; }
  void             SetInvDist(const double& inv_dist) { inv_dist_ = inv_dist; };
  auto&            GetFrameCamIdToUv() { return frame_cam_id_to_uv; }
  uint64_t&        GetHostFrameId() { return host_frame_id_; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  const uint64_t id_;
  Status         status_;

  Eigen::Vector2d direction_;
  double          inv_dist_;

  uint64_t host_frame_id_;

  std::unordered_map<FrameCamId, Eigen::Vector2d> frame_cam_id_to_uv;
};
}  // namespace omni_slam
