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

  void AddObservation(const FrameCamId& frame_cam_id, const Eigen::Vector3d& uv);
  void RemoveObservation(const FrameCamId& frame_cam_id);

public:
  const uint64_t    GetId() const { return id_; }
  const Status&     GetStatus() const { return status_; }
  void              SetStatus(const Status& status) { status_ = status; }
  Eigen::Vector3d&  GetBearing() { return bearing_; }
  double&           GetInvDist() { return inv_dist_; }
  void              SetInvDist(const double& inv_dist) { inv_dist_ = inv_dist; };
  auto&             GetObservation() { return frame_cam_id_to_bearing_; }
  FrameCamId&       GetHostFrameCamId() { return host_frame_cam_id_; }
  const FrameCamId& GetHostFrameCamId() const { return host_frame_cam_id_; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  const uint64_t id_;
  Status         status_;

  Eigen::Vector3d bearing_;
  double          inv_dist_;

  FrameCamId host_frame_cam_id_;

  std::unordered_map<FrameCamId, Eigen::Vector3d> frame_cam_id_to_bearing_;
};
}  // namespace omni_slam
