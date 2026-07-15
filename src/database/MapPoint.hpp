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

  void add_observation(const FrameCamId&      frame_cam_id,
                       const Eigen::Vector3d& uv);
  void remove_observation(const FrameCamId& frame_cam_id);

 public:
  const uint64_t   get_id() const { return id_; }
  const Status&    get_status() const { return status_; }
  void             set_status(const Status& status) { status_ = status; }
  Eigen::Vector3d& get_bearing() { return bearing_; }
  double&          get_inv_dist() { return inv_dist_; }
  void        set_inv_dist(const double& inv_dist) { inv_dist_ = inv_dist; };
  auto&       get_observation() { return frame_cam_id_to_bearing_; }
  FrameCamId& get_host_frame_cam_id() { return host_frame_cam_id_; }
  const FrameCamId& get_host_frame_cam_id() const { return host_frame_cam_id_; }

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
