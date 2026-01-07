#pragma once

#include <memory>
#include <unordered_map>
#include <Eigen/Dense>
#include "utils/types.hpp"
namespace omni_slam {

class Frame;
struct ReprojectionFactor {
  std::weak_ptr<Frame> frame;
  size_t               cam_id;
  Eigen::Vector2d      uv;
};

class MapPoint {
public:
  MapPoint() = delete;
  MapPoint(const size_t& id);
  ~MapPoint();

  void AddFactor(ReprojectionFactor factor);
  void RemoveFactor(const FrameCamId& frame_cam_id);

public:
  const uint64_t Id() const { return id_; }
  double&        InvDepth() { return inv_depth_; }
  void           SetInvDepth(double inv_depth);

private:
  const uint64_t id_;

  Eigen::Vector2d n_uv_;
  double          inv_depth_;

  uint64_t host_frame_id_;

  std::unordered_map<FrameCamId, ReprojectionFactor> reprojection_factor_map;
};
}  // namespace omni_slam
