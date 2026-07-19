#pragma once
#include <cstdint>
#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

#include "camera_model/camera_model.hpp"
#include "omni_slam/types.hpp"

namespace omni_slam {
class TrackingResult;
class CameraModelBase;
class Frame {
 public:
  Frame() = delete;
  Frame(int64_t                             timestamp_ns,
        const std::vector<cv::Mat>&         images,
        const std::vector<CameraParameter>& camera_parameters);
  ~Frame();

 public:
  const size_t          cam_num() const { return kCamNum; }
  uint64_t              id() const { return id_; }
  int64_t               timestamp_ns() const { return timestamp_ns_; }
  cv::Mat&              image(size_t cam_idx) { return images_[cam_idx]; }
  const cv::Mat&        image(size_t cam_idx) const { return images_[cam_idx]; }
  std::vector<cv::Mat>& image_pyramid(size_t cam_idx) {
    return image_pyramids_[cam_idx];
  }
  const std::vector<cv::Mat>& image_pyramid(size_t cam_idx) const {
    return image_pyramids_[cam_idx];
  }
  TrackingResult*       tracking_result_ptr() { return tracking_result_.get(); }
  const TrackingResult* tracking_result_ptr() const {
    return tracking_result_.get();
  }

  CameraModelBase* cam(size_t cam_idx) { return cams_[cam_idx].get(); }

  const Sophus::SE3d& twb() const { return T_w_b_; }
  Sophus::SE3d&       twb() { return T_w_b_; }
  void                twb(const Sophus::SE3d T_w_b) { T_w_b_ = T_w_b; }
  Sophus::SE3d        twc(size_t i) { return T_w_b_ * T_b_cs_[i]; }
  const Sophus::SE3d& tbc(size_t i) const { return T_b_cs_[i]; }

  // First-estimate (FEJ) accessors. Until set_lin_true() is called the
  // linearization point follows the current estimate; afterwards it stays
  // frozen at the value it had when the frame joined the marginalization
  // prior, and only twb() keeps moving.
  const Sophus::SE3d& twb_lin() const {
    return T_w_b_lin_ ? *T_w_b_lin_ : T_w_b_;
  }
  bool is_linearized() const { return T_w_b_lin_.has_value(); }
  void set_lin_true() {
    if (!T_w_b_lin_) {
      T_w_b_lin_ = T_w_b_;
    }
  }

  void       keyframe() { is_keyframe_ = true; }
  const bool is_keyframe() const { return is_keyframe_; }

  void add_observation(size_t                 cam_idx,
                       uint64_t               mp_id,
                       const Eigen::Vector3d& bearing);
  std::vector<std::unordered_map<uint64_t, Eigen::Vector3d>>& observations() {
    return mp_id_to_bearings_;
  }
  std::unordered_map<uint64_t, Eigen::Vector3d>& observation(size_t i) {
    return mp_id_to_bearings_[i];
  }
  void remove_observation(uint64_t mp_id);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  const size_t                      kCamNum;
  uint64_t                          id_;
  int64_t                           timestamp_ns_;
  std::vector<cv::Mat>              images_;
  std::vector<std::vector<cv::Mat>> image_pyramids_;

  std::unique_ptr<TrackingResult> tracking_result_;

  Sophus::SE3d                                  T_w_b_;
  std::optional<Sophus::SE3d>                   T_w_b_lin_;
  std::vector<std::unique_ptr<CameraModelBase>> cams_;
  std::vector<Sophus::SE3d>                     T_b_cs_;

  bool is_keyframe_;

  std::vector<std::unordered_map<uint64_t, Eigen::Vector3d>> mp_id_to_bearings_;
};

}  // namespace omni_slam
