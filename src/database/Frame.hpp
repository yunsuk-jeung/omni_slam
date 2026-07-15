#pragma once
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

#include "camera_model/camera_model.hpp"
#include "utils/types.hpp"

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
  const size_t   get_cam_num() const { return kCamNum; }
  uint64_t       get_id() const { return id_; }
  int64_t        get_timestamp_ns() const { return timestamp_ns_; }
  cv::Mat&       get_image(size_t cam_idx) { return images_[cam_idx]; }
  const cv::Mat& get_image(size_t cam_idx) const { return images_[cam_idx]; }
  std::vector<cv::Mat>& get_image_pyramid(size_t cam_idx) {
    return image_pyramids_[cam_idx];
  }
  const std::vector<cv::Mat>& get_image_pyramid(size_t cam_idx) const {
    return image_pyramids_[cam_idx];
  }
  TrackingResult* get_tracking_result_ptr() { return tracking_result_.get(); }
  const TrackingResult* get_tracking_result_ptr() const {
    return tracking_result_.get();
  }

  CameraModelBase* get_cam(size_t cam_idx) { return cams_[cam_idx].get(); }

  const Sophus::SE3d& get_twb() const { return T_w_b_; }
  Sophus::SE3d&       get_twb() { return T_w_b_; }
  void                set_twb(const Sophus::SE3d T_wb) { T_w_b_ = T_wb; }
  Sophus::SE3d        get_twc(size_t i) { return T_w_b_ * T_b_cs_[i]; }
  const Sophus::SE3d& get_tbc(size_t i) const { return T_b_cs_[i]; }

  void       set_keyframe() { is_keyframe_ = true; }
  const bool is_keyframe() const { return is_keyframe_; }

  void add_observation(size_t                 cam_idx,
                       size_t                 mp_id,
                       const Eigen::Vector3d& bearing);
  std::vector<std::unordered_map<size_t, Eigen::Vector3d>>& get_observations() {
    return mp_id_to_bearings_;
  }
  std::unordered_map<size_t, Eigen::Vector3d>& get_observation(size_t i) {
    return mp_id_to_bearings_[i];
  }
  void remove_observation(const uint64_t& mp_id);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  const size_t                      kCamNum;
  uint64_t                          id_;
  int64_t                           timestamp_ns_;
  std::vector<cv::Mat>              images_;
  std::vector<std::vector<cv::Mat>> image_pyramids_;

  std::unique_ptr<TrackingResult> tracking_result_;

  Sophus::SE3d                                  T_w_b_;
  std::vector<std::unique_ptr<CameraModelBase>> cams_;
  std::vector<Sophus::SE3d>                     T_b_cs_;

  bool is_keyframe_;

  std::vector<std::unordered_map<size_t, Eigen::Vector3d>> mp_id_to_bearings_;
};

}  // namespace omni_slam
