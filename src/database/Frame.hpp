#pragma once
#include <cstdint>
#include <memory>
#include <vector>
#include <unordered_map>
#include <sophus/se3.hpp>
#include <opencv2/core.hpp>
#include "utils/types.hpp"
#include "camera_model/camera_model.hpp"

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
  const size_t          GetCamNum() const { return kCamNum; }
  uint64_t              GetId() const { return id_; }
  int64_t               GetTimestampNs() const { return timestamp_ns_; }
  cv::Mat&              GetImage(size_t cam_idx) { return images_[cam_idx]; }
  const cv::Mat&        GetImage(size_t cam_idx) const { return images_[cam_idx]; }
  std::vector<cv::Mat>& GetImagePyramid(size_t cam_idx) {
    return image_pyramids_[cam_idx];
  }
  const std::vector<cv::Mat>& GetImagePyramid(size_t cam_idx) const {
    return image_pyramids_[cam_idx];
  }
  TrackingResult*       GetTrackingResultPtr() { return tracking_result_.get(); }
  const TrackingResult* GetTrackingResultPtr() const { return tracking_result_.get(); }

  CameraModelBase* GetCam(size_t cam_idx) { return cams_[cam_idx].get(); }

  const Sophus::SE3d& GetTwb() const { return T_w_b_; }
  Sophus::SE3d&       GetTwb() { return T_w_b_; }
  void                SetTwb(const Sophus::SE3d T_wb) { T_w_b_ = T_wb; }
  Sophus::SE3d        GetTwc(size_t i) { return T_w_b_ * T_b_cs_[i]; }
  const Sophus::SE3d& GetTbc(size_t i) const { return T_b_cs_[i]; }

  void       SetKeyframe() { is_keyframe_ = true; }
  const bool IsKeyframe() const { return is_keyframe_; }

  void AddObservation(size_t cam_idx, size_t mp_id, const Eigen::Vector3d& bearing);
  std::unordered_map<size_t, Eigen::Vector3d>& GetObservation(size_t i) {
    return mp_id_to_bearings_[i];
  }
  void RemoveObservation(const uint64_t& mp_id);

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
