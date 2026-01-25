#pragma once
#include <cstdint>
#include <memory>
#include <vector>
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

  const Sophus::SE3d& GetTwb() const { return T_wb_; }
  Sophus::SE3d&       GetTwb() { return T_wb_; }
  Sophus::SE3d        GetTwc(size_t i) { return T_wb_ * T_bcs_[i]; }
  const std::vector<Sophus::SE3d>& GetTbc() const { return T_bcs_; }

  void       SetKeyframe() { is_keyframe_ = true; }
  const bool IsKeyframe() const { return is_keyframe_; }

private:
  const size_t                      kCamNum;
  uint64_t                          id_;
  int64_t                           timestamp_ns_;
  std::vector<cv::Mat>              images_;
  std::vector<std::vector<cv::Mat>> image_pyramids_;

  std::unique_ptr<TrackingResult> tracking_result_;

  Sophus::SE3d                                  T_wb_;
  std::vector<std::unique_ptr<CameraModelBase>> cams_;
  std::vector<Sophus::SE3d>                     T_bcs_;

  bool is_keyframe_;
};

}  // namespace omni_slam
