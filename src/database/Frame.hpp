#pragma once
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
  Frame();
  Frame(const std::vector<cv::Mat>&         images,
        const std::vector<CameraParameter>& camera_parameters);
  ~Frame();

  void SetCameraParams(const std::vector<CameraParameter>& camera_parameters);

  cv::Mat&              Image(size_t cam_idx) { return images_[cam_idx]; }
  const cv::Mat&        Image(size_t cam_idx) const { return images_[cam_idx]; }
  std::vector<cv::Mat>& ImagePyramid(size_t cam_idx) { return image_pyramids_[cam_idx]; }
  const std::vector<cv::Mat>& ImagePyramid(size_t cam_idx) const {
    return image_pyramids_[cam_idx];
  }
  TrackingResult*       TrackingResultPtr() { return tracking_result_.get(); }
  const TrackingResult* TrackingResultPtr() const { return tracking_result_.get(); }
  void SetTrackingResult(std::unique_ptr<TrackingResult> tracking_result) {
    tracking_result_ = std::move(tracking_result);
  }

private:
  std::vector<cv::Mat>              images_;
  std::vector<std::vector<cv::Mat>> image_pyramids_;

  std::vector<std::unique_ptr<CameraModelBase>> cams_;
  std::vector<Sophus::SE3d>                     T_bcs_;

  std::unique_ptr<TrackingResult> tracking_result_;
};

}  // namespace omni_slam
