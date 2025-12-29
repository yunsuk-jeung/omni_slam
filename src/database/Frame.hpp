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
  Frame() = delete;
  Frame(const std::vector<cv::Mat>&         images,
        const std::vector<CameraParameter>& camera_parameters);
  ~Frame();

public:
  const size_t          CamNum() const { return cam_num_; }
  uint64_t              Id() const { return id_; }
  cv::Mat&              Image(size_t cam_idx) { return images_[cam_idx]; }
  const cv::Mat&        Image(size_t cam_idx) const { return images_[cam_idx]; }
  std::vector<cv::Mat>& ImagePyramid(size_t cam_idx) { return image_pyramids_[cam_idx]; }
  const std::vector<cv::Mat>& ImagePyramid(size_t cam_idx) const {
    return image_pyramids_[cam_idx];
  }
  TrackingResult*       TrackingResultPtr() { return tracking_result_.get(); }
  const TrackingResult* TrackingResultPtr() const { return tracking_result_.get(); }

private:
  const size_t                      cam_num_;
  uint64_t                          id_;
  std::vector<cv::Mat>              images_;
  std::vector<std::vector<cv::Mat>> image_pyramids_;

  std::unique_ptr<TrackingResult> tracking_result_;

  std::vector<std::unique_ptr<CameraModelBase>> cams_;
  std::vector<Sophus::SE3d>                     T_bcs_;

  Sophus::SE3d T_wb;
};

}  // namespace omni_slam
