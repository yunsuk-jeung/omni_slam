

#include <algorithm>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include "config/svo_config.hpp"
#include "database/frame.hpp"
#include "feature_tracking/grid_detector.hpp"
#include "feature_tracking/optical_flow.hpp"

namespace omni_slam {
namespace {
bool is_point_in_image(const cv::Point2f& pt, const cv::Mat& image) {
  return pt.x >= 0.0f && pt.y >= 0.0f && pt.x < image.cols && pt.y < image.rows;
}

void run_forward_backward_optical_flow(const std::vector<cv::Mat>& src_pyramid,
                                       const std::vector<cv::Mat>& dst_pyramid,
                                       const std::vector<cv::Point2f>& src_uvs,
                                       std::vector<cv::Point2f>&       dst_uvs,
                                       std::vector<uchar>&             status) {
  std::vector<cv::Point2f> reverse_uvs;
  std::vector<uchar>       reverse_status;

  cv::calcOpticalFlowPyrLK(src_pyramid,
                           dst_pyramid,
                           src_uvs,
                           dst_uvs,
                           status,
                           cv::noArray(),
                           cv::Size(SVOConfig::optical_flow_patch_size,
                                    SVOConfig::optical_flow_patch_size),
                           SVOConfig::max_pyramid_level);

  cv::calcOpticalFlowPyrLK(dst_pyramid,
                           src_pyramid,
                           dst_uvs,
                           reverse_uvs,
                           reverse_status,
                           cv::noArray(),
                           cv::Size(SVOConfig::optical_flow_patch_size,
                                    SVOConfig::optical_flow_patch_size),
                           SVOConfig::max_pyramid_level);

  const size_t track_count = status.size();
  for (size_t i = 0; i < track_count; ++i) {
    if (!status[i] || !reverse_status[i]) {
      status[i] = 0;
      continue;
    }
    if (!is_point_in_image(dst_uvs[i], dst_pyramid.front())) {
      status[i] = 0;
      continue;
    }
    // Threshold is compared against the squared forward-backward error.
    const cv::Point2f diff         = src_uvs[i] - reverse_uvs[i];
    const float       dist_norm_sq = diff.x * diff.x + diff.y * diff.y;
    if (dist_norm_sq > SVOConfig::optical_flow_dist_threshold) {
      status[i] = 0;
    }
  }
}

}  // namespace

OpticalFlow::OpticalFlow(const size_t cam_num)
  : kCamNum{cam_num}
  , prev_frame_{nullptr}
  , next_feature_id_{0} {
  if (SVOConfig::equalize_histogram) {
    clahe_ = cv::createCLAHE(SVOConfig::clahe_clip_limit,
                             cv::Size(SVOConfig::clahe_tile_size,
                                      SVOConfig::clahe_tile_size));
  }
}

void OpticalFlow::prepare_images_and_pyramids(
  std::shared_ptr<Frame>& curr_frame) {
  for (size_t i = 0; i < kCamNum; i++) {
    const auto& image = curr_frame->image(i);
    cv::Mat     gray;
    if (image.channels() == 1) {
      gray = image;
    }
    else {
      cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }

    if (SVOConfig::equalize_histogram && clahe_) {
      clahe_->apply(gray, gray);
    }
    // Store the grayscale image in the frame so downstream uses match pyramids.
    curr_frame->image(i) = gray;
    const cv::Point2i patch(SVOConfig::optical_flow_patch_size,
                            SVOConfig::optical_flow_patch_size);
    cv::buildOpticalFlowPyramid(gray,
                                curr_frame->image_pyramid(i),
                                patch,
                                SVOConfig::max_pyramid_level,
                                true,
                                cv::BORDER_REFLECT_101,
                                cv::BORDER_CONSTANT);
  }
}

void OpticalFlow::track_mono(const std::shared_ptr<Frame>& curr_frame) {
  constexpr size_t kLeftCam = 0;
  if (!prev_frame_ || !prev_frame_->tracking_result_ptr()) {
    return;
  }

  const auto* prev_result = prev_frame_->tracking_result_ptr();
  const auto& prev_ids    = prev_result->ids(kLeftCam);
  const auto& prev_uvs    = prev_result->uvs(kLeftCam);
  auto*       curr_result = curr_frame->tracking_result_ptr();
  if (prev_uvs.empty()) {
    return;
  }

  std::vector<cv::Point2f> tracked_uvs;
  std::vector<uchar>       status;
  run_forward_backward_optical_flow(prev_frame_->image_pyramid(kLeftCam),
                                    curr_frame->image_pyramid(kLeftCam),
                                    prev_uvs,
                                    tracked_uvs,
                                    status);

  const size_t track_count = status.size();

  for (size_t i = 0; i < track_count; ++i) {
    if (!status[i]) {
      continue;
    }
    curr_result->add_feature(kLeftCam, tracked_uvs[i], prev_ids[i]);
  }
}

void OpticalFlow::track_stereo(const std::shared_ptr<Frame>& curr_frame) {
  constexpr size_t kLeftCam    = 0;
  constexpr size_t kRightCam   = 1;
  auto*            curr_result = curr_frame->tracking_result_ptr();

  const auto& left_ids = curr_result->ids(kLeftCam);
  const auto& left_uvs = curr_result->uvs(kLeftCam);
  if (left_uvs.empty()) {
    return;
  }

  std::vector<cv::Point2f> right_uvs;
  std::vector<uchar>       status;
  run_forward_backward_optical_flow(curr_frame->image_pyramid(kLeftCam),
                                    curr_frame->image_pyramid(kRightCam),
                                    left_uvs,
                                    right_uvs,
                                    status);

  const size_t track_count = status.size();
  for (size_t i = 0; i < track_count; ++i) {
    if (!status[i]) {
      continue;
    }
    curr_result->add_feature(kRightCam, right_uvs[i], left_ids[i]);
  }
}

void OpticalFlow::detect_features(const std::shared_ptr<Frame>& curr_frame) {
  constexpr size_t kLeftCam = 0;

  auto*       curr_result = curr_frame->tracking_result_ptr();
  const auto& curr_uvs    = curr_result->uvs(kLeftCam);

  const auto keypoints = detect_grid_features(curr_frame->image(kLeftCam),
                                              SVOConfig::feature_grid_rows,
                                              SVOConfig::feature_grid_cols,
                                              SVOConfig::fast_threshold,
                                              curr_uvs);
  for (const auto& keypoint : keypoints) {
    curr_result->add_feature(kLeftCam, keypoint.pt, next_feature_id_++);
  }
}

void OpticalFlow::process(std::shared_ptr<Frame>& curr_frame) {
  auto* curr_result = curr_frame->tracking_result_ptr();

  const size_t prev_left_size = (prev_frame_
                                 && prev_frame_->tracking_result_ptr())
                                  ? prev_frame_->tracking_result_ptr()->size(0)
                                  : 0;
  const int    grid_rows      = std::max(1, SVOConfig::feature_grid_rows);
  const int    grid_cols      = std::max(1, SVOConfig::feature_grid_cols);
  const size_t expected_left  = (prev_left_size >> 1)
                               + static_cast<size_t>(grid_rows * grid_cols);
  curr_result->reserve(0, expected_left);
  curr_result->reserve(1, expected_left);
  prepare_images_and_pyramids(curr_frame);
  track_mono(curr_frame);
  detect_features(curr_frame);
  if (kCamNum == 2) {
    track_stereo(curr_frame);
  }
  prev_frame_ = curr_frame;
}
}  // namespace omni_slam
