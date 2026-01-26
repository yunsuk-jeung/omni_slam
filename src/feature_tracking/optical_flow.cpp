

#include <algorithm>
#include <chrono>
#include <thread>
#include <vector>

#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include "utils/logger.hpp"
#include "config/svo_config.hpp"
#include "database/Frame.hpp"
#include "feature_tracking/optical_flow.hpp"

namespace omni_slam {
namespace {
bool IsPointInImage(const cv::Point2f& pt, const cv::Mat& image) {
  return pt.x >= 0.0f && pt.y >= 0.0f && pt.x < image.cols && pt.y < image.rows;
}

}  // namespace

OpticalFlow::OpticalFlow(const size_t                                   cam_num,
                         tbb::concurrent_queue<std::shared_ptr<Frame>>& in_queue,
                         tbb::concurrent_queue<std::shared_ptr<Frame>>& out_queue)
  : kCamNum{cam_num}
  , in_queue_(in_queue)
  , out_queue_(out_queue)
  , prev_frame_{nullptr}
  , next_feature_id_{0} {
  if (SVOConfig::equalize_histogram) {
    clahe_ = cv::createCLAHE(SVOConfig::clahe_clip_limit,
                             cv::Size(SVOConfig::clahe_tile_size,
                                      SVOConfig::clahe_tile_size));
  }
}

void OpticalFlow::PrepareImagesAndPyramids(std::shared_ptr<Frame>& curr_frame) {
  for (size_t i = 0; i < kCamNum; i++) {
    const auto& image = curr_frame->GetImage(i);
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
    curr_frame->GetImage(i) = gray;
    const cv::Point2i patch(SVOConfig::optical_flow_patch_size,
                            SVOConfig::optical_flow_patch_size);
    cv::buildOpticalFlowPyramid(gray,
                                curr_frame->GetImagePyramid(i),
                                patch,
                                SVOConfig::max_pyramid_level,
                                true,
                                cv::BORDER_REFLECT_101,
                                cv::BORDER_CONSTANT);
  }
}

void OpticalFlow::TrackMono(const std::shared_ptr<Frame>& curr_frame) {
  constexpr size_t kLeftCam = 0;
  if (!prev_frame_ || !prev_frame_->GetTrackingResultPtr()) {
    return;
  }

  const auto* prev_result = prev_frame_->GetTrackingResultPtr();
  const auto& prev_ids    = prev_result->GetIds(kLeftCam);
  const auto& prev_uvs    = prev_result->GetUvs(kLeftCam);
  auto*       curr_result = curr_frame->GetTrackingResultPtr();
  if (prev_uvs.empty()) {
    return;
  }

  std::vector<cv::Point2f> tracked_uvs;
  std::vector<uchar>       status;

  cv::calcOpticalFlowPyrLK(prev_frame_->GetImagePyramid(kLeftCam),
                           curr_frame->GetImagePyramid(kLeftCam),
                           prev_uvs,
                           tracked_uvs,
                           status,
                           cv::noArray(),
                           cv::Size(SVOConfig::optical_flow_patch_size,
                                    SVOConfig::optical_flow_patch_size),
                           SVOConfig::max_pyramid_level);

  std::vector<cv::Point2f> reverse_uvs;
  std::vector<uchar>       reverse_status;

  cv::calcOpticalFlowPyrLK(curr_frame->GetImagePyramid(kLeftCam),
                           prev_frame_->GetImagePyramid(kLeftCam),
                           tracked_uvs,
                           reverse_uvs,
                           reverse_status,
                           cv::noArray(),
                           cv::Size(SVOConfig::optical_flow_patch_size,
                                    SVOConfig::optical_flow_patch_size),
                           SVOConfig::max_pyramid_level);

  for (size_t i = 0; i < tracked_uvs.size(); ++i) {
    if (!status[i] || !reverse_status[i]) {
      continue;
    }
    if (!IsPointInImage(tracked_uvs[i], curr_frame->GetImage(kLeftCam))) {
      continue;
    }
    const cv::Point2f dist       = prev_uvs[i] - reverse_uvs[i];
    const float       distNormSq = dist.x * dist.x + dist.y * dist.y;

    if (distNormSq > SVOConfig::optical_flow_dist_threshold) {
      continue;
    }
    curr_result->AddFeature(kLeftCam, tracked_uvs[i], prev_ids[i]);
  }
}

void OpticalFlow::TrackStereo(const std::shared_ptr<Frame>& curr_frame) {
  constexpr size_t kLeftCam    = 0;
  constexpr size_t kRightCam   = 1;
  auto*            curr_result = curr_frame->GetTrackingResultPtr();

  const auto& left_ids = curr_result->GetIds(kLeftCam);
  const auto& left_uvs = curr_result->GetUvs(kLeftCam);
  if (left_uvs.empty()) {
    return;
  }

  std::vector<cv::Point2f> right_uvs;
  std::vector<uchar>       status;

  cv::calcOpticalFlowPyrLK(curr_frame->GetImagePyramid(kLeftCam),
                           curr_frame->GetImagePyramid(kRightCam),
                           left_uvs,
                           right_uvs,
                           status,
                           cv::noArray(),
                           cv::Size(SVOConfig::optical_flow_patch_size,
                                    SVOConfig::optical_flow_patch_size),
                           SVOConfig::max_pyramid_level);

  std::vector<cv::Point2f> reverse_uvs;
  std::vector<uchar>       reverse_status;

  cv::calcOpticalFlowPyrLK(curr_frame->GetImagePyramid(kRightCam),
                           curr_frame->GetImagePyramid(kLeftCam),
                           right_uvs,
                           reverse_uvs,
                           reverse_status,
                           cv::noArray(),
                           cv::Size(SVOConfig::optical_flow_patch_size,
                                    SVOConfig::optical_flow_patch_size),
                           SVOConfig::max_pyramid_level);

  for (size_t i = 0; i < right_uvs.size(); ++i) {
    if (!status[i] || !reverse_status[i]) {
      continue;
    }
    if (!IsPointInImage(right_uvs[i], curr_frame->GetImage(kRightCam))) {
      continue;
    }
    const cv::Point2f dist       = left_uvs[i] - reverse_uvs[i];
    const float       distNormSq = dist.x * dist.x + dist.y * dist.y;

    if (distNormSq > SVOConfig::optical_flow_dist_threshold) {
      continue;
    }
    curr_result->AddFeature(kRightCam, right_uvs[i], left_ids[i]);
  }
}

void OpticalFlow::DetectFeatures(const std::shared_ptr<Frame>& curr_frame) {
  constexpr size_t kLeftCam = 0;

  auto* curr_result = curr_frame->GetTrackingResultPtr();
  auto& curr_uvs    = curr_result->GetUvs(kLeftCam);

  const int         grid_rows = std::max(1, SVOConfig::feature_grid_rows);
  const int         grid_cols = std::max(1, SVOConfig::feature_grid_cols);
  const int         cell_w = std::max(1, curr_frame->GetImage(kLeftCam).cols / grid_cols);
  const int         cell_h = std::max(1, curr_frame->GetImage(kLeftCam).rows / grid_rows);
  std::vector<bool> cell_has_feature(grid_rows * grid_cols, false);

  for (const auto& uv : curr_uvs) {
    const int col = std::min(static_cast<int>(uv.x / cell_w), grid_cols - 1);
    const int row = std::min(static_cast<int>(uv.y / cell_h), grid_rows - 1);
    cell_has_feature[row * grid_cols + col] = true;
  }

  for (int row = 0; row < grid_rows; ++row) {
    for (int col = 0; col < grid_cols; ++col) {
      if (cell_has_feature[row * grid_cols + col]) {
        continue;
      }
      const int x0 = col * cell_w;
      const int y0 = row * cell_h;
      const int x1 = (col == grid_cols - 1) ? curr_frame->GetImage(kLeftCam).cols
                                            : (col + 1) * cell_w;
      const int y1 = (row == grid_rows - 1) ? curr_frame->GetImage(kLeftCam).rows
                                            : (row + 1) * cell_h;
      if (x1 <= x0 || y1 <= y0) {
        continue;
      }

      const cv::Rect            roi(x0, y0, x1 - x0, y1 - y0);
      std::vector<cv::KeyPoint> keypoints;
      cv::FAST(curr_frame->GetImage(kLeftCam)(roi),
               keypoints,
               SVOConfig::fast_threshold,
               true);
      if (keypoints.empty()) {
        continue;
      }

      std::nth_element(keypoints.begin(),
                       keypoints.begin(),
                       keypoints.end(),
                       [](const cv::KeyPoint& a, const cv::KeyPoint& b) {
                         return a.response > b.response;
                       });
      auto&       keypoint = keypoints.front();
      cv::Point2f uv       = keypoint.pt;
      uv.x += static_cast<float>(x0);
      uv.y += static_cast<float>(y0);

      curr_result->AddFeature(kLeftCam, uv, next_feature_id_++);
    }
  }
}

void OpticalFlow::Run(std::atomic<bool>& running) {
  std::shared_ptr<Frame> curr_frame;
  while (running.load(std::memory_order_acquire)) {
    if (!in_queue_.try_pop(curr_frame)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    if (!curr_frame) {
      continue;
    }

    auto* curr_result = curr_frame->GetTrackingResultPtr();

    const size_t prev_left_size = (prev_frame_ && prev_frame_->GetTrackingResultPtr())
                                    ? prev_frame_->GetTrackingResultPtr()->GetSize(0)
                                    : 0;
    const int    grid_rows      = std::max(1, SVOConfig::feature_grid_rows);
    const int    grid_cols      = std::max(1, SVOConfig::feature_grid_cols);
    const size_t expected_left  = (prev_left_size >> 1)
                                 + static_cast<size_t>(grid_rows * grid_cols);
    curr_result->Reserve(0, expected_left);
    curr_result->Reserve(1, expected_left);

    PrepareImagesAndPyramids(curr_frame);

    TrackMono(curr_frame);
    DetectFeatures(curr_frame);
    if (kCamNum == 2) {
      TrackStereo(curr_frame);
    }
    prev_frame_ = curr_frame;
    out_queue_.push(curr_frame);
  }
}
}  // namespace omni_slam
