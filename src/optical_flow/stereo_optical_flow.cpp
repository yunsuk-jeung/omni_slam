

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
#include "stereo_optical_flow.hpp"

namespace omni_slam {
namespace {
bool IsPointInImage(const cv::Point2f& pt, const cv::Mat& image) {
  return pt.x >= 0.0f && pt.y >= 0.0f && pt.x < image.cols && pt.y < image.rows;
}

}  // namespace

TrackingResult::TrackingResult() = default;

TrackingResult::TrackingResult(std::array<cv::Mat, kCamNum>& images) {
  images_ = images;
}

TrackingResult::TrackingResult(const TrackingResult& src) {
  ids_    = src.ids_;
  uvs_    = src.uvs_;
  id_idx_ = src.id_idx_;
}

TrackingResult::~TrackingResult() = default;

size_t TrackingResult::Size(size_t cam_idx) const {
  return ids_[cam_idx].size();
}

void TrackingResult::Clear() {
  for (size_t i = 0; i < kCamNum; ++i) {
    ids_[i].clear();
    uvs_[i].clear();
    id_idx_[i].clear();
  }
}

void TrackingResult::Reserve(size_t cam_idx, size_t size) {
  const auto idx = cam_idx;
  ids_[idx].reserve(size);
  uvs_[idx].reserve(size);
}

void TrackingResult::AddFeature(size_t cam_idx, const cv::Point2f& uv, int64_t id) {
  ids_[cam_idx].push_back(id);
  uvs_[cam_idx].push_back(uv);
  id_idx_[cam_idx][id] = ids_[cam_idx].size() - 1;
}

StereoOpticalFlow::StereoOpticalFlow(
  tbb::concurrent_queue<std::array<cv::Mat, kCamNum>>&    in_queue,
  tbb::concurrent_queue<std::shared_ptr<TrackingResult>>& out_queue)
  : in_queue_(in_queue)
  , out_queue_(out_queue)
  , prev_result_{nullptr}
  , next_feature_id_{0} {
  if (SVOConfig::equalize_histogram) {
    clahe_ = cv::createCLAHE(SVOConfig::clahe_clip_limit,
                             cv::Size(SVOConfig::clahe_tile_size,
                                      SVOConfig::clahe_tile_size));
  }
}

void StereoOpticalFlow::PrepareImagesAndPyramids(
  const std::array<cv::Mat, kCamNum>& images,
  TrackingResult*                     curr_result) {
  for (size_t i = 0; i < kCamNum; i++) {
    auto& gray = curr_result->Image(i);

    if (images[i].channels() == 1) {
      gray = images[i].clone();
    }
    else {
      cv::cvtColor(images[i], gray, cv::COLOR_BGR2GRAY);
    }

    if (SVOConfig::equalize_histogram && clahe_) {
      clahe_->apply(gray, gray);
    }
    const cv::Point2i patch(SVOConfig::optical_flow_patch_size,
                            SVOConfig::optical_flow_patch_size);
    cv::buildOpticalFlowPyramid(curr_result->Image(i),
                                curr_result->ImagePyramid(i),
                                patch,
                                SVOConfig::max_pyramid_level,
                                true,
                                cv::BORDER_REFLECT_101,
                                cv::BORDER_CONSTANT);
  }
}

void StereoOpticalFlow::TrackLeft(TrackingResult* curr_result) {
  constexpr size_t kLeftCam = 0;
  if (!prev_result_) {
    return;
  }

  const auto& prev_ids = prev_result_->Ids(kLeftCam);
  const auto& prev_uvs = prev_result_->Uvs(kLeftCam);
  if (prev_uvs.empty()) {
    return;
  }

  std::vector<cv::Point2f> tracked_uvs;
  std::vector<uchar>       status;

  cv::calcOpticalFlowPyrLK(prev_result_->ImagePyramid(kLeftCam),
                           curr_result->ImagePyramid(kLeftCam),
                           prev_uvs,
                           tracked_uvs,
                           status,
                           cv::noArray(),
                           cv::Size(SVOConfig::optical_flow_patch_size,
                                    SVOConfig::optical_flow_patch_size),
                           SVOConfig::max_pyramid_level);

  std::vector<cv::Point2f> reverse_uvs;
  std::vector<uchar>       reverse_status;

  cv::calcOpticalFlowPyrLK(curr_result->ImagePyramid(kLeftCam),
                           prev_result_->ImagePyramid(kLeftCam),
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
    if (!IsPointInImage(tracked_uvs[i], curr_result->Image(kLeftCam))) {
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

void StereoOpticalFlow::TrackStereo(TrackingResult* curr_result) {
  constexpr size_t kLeftCam  = 0;
  constexpr size_t kRightCam = 1;

  const auto& left_ids = curr_result->Ids(kLeftCam);
  const auto& left_uvs = curr_result->Uvs(kLeftCam);
  if (left_uvs.empty()) {
    return;
  }

  std::vector<cv::Point2f> right_uvs;
  std::vector<uchar>       status;

  cv::calcOpticalFlowPyrLK(curr_result->ImagePyramid(kLeftCam),
                           curr_result->ImagePyramid(kRightCam),
                           left_uvs,
                           right_uvs,
                           status,
                           cv::noArray(),
                           cv::Size(SVOConfig::optical_flow_patch_size,
                                    SVOConfig::optical_flow_patch_size),
                           SVOConfig::max_pyramid_level);

  std::vector<cv::Point2f> reverse_uvs;
  std::vector<uchar>       reverse_status;

  cv::calcOpticalFlowPyrLK(curr_result->ImagePyramid(kRightCam),
                           curr_result->ImagePyramid(kLeftCam),
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
    if (!IsPointInImage(right_uvs[i], curr_result->Image(kRightCam))) {
      continue;
    }
    curr_result->AddFeature(kRightCam, right_uvs[i], left_ids[i]);
  }
}

void StereoOpticalFlow::DetectFeatures(TrackingResult* curr_result) {
  constexpr size_t kLeftCam = 0;
  auto&            curr_uvs = curr_result->Uvs(kLeftCam);

  const int         grid_rows = std::max(1, SVOConfig::feature_grid_rows);
  const int         grid_cols = std::max(1, SVOConfig::feature_grid_cols);
  const int         cell_w = std::max(1, curr_result->Image(kLeftCam).cols / grid_cols);
  const int         cell_h = std::max(1, curr_result->Image(kLeftCam).rows / grid_rows);
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
      const int x1 = (col == grid_cols - 1) ? curr_result->Image(kLeftCam).cols
                                            : (col + 1) * cell_w;
      const int y1 = (row == grid_rows - 1) ? curr_result->Image(kLeftCam).rows
                                            : (row + 1) * cell_h;
      if (x1 <= x0 || y1 <= y0) {
        continue;
      }

      const cv::Rect            roi(x0, y0, x1 - x0, y1 - y0);
      std::vector<cv::KeyPoint> keypoints;
      cv::FAST(curr_result->Image(kLeftCam)(roi),
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

void StereoOpticalFlow::Run(std::atomic<bool>& running) {
  std::array<cv::Mat, 2> images;
  while (running.load(std::memory_order_acquire)) {
    if (!in_queue_.try_pop(images)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    auto curr_result = std::make_shared<TrackingResult>();

    const size_t prev_left_size = prev_result_ ? prev_result_->Size(0) : 0;
    const int    grid_rows      = std::max(1, SVOConfig::feature_grid_rows);
    const int    grid_cols      = std::max(1, SVOConfig::feature_grid_cols);
    const size_t expected_left  = (prev_left_size >> 1)
                                 + static_cast<size_t>(grid_rows * grid_cols);
    curr_result->Reserve(0, expected_left);
    curr_result->Reserve(1, expected_left);

    PrepareImagesAndPyramids(images, curr_result.get());

    TrackLeft(curr_result.get());
    // TrackStereo(curr_result.get());
    DetectFeatures(curr_result.get());

    prev_result_ = curr_result;
    out_queue_.push(curr_result);
  }
}
}  // namespace omni_slam
