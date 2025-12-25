

#include <chrono>
#include <thread>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include "config/svo_config.hpp"
#include "stereo_optical_flow.hpp"

namespace omni_slam {

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

void TrackingResult::PushBack(size_t cam_idx, const TrackingResult& kpts) {
  auto idx = ids_[cam_idx].size();
  ids_[cam_idx].insert(ids_[cam_idx].end(),
                       kpts.ids_[cam_idx].begin(),
                       kpts.ids_[cam_idx].end());

  for (; idx < ids_[cam_idx].size(); ++idx) {
    id_idx_[cam_idx][ids_[cam_idx][idx]] = idx;
  }

  uvs_[cam_idx].insert(uvs_[cam_idx].end(),
                       kpts.uvs_[cam_idx].begin(),
                       kpts.uvs_[cam_idx].end());
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

void StereoOpticalFlow::Run(std::atomic<bool>& running) {
  std::array<cv::Mat, 2> images;
  while (running.load(std::memory_order_acquire)) {
    if (!in_queue_.try_pop(images)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    auto curr_result = std::make_shared<TrackingResult>();

    for (size_t i = 0; i < kCamNum; i++) {
      if (images[i].type() != 0) {
        images[i].convertTo(curr_result->Image(i), CV_8UC1);
      }
      else {
        curr_result->Image(i) = images[i];
      }
      if (SVOConfig::equalize_histogram && clahe_) {
        clahe_->apply(curr_result->Image(i), curr_result->Image(i));
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

    if (!prev_result_) {
      prev_result_ = curr_result;
      continue;
    }
    prev_result_ = curr_result;
    out_queue_.push(curr_result);
  }
}
}  // namespace omni_slam
