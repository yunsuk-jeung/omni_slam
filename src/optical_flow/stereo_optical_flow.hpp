#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <vector>
#include <tbb/concurrent_queue.h>
#include <opencv2/core.hpp>

namespace cv {
class CLAHE;
}

namespace omni_slam {

class TrackingResult {
public:
  static constexpr size_t kCamNum = 2;

  TrackingResult();
  TrackingResult(std::array<cv::Mat, kCamNum>& images);
  TrackingResult(const TrackingResult& src);
  ~TrackingResult();

  size_t Size(size_t cam_idx) const;

  void Clear();
  void Reserve(size_t cam_idx, size_t size);

  void AddFeature(size_t cam_idx, const cv::Point2f& uv, int64_t id);

  std::vector<size_t>&            Ids(size_t cam_idx) { return ids_[cam_idx]; }
  const std::vector<size_t>&      Ids(size_t cam_idx) const { return ids_[cam_idx]; }
  std::vector<cv::Point2f>&       Uvs(size_t cam_idx) { return uvs_[cam_idx]; }
  const std::vector<cv::Point2f>& Uvs(size_t cam_idx) const { return uvs_[cam_idx]; }

  std::map<size_t, size_t>&       IdIndex(size_t cam_idx) { return id_idx_[cam_idx]; }
  const std::map<size_t, size_t>& IdIndex(size_t cam_idx) const {
    return id_idx_[cam_idx];
  }
  cv::Mat&              Image(size_t cam_idx) { return images_[cam_idx]; }
  const cv::Mat&        Image(size_t cam_idx) const { return images_[cam_idx]; }
  std::vector<cv::Mat>& ImagePyramid(size_t cam_idx) { return image_pyramids_[cam_idx]; }
  const std::vector<cv::Mat>& ImagePyramid(size_t cam_idx) const {
    return image_pyramids_[cam_idx];
  }

private:
  std::array<std::vector<size_t>, kCamNum>      ids_;
  std::array<std::vector<cv::Point2f>, kCamNum> uvs_;
  std::array<std::map<size_t, size_t>, kCamNum> id_idx_;
  std::array<cv::Mat, kCamNum>                  images_;
  std::array<std::vector<cv::Mat>, kCamNum>     image_pyramids_;
};

class StereoOpticalFlow {
public:
  static constexpr size_t kCamNum = 2;

  StereoOpticalFlow(tbb::concurrent_queue<std::array<cv::Mat, kCamNum>>&    in_queue,
                    tbb::concurrent_queue<std::shared_ptr<TrackingResult>>& out_queue);

  ~StereoOpticalFlow() = default;

  void Run(std::atomic<bool>& running);

private:
  void PrepareImagesAndPyramids(const std::array<cv::Mat, kCamNum>& images,
                                TrackingResult*                     curr_result);
  void TrackLeft(TrackingResult* curr_result);
  void TrackStereo(TrackingResult* curr_result);
  void DetectFeatures(TrackingResult* curr_result);

  tbb::concurrent_queue<std::array<cv::Mat, kCamNum>>&    in_queue_;
  tbb::concurrent_queue<std::shared_ptr<TrackingResult>>& out_queue_;

  cv::Ptr<cv::CLAHE>              clahe_;
  std::shared_ptr<TrackingResult> prev_result_;
  uint64_t                        next_feature_id_;
};

}  // namespace omni_slam
