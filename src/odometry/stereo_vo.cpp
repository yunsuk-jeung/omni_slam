#include <chrono>
#include <cmath>
#include <set>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "utils/logger.hpp"
#include "config/svo_config.hpp"
#include "database/Frame.hpp"
#include "database/MapPoint.hpp"
#include "feature_tracking/optical_flow.hpp"
#include "optimizer/geometry.hpp"
#include "optimizer/vo_estimator.hpp"
#include "odometry/sliding_window.hpp"
#include "odometry/stereo_vo.hpp"
#include "stereo_vo.hpp"

namespace omni_slam {
StereoVO::StereoVO()
  : frame_queue_{}
  , result_queue_{}
  , optical_flow_{nullptr}
  , running_{false}
  , make_keyframe_{true}
  , new_keyframe_after_{100}
  , created_map_point_nums_{}
  , result_mutex_{}
  , has_result_{false}
  , latest_result_{} {
  sliding_window_ = std::make_unique<SlidingWindow>();
}

StereoVO::~StereoVO() {}

bool StereoVO::Initialize(const std::string& config_path) {
  Logger::Info("Initializing VO Pipeline");
  if (config_path.empty()) {
    Logger::Warn("Empty config path for VO pipeline");
    return false;
  }

  SVOConfig::ParseConfig(config_path);
  Logger::Info("Loaded VO config: {}", config_path.c_str());

  sliding_window_->SetMaxSize(SVOConfig::max_window);
  optical_flow_ = std::make_unique<OpticalFlow>(kCamNum, frame_queue_, result_queue_);

  return true;
}

void StereoVO::Run() {
  Logger::Info("Running VO Pipeline");
  running_.store(true, std::memory_order_release);

  optical_flow_thread_ = std::thread(&StereoVO::OpticalFlowLoop, this);
  estimator_thread_    = std::thread(&StereoVO::EstimatorLoop, this);
}

void StereoVO::Shutdown() {
  Logger::Info("Shutting down VO Pipeline");
  running_.store(false, std::memory_order_release);

  if (optical_flow_thread_.joinable()) {
    optical_flow_thread_.join();
  }
  if (estimator_thread_.joinable()) {
    estimator_thread_.join();
  }
}

void StereoVO::OnCameraFrame(int64_t                             timestamp_ns,
                             const std::vector<cv::Mat>&         images,
                             const std::vector<CameraParameter>& camera_parameters) {
  if (images.empty() || images[0].empty()) {
    Logger::Warn("Received camera frame with empty left image");
    return;
  }

  auto frame = std::make_shared<Frame>(timestamp_ns, images, camera_parameters);
  frame_queue_.push(frame);
}

void StereoVO::OpticalFlowLoop() {
  optical_flow_->Run(running_);
}

void StereoVO::EstimatorLoop() {
  std::shared_ptr<Frame> frame;
  while (running_.load(std::memory_order_acquire)) {
    if (!result_queue_.try_pop(frame)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    if (!frame || !frame->GetTrackingResultPtr()) {
      continue;
    }

    sliding_window_->AddFrame(frame);

    TrackingResult* tracking_result = frame->GetTrackingResultPtr();
    const size_t    kCamNum         = frame->GetCamNum();

    size_t connected = 0;

    for (size_t i = 0; i < kCamNum; ++i) {
      auto&                        ids = tracking_result->GetIds(i);
      auto&                        uvs = tracking_result->GetUvs(i);
      std::vector<Eigen::Vector3d> bearings;
      std::vector<bool>            valid;

      frame->GetCam(i)->Unproject(uvs, bearings, valid);

      const auto& point_num = tracking_result->GetSize(i);
      for (size_t j = 0; j < point_num; j++) {
        if (!valid[j]) {
          continue;
        }
        const auto&     id = ids[j];
        std::shared_ptr mp = sliding_window_->GetMapPoint(ids[j]);
        if (mp) {
          if (i == 0) {
            ++connected;
          }
        }
        else {
          mp = sliding_window_->GetOrCreateMapPointCandidate(id);
        }

        frame->AddObservation(i, id, bearings[j]);

        FrameCamId frame_cam_id{frame->GetId(), i};
        mp->AddObservation(frame_cam_id, bearings[j]);
      }
    }

    size_t kpt_num = frame->GetTrackingResultPtr()->GetSize(0);
    float  ratio   = kpt_num > 0
                       ? static_cast<float>(connected) / static_cast<float>(kpt_num)
                       : 1.0f;

    if (ratio < SVOConfig::keyframe_min_mp_ratio) {
      LogD("{}th frame, mp ratio : {} = {} /{}",
           frame->GetId(),
           ratio,
           connected,
           kpt_num);
      make_keyframe_ = true;
    }

    if (make_keyframe_ && new_keyframe_after_ > SVOConfig::new_keyframe_after) {
      int created_map_point_num = InitializeMapPoints(frame);

      if (created_map_point_num > 0) {
        created_map_point_nums_[frame->GetId()] = created_map_point_num;
        frame->SetKeyframe();
      }
    }

    if (frame->IsKeyframe()) {
      new_keyframe_after_ = 0;
    }
    else {
      ++new_keyframe_after_;
    }

    VOEstimator::OptimizeSingleFrame(frame, this->sliding_window_.get());

    OdometryResult result = BuildOdometryResult(frame, tracking_result);

    {
      std::lock_guard<std::mutex> lock(result_mutex_);
      latest_result_ = std::move(result);
      has_result_    = true;
    }
  }
}

OdometryResult StereoVO::BuildOdometryResult(const std::shared_ptr<Frame>& frame,
                                             TrackingResult* tracking_result) {
  OdometryResult result;
  result.timestamp_ns = frame->GetTimestampNs();

  const size_t cam_num = frame->GetCamNum();
  result.T_b_c.reserve(cam_num);
  for (size_t i = 0; i < cam_num; i++) {
    result.T_b_c.push_back(frame->GetTbc(i));
  }

  result.window_frame_ids = sliding_window_->GetFrameIds();
  result.T_w_b_window.reserve(result.window_frame_ids.size());
  for (const auto frame_id : result.window_frame_ids) {
    std::shared_ptr<Frame> window_frame = sliding_window_->GetFrame(frame_id);
    if (window_frame) {
      result.T_w_b_window.push_back(window_frame->GetTwb());
    }
    else {
      result.T_w_b_window.emplace_back();
    }
  }

  result.images.reserve(cam_num);
  result.tracking.ids.resize(cam_num);
  result.tracking.uvs.resize(cam_num);
  for (size_t i = 0; i < cam_num; ++i) {
    result.images.push_back(frame->GetImage(i));
    result.tracking.ids[i] = tracking_result->GetIds(i);
    result.tracking.uvs[i] = tracking_result->GetUvs(i);
  }

  const auto& map_points = sliding_window_->GetMapPoints();
  result.map_points.reserve(map_points.size());
  result.map_point_uvs.resize(cam_num);
  std::vector<CameraModelBase*> cams(cam_num, nullptr);
  std::vector<Sophus::SE3d>     T_c_w(cam_num);
  std::vector<cv::Size>         img_sizes(cam_num);
  for (size_t i = 0; i < cam_num; ++i) {
    cams[i]            = frame->GetCam(i);
    T_c_w[i]           = frame->GetTwc(i).inverse();
    const cv::Mat& img = frame->GetImage(i);
    img_sizes[i]       = cv::Size(img.cols, img.rows);
    result.map_point_uvs[i].reserve(map_points.size());
  }
  for (const auto& [mp_id, mp] : map_points) {
    const double inv_dist = mp->GetInvDist();
    if (inv_dist <= 0.0) {
      continue;
    }

    std::shared_ptr<Frame> host_frame = sliding_window_->GetFrame(mp->GetHostFrameId());
    if (!host_frame) {
      continue;
    }

    const Eigen::Vector3d bearing = mp->GetBearing();
    const Eigen::Vector3d p_c     = bearing / inv_dist;
    const Eigen::Vector3d p_w     = host_frame->GetTwc(0) * p_c;

    Eigen::Vector4f packed;
    packed << static_cast<float>(p_w.x()), static_cast<float>(p_w.y()),
      static_cast<float>(p_w.z()), static_cast<float>(mp_id);
    result.map_points.push_back(packed);

    for (size_t i = 0; i < cam_num; ++i) {
      if (!cams[i]) {
        continue;
      }
      const Eigen::Vector3d p_c = T_c_w[i] * p_w;
      if (!p_c.array().isFinite().all() || p_c.z() <= 0.0) {
        continue;
      }
      const cv::Point2d uv       = cams[i]->Project(p_c);
      const cv::Size&   img_size = img_sizes[i];
      if ((img_size.width == 0 && img_size.height == 0)
          || (uv.x >= 0.0 && uv.y >= 0.0 && uv.x < img_size.width
              && uv.y < img_size.height)) {
        result.map_point_uvs[i].emplace_back(static_cast<float>(uv.x),
                                             static_cast<float>(uv.y));
      }
    }
  }

  return result;
}

bool StereoVO::FetchResult(OdometryResult& out) {
  std::lock_guard<std::mutex> lock(result_mutex_);
  if (!has_result_) {
    return false;
  }
  out = latest_result_;
  return true;
}

int StereoVO::InitializeMapPoints(std::shared_ptr<Frame>& frame) {
  // triangulate
  auto& candidates = sliding_window_->GetMapPointCandidates();

  int init_count = 0;
  int old_count  = 0;
  int try_count  = candidates.size();

  FrameCamId         frame_cam_id0{frame->GetId(), 0};
  std::set<uint64_t> erase_mp_ids;

  std::vector<std::shared_ptr<MapPoint>> new_map_points;

  // add map points in SlidingWindow
  for (auto& [mp_id, mp] : candidates) {
    auto& frame_id_to_bearing = mp->GetObservation();

    if (frame_id_to_bearing.count(frame_cam_id0) == 0) {
      old_count++;
      erase_mp_ids.insert(mp_id);
      continue;
    }

    Eigen::Vector3d bearing0 = frame_id_to_bearing[frame_cam_id0];

    bool success = false;
    for (auto& [frame_cam_id1, bearing1] : frame_id_to_bearing) {
      if (frame_cam_id0 == frame_cam_id1) {
        continue;
      }

      std::shared_ptr<Frame> frame1 = sliding_window_->GetFrame(frame_cam_id1.frame_id);

      auto T_w_c0  = frame->GetTwc(frame_cam_id0.cam_id);
      auto T_w_c1  = frame1->GetTwc(frame_cam_id1.cam_id);
      auto T_c1_c0 = T_w_c1.inverse() * T_w_c0;

      if (T_c1_c0.translation().squaredNorm() < SVOConfig::triangulation_dist_threshold) {
        continue;
      }

      Eigen::Vector4d t_c0_x = Geometry::triangulate(bearing0, bearing1, T_c1_c0);
      if (t_c0_x.array().isFinite().all() && t_c0_x[3] > 0 && t_c0_x[3] < 3.0) {
        mp->GetBearing()     = t_c0_x.head<3>();
        mp->GetInvDist()     = t_c0_x[3];
        mp->GetHostFrameId() = frame->GetId();
        mp->SetStatus(MapPoint::Status::TRACKING);
        erase_mp_ids.insert(mp_id);
        sliding_window_->AddMapPoint(mp);
        new_map_points.push_back(mp);
        init_count++;
        break;
      }
    }
  }

  for (auto& id : erase_mp_ids) {
    candidates.erase(id);
  }

  LogD("init : {}, oldCount :{}, cand : {} -> {}",
       init_count,
       old_count,
       try_count,
       candidates.size());

  // if (!new_map_points.empty()) {
  //   const cv::Mat& src = frame->GetImage(0);
  //   if (!src.empty()) {
  //     cv::Mat vis;
  //     if (src.channels() == 1) {
  //       cv::cvtColor(src, vis, cv::COLOR_GRAY2BGR);
  //     }
  //     else {
  //       src.copyTo(vis);
  //     }

  // auto* cam = frame->GetCam(0);
  // const int width = vis.cols;
  // const int height = vis.rows;

  // int drawn = 0;
  // for (const auto& mp : new_map_points) {
  //   const double inv_dist = mp->GetInvDist();
  //   if (!std::isfinite(inv_dist) || std::abs(inv_dist) <= 1e-12) {
  //     continue;
  //   }
  //   const Eigen::Vector3d p_c0 = mp->GetBearing() / inv_dist;
  //   if (!p_c0.array().isFinite().all() || p_c0.z() <= 0.0) {
  //     continue;
  //   }
  //   const cv::Point2d uv = cam->Project(p_c0);
  //   if (uv.x < 0.0 || uv.y < 0.0 || uv.x >= width || uv.y >= height) {
  //     continue;
  //   }
  //   cv::circle(vis, uv, 2, cv::Scalar(0, 255, 255), -1, cv::LINE_AA);
  //   ++drawn;
  // }

  // cv::putText(vis,
  //             "init mp proj: " + std::to_string(drawn),
  //             cv::Point(12, 24),
  //             cv::FONT_HERSHEY_SIMPLEX,
  //             0.6,
  //             cv::Scalar(0, 255, 0),
  //             1,
  //             cv::LINE_AA);
  // cv::imshow("init_map_points_proj", vis);
  // cv::waitKey(1);
  // }
  // }

  return init_count;
}
}  // namespace omni_slam
