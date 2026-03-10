#include <chrono>
#include <cmath>
#include <limits>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "utils/logger.hpp"
#include "utils/timer.hpp"
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
  , status_{Status::Initializing}
  , make_keyframe_{true}
  , new_keyframe_after_{100}
  , created_map_point_nums_{}
  , result_mutex_{}
  , has_result_{false}
  , latest_result_{} {
  sliding_window_ = std::make_unique<SlidingWindow>();
  estimator_      = std::make_unique<VOEstimator>();
}

StereoVO::~StereoVO() {}

bool StereoVO::Setup(const std::string& config_path) {
  Logger::Info("Initializing VO Pipeline");
  if (config_path.empty()) {
    Logger::Warn("Empty config path for VO pipeline");
    return false;
  }

  SVOConfig::ParseConfig(config_path);
  Logger::Info("Loaded VO config: {}", config_path.c_str());

  sliding_window_->SetMaxSize(SVOConfig::max_keyframe_size + 1u);
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

    Process(frame);
  }
}

void StereoVO::Process(std::shared_ptr<Frame>& frame) {
  if (!frame || !frame->GetTrackingResultPtr()) {
    return;
  }

  switch (status_) {
  case Status::Initializing:
    if (Initialize(frame)) {
      status_ = Status::Tracking;
    }
    break;
  case Status::Tracking:
    Track(frame);
    break;
  default:
    // TODO: Handle Status::Lost and other states here.
    break;
  }

  // build result
  {
    ScopedTimer                 timer("build result");
    OdometryResult              result = BuildOdometryResult(frame);
    std::lock_guard<std::mutex> lock(result_mutex_);
    latest_result_ = std::move(result);
    has_result_    = true;
  }

  Statistics::reportAll();
}

bool StereoVO::Initialize(std::shared_ptr<Frame>& frame) {
  ScopedTimer loop_timer("stereo_vo::Initialize");

  sliding_window_->AddFrame(frame);

  UpdateFrameObservations(frame);

  int created_map_point_num = InitializeMapPoints(frame);

  if (created_map_point_num < SVOConfig::min_init_map_point_count) {
    Logger::Warn(
      "StereoVO initialization failed at frame {} (map points: {}, required: {}), "
      "resetting sliding window",
      frame->GetId(),
      created_map_point_num,
      SVOConfig::min_init_map_point_count);
    sliding_window_->Clear();
    created_map_point_nums_.clear();
    new_keyframe_after_ = SVOConfig::new_keyframe_after + 1;
    return false;
  }

  created_map_point_nums_[frame->GetId()] = created_map_point_num;
  frame->SetKeyframe();
  sliding_window_->MarkKeyframe(frame->GetId());

  new_keyframe_after_ = 0;

  Logger::Info("stereoVO initialized at frame {}, created_map_point",
               frame->GetId(),
               sliding_window_->GetMapPointCount());

  return true;
}

void StereoVO::Track(std::shared_ptr<Frame>& frame) {
  ScopedTimer loop_timer("loop_timer_tracking");

  // use last frame's pose for initial pose
  const auto& frame_ids = sliding_window_->GetFrameIds();
  if (!frame_ids.empty()) {
    const uint64_t         latest_id    = *frame_ids.rbegin();
    std::shared_ptr<Frame> latest_frame = sliding_window_->GetFrame(latest_id);
    if (latest_frame) {
      frame->GetTwb() = latest_frame->GetTwb();
    }
  }

  sliding_window_->AddFrame(frame);

  float connect_mp_ratio = UpdateFrameObservations(frame);

  if (connect_mp_ratio < SVOConfig::keyframe_min_mp_ratio) {
    make_keyframe_ = true;
  }

  if (make_keyframe_ && new_keyframe_after_ > SVOConfig::new_keyframe_after) {
    int created_map_point_num = InitializeMapPoints(frame);

    created_map_point_nums_[frame->GetId()] = created_map_point_num;
    frame->SetKeyframe();

    new_keyframe_after_ = 0;
    sliding_window_->MarkKeyframe(frame->GetId());
  }
  else {
    ++new_keyframe_after_;
  }

  // single frame pose estimation
  {
    ScopedTimer timer("optimize_frame");
    VOEstimator::OptimizeSingleFrame(frame, this->sliding_window_.get());
  }

  // sliding window bundle
  {
    ScopedTimer timer("optimize_window");
    estimator_->OptimizeWindow(this->sliding_window_.get());
  }

  // select marginal frames
  std::set<uint64_t> marginal_none_keyframe_ids;
  std::set<uint64_t> marginal_keyframe_ids;
  SelectMarginalFrames(marginal_none_keyframe_ids, marginal_keyframe_ids);

  // remove none keyframes before marginalize
  sliding_window_->RemoveFrames(marginal_none_keyframe_ids);

  // marginalize
  {
    ScopedTimer timer("marginalize ");
    estimator_->Marginalize(this->sliding_window_.get(), marginal_keyframe_ids);
  }

  // remove keyframe
  {
    ScopedTimer timer("remove keyframe");
    sliding_window_->RemoveFrames(marginal_keyframe_ids);

    for (const auto& id : marginal_keyframe_ids) {
      created_map_point_nums_.erase(id);
    }
  }
}

float StereoVO::UpdateFrameObservations(std::shared_ptr<Frame>& frame) {
  TrackingResult* tracking_result = frame->GetTrackingResultPtr();
  const size_t    kCamNum         = frame->GetCamNum();
  size_t          connected       = 0;

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

  size_t kpt_num            = frame->GetTrackingResultPtr()->GetSize(0);
  float  connected_mp_ratio = kpt_num > 0 ? static_cast<float>(connected)
                                             / static_cast<float>(kpt_num)
                                          : 1.0f;
  LogD("frame {}, connected map point ratio : {} = {} /{}",
       frame->GetId(),
       connected_mp_ratio,
       connected,
       kpt_num);

  return connected_mp_ratio;
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
        mp->GetBearing()        = t_c0_x.head<3>();
        mp->GetInvDist()        = t_c0_x[3];
        mp->GetHostFrameCamId() = frame_cam_id0;
        mp->SetStatus(MapPoint::Status::TRACKING);
        erase_mp_ids.insert(mp_id);
        sliding_window_->AddMapPoint(mp);
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

  return init_count;
}

void StereoVO::SelectMarginalFrames(std::set<uint64_t>& marginal_none_keyframe_ids,
                                    std::set<uint64_t>& marginal_keyframe_ids) {
  marginal_none_keyframe_ids.clear();
  marginal_keyframe_ids.clear();

  const auto& frame_ids    = sliding_window_->GetFrameIds();
  const auto& keyframe_ids = sliding_window_->GetKeyframeIds();

  if (frame_ids.empty()) {
    return;
  }

  const uint64_t latest_id = *frame_ids.rbegin();
  for (const auto id : frame_ids) {
    if (id == latest_id) {
      continue;
    }
    if (keyframe_ids.find(id) != keyframe_ids.end()) {
      continue;
    }
    marginal_none_keyframe_ids.insert(id);
  }

  if (keyframe_ids.size() <= SVOConfig::max_keyframe_size) {
    return;
  }

  std::map<uint64_t, int> connected_map_points;
  std::shared_ptr<Frame>  latest_frame = sliding_window_->GetFrame(latest_id);
  if (latest_frame) {
    const auto& obs = latest_frame->GetObservations().front();
    for (const auto& [mp_id, _] : obs) {
      auto mp = sliding_window_->GetMapPoint(mp_id);
      if (!mp || mp->GetStatus() < MapPoint::Status::TRACKING) {
        continue;
      }

      connected_map_points[mp->GetHostFrameCamId().frame_id]++;
    }
  }

  auto kf_ids = keyframe_ids;
  while (kf_ids.size() > SVOConfig::max_keyframe_size) {
    if (kf_ids.size() <= 2) {
      break;
    }

    bool     selected   = false;
    uint64_t id_to_marg = std::numeric_limits<uint64_t>::max();

    auto end_minus_2 = std::prev(kf_ids.end(), 2);
    for (auto it = kf_ids.begin(); it != end_minus_2; ++it) {
      const uint64_t kf_id   = *it;
      const int      count   = connected_map_points[kf_id];
      int            created = created_map_point_nums_[kf_id];

      const double ratio = static_cast<double>(count) / static_cast<double>(created);
      if (count == 0
          || float(count) / float(created)
               < float(SVOConfig::marg_feature_connection_ratio)) {
        id_to_marg = kf_id;
        selected   = true;
        break;
      }
    }

    if (!selected) {
      const uint64_t last_kf_id   = *kf_ids.rbegin();
      uint64_t       min_score_id = std::numeric_limits<uint64_t>::max();
      double         min_score    = std::numeric_limits<double>::max();

      for (auto it1 = kf_ids.begin(); it1 != end_minus_2; ++it1) {
        std::shared_ptr<Frame> frame_i = sliding_window_->GetFrame(*it1);
        if (!frame_i) {
          continue;
        }
        double denom = 0.0;
        for (auto it2 = kf_ids.begin(); it2 != end_minus_2; ++it2) {
          std::shared_ptr<Frame> frame_j = sliding_window_->GetFrame(*it2);
          if (!frame_j) {
            continue;
          }
          denom += 1.0
                   / ((frame_i->GetTwb().translation() - frame_j->GetTwb().translation())
                        .norm()
                      + 1e-5);
        }

        std::shared_ptr<Frame> last_kf = sliding_window_->GetFrame(last_kf_id);
        if (!last_kf) {
          continue;
        }
        double score = std::sqrt((frame_i->GetTwb().translation()
                                  - last_kf->GetTwb().translation())
                                   .norm())
                       * denom;

        if (score < min_score) {
          min_score_id = *it1;
          min_score    = score;
        }
      }
      id_to_marg = min_score_id;
    }

    if (id_to_marg == std::numeric_limits<uint64_t>::max()) {
      break;
    }

    kf_ids.erase(id_to_marg);
    marginal_keyframe_ids.insert(id_to_marg);
  }
}

OdometryResult StereoVO::BuildOdometryResult(const std::shared_ptr<Frame>& frame) {
  TrackingResult* tracking_result = frame->GetTrackingResultPtr();

  OdometryResult result;
  result.frame_id     = frame->GetId();
  result.timestamp_ns = frame->GetTimestampNs();

  const size_t cam_num = frame->GetCamNum();
  result.T_b_c.reserve(cam_num);
  for (size_t i = 0; i < cam_num; i++) {
    result.T_b_c.push_back(frame->GetTbc(i));
  }

  const auto& window_ids = sliding_window_->GetFrameIds();
  result.window_frame_ids.assign(window_ids.begin(), window_ids.end());
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

    std::shared_ptr<Frame> host_frame = sliding_window_->GetFrame(
      mp->GetHostFrameCamId().frame_id);
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

}  // namespace omni_slam
