#include <cmath>
#include <limits>
#include <set>
#include <unordered_map>

#include "config/svo_config.hpp"
#include "database/frame.hpp"
#include "database/map_point.hpp"
#include "feature_tracking/optical_flow.hpp"
#include "odometry/sliding_window.hpp"
#include "omni_slam/stereo_vo.hpp"
#include "optimizer/geometry.hpp"
#include "optimizer/vo_estimator.hpp"
#include "utils/logger.hpp"
#include "utils/timer.hpp"

namespace omni_slam {
// Large sentinel so the first tracked frame always passes the
// new_keyframe_after check before setup() overrides it from config.
static constexpr int kInitialNewKeyframeAfter = 100;

StereoVO::StereoVO()
  : raw_frame_queue_{}
  , tracked_frame_queue_{}
  , optical_flow_{nullptr}
  , running_{false}
  , status_{Status::Initializing}
  , make_keyframe_{true}
  , new_keyframe_after_{kInitialNewKeyframeAfter}
  , created_map_point_nums_{}
  , result_mutex_{}
  , has_result_{false}
  , latest_result_{} {
  sliding_window_ = std::make_unique<SlidingWindow>();
  estimator_      = std::make_unique<VOEstimator>();
}

StereoVO::~StereoVO() {}

bool StereoVO::setup(const std::string& config_path) {
  Logger::info("Initializing VO Pipeline");
  if (config_path.empty()) {
    Logger::warn("Empty config path for VO pipeline");
    return false;
  }

  SVOConfig::parse_config(config_path);
  Logger::info("Loaded VO config: {}", config_path.c_str());

  sliding_window_->max_size(SVOConfig::max_keyframe_size + 1u);
  optical_flow_ = std::make_unique<OpticalFlow>(kCamNum);

  return true;
}

void StereoVO::run() {
  Logger::info("Running VO Pipeline");
  running_.store(true, std::memory_order_release);

  optical_flow_thread_ = std::thread(&StereoVO::optical_flow_loop, this);
  estimator_thread_    = std::thread(&StereoVO::estimator_loop, this);
}

void StereoVO::shutdown() {
  Logger::info("Shutting down VO Pipeline");
  running_.store(false, std::memory_order_release);
  raw_frame_queue_.close();
  tracked_frame_queue_.close();

  if (optical_flow_thread_.joinable()) {
    optical_flow_thread_.join();
  }
  if (estimator_thread_.joinable()) {
    estimator_thread_.join();
  }
}

void StereoVO::on_camera_frame(
  int64_t                             timestamp_ns,
  const std::vector<cv::Mat>&         images,
  const std::vector<CameraParameter>& camera_parameters) {
  if (images.empty() || images[0].empty()) {
    Logger::warn("Received camera frame with empty left image");
    return;
  }

  auto frame = std::make_shared<Frame>(timestamp_ns, images, camera_parameters);
  raw_frame_queue_.push(frame);
}

void StereoVO::optical_flow_loop() {
  std::shared_ptr<Frame> frame;
  while (running_.load(std::memory_order_acquire)) {
    if (!raw_frame_queue_.wait()) {
      break;
    }
    if (!raw_frame_queue_.try_pop(frame)) {
      continue;
    }
    optical_flow_->process(frame);
    tracked_frame_queue_.push(frame);
  }
}

void StereoVO::estimator_loop() {
  std::shared_ptr<Frame> frame;
  while (running_.load(std::memory_order_acquire)) {
    if (!tracked_frame_queue_.wait()) {
      break;
    }
    if (!tracked_frame_queue_.try_pop(frame)) {
      continue;
    }
    process(frame);
  }
}

void StereoVO::process(std::shared_ptr<Frame>& frame) {
  if (!frame || !frame->tracking_result_ptr()) {
    return;
  }

  switch (status_) {
  case Status::Initializing:
    if (initialize(frame)) {
      status_ = Status::Tracking;
    }
    break;
  case Status::Tracking:
    track(frame);
    break;
  default:
    // TODO: Handle Status::Lost and other states here.
    break;
  }

  // build result
  {
    ScopedTimer                 timer("build result");
    OdometryResult              result = build_odometry_result(frame);
    std::lock_guard<std::mutex> lock(result_mutex_);
    latest_result_ = std::move(result);
    has_result_    = true;
  }

  Statistics::report_all();
}

bool StereoVO::initialize(std::shared_ptr<Frame>& frame) {
  ScopedTimer loop_timer("stereo_vo::Initialize");

  sliding_window_->add_frame(frame);

  update_frame_observations(frame);

  int created_map_point_num = initialize_map_points(frame);

  if (created_map_point_num < SVOConfig::min_init_map_point_count) {
    Logger::warn("StereoVO initialization failed at frame {} (map points: {}, "
                 "required: {}), "
                 "resetting sliding window",
                 frame->id(),
                 created_map_point_num,
                 SVOConfig::min_init_map_point_count);
    sliding_window_->clear();
    created_map_point_nums_.clear();
    new_keyframe_after_ = SVOConfig::new_keyframe_after + 1;
    return false;
  }

  created_map_point_nums_[frame->id()] = created_map_point_num;
  frame->keyframe();
  sliding_window_->mark_keyframe(frame->id());

  new_keyframe_after_ = 0;

  Logger::info("stereoVO initialized at frame {}, created map points: {}",
               frame->id(),
               sliding_window_->map_point_count());

  return true;
}

void StereoVO::track(std::shared_ptr<Frame>& frame) {
  ScopedTimer loop_timer("loop_timer_tracking");

  // use last frame's pose for initial pose
  const auto&            frame_ids    = sliding_window_->frame_ids();
  const uint64_t         latest_id    = *frame_ids.rbegin();
  std::shared_ptr<Frame> latest_frame = sliding_window_->frame(latest_id);
  if (latest_frame) {
    frame->twb() = latest_frame->twb();
  }

  sliding_window_->add_frame(frame);

  float connect_mp_ratio = update_frame_observations(frame);

  if (connect_mp_ratio < SVOConfig::keyframe_min_mp_ratio) {
    make_keyframe_ = true;
  }

  if (make_keyframe_ && new_keyframe_after_ > SVOConfig::new_keyframe_after) {
    int created_map_point_num = initialize_map_points(frame);

    created_map_point_nums_[frame->id()] = created_map_point_num;
    frame->keyframe();

    new_keyframe_after_ = 0;
    sliding_window_->mark_keyframe(frame->id());
  }
  else {
    ++new_keyframe_after_;
  }

  // single frame pose estimation
  {
    ScopedTimer timer("optimize_frame");
    VOEstimator::optimize_single_frame(frame, this->sliding_window_.get());
  }

  // sliding window bundle
  {
    ScopedTimer timer("optimize_window");
    estimator_->optimize_window(this->sliding_window_.get());
  }

  // select marginal frames
  std::set<uint64_t> marginal_none_keyframe_ids;
  std::set<uint64_t> marginal_keyframe_ids;
  select_marginal_frames(marginal_none_keyframe_ids, marginal_keyframe_ids);

  // remove none keyframes before marginalize
  sliding_window_->remove_frames(marginal_none_keyframe_ids);

  // marginalize
  {
    ScopedTimer timer("marginalize ");
    estimator_->marginalize(this->sliding_window_.get(), marginal_keyframe_ids);
  }

  // remove keyframe
  {
    ScopedTimer timer("remove keyframe");
    sliding_window_->remove_frames(marginal_keyframe_ids);

    for (const auto& id : marginal_keyframe_ids) {
      created_map_point_nums_.erase(id);
    }
  }
}

float StereoVO::update_frame_observations(std::shared_ptr<Frame>& frame) {
  TrackingResult* tracking_result = frame->tracking_result_ptr();
  const size_t    cam_num         = frame->cam_num();
  size_t          connected       = 0;

  for (size_t i = 0; i < cam_num; ++i) {
    auto&                        ids = tracking_result->ids(i);
    auto&                        uvs = tracking_result->uvs(i);
    std::vector<Eigen::Vector3d> bearings;
    std::vector<bool>            valid;

    frame->cam(i)->unproject(uvs, bearings, valid);

    const size_t point_num = tracking_result->size(i);
    for (size_t j = 0; j < point_num; j++) {
      if (!valid[j]) {
        continue;
      }
      const auto&     id = ids[j];
      std::shared_ptr mp = sliding_window_->map_point(id);
      if (mp) {
        if (i == 0) {
          ++connected;
        }
      }
      else {
        mp = sliding_window_->get_or_create_map_point_candidate(id);
      }

      frame->add_observation(i, id, bearings[j]);

      FrameCamId frame_cam_id{frame->id(), i};
      mp->add_observation(frame_cam_id, bearings[j]);
    }
  }

  size_t kpt_num            = frame->tracking_result_ptr()->size(0);
  float  connected_mp_ratio = kpt_num > 0 ? static_cast<float>(connected)
                                             / static_cast<float>(kpt_num)
                                          : 1.0f;
  LogD("frame {}, connected map point ratio : {} = {} /{}",
       frame->id(),
       connected_mp_ratio,
       connected,
       kpt_num);

  return connected_mp_ratio;
}

bool StereoVO::fetch_result(OdometryResult& out) {
  std::lock_guard<std::mutex> lock(result_mutex_);
  if (!has_result_) {
    return false;
  }
  out = latest_result_;
  return true;
}

int StereoVO::initialize_map_points(std::shared_ptr<Frame>& frame) {
  // triangulate
  auto& candidates = sliding_window_->map_point_candidates();

  // Inverse-distance upper bound accepted for a newly triangulated point
  // (i.e. a minimum depth of ~1/3 m).
  constexpr double kMaxTriangulatedInvDist = 3.0;

  int init_count              = 0;
  int stale_candidate_count   = 0;
  int initial_candidate_count = candidates.size();

  FrameCamId         frame_cam_id0{frame->id(), 0};
  std::set<uint64_t> erase_mp_ids;

  // add map points in SlidingWindow
  for (auto& [mp_id, mp] : candidates) {
    auto& frame_id_to_bearing = mp->observation();

    if (frame_id_to_bearing.count(frame_cam_id0) == 0) {
      stale_candidate_count++;
      erase_mp_ids.insert(mp_id);
      continue;
    }

    Eigen::Vector3d bearing0 = frame_id_to_bearing[frame_cam_id0];

    bool success = false;
    for (auto& [frame_cam_id1, bearing1] : frame_id_to_bearing) {
      if (frame_cam_id0 == frame_cam_id1) {
        continue;
      }

      std::shared_ptr<Frame> frame1 =
        sliding_window_->frame(frame_cam_id1.frame_id);

      auto T_w_c0  = frame->twc(frame_cam_id0.cam_id);
      auto T_w_c1  = frame1->twc(frame_cam_id1.cam_id);
      auto T_c1_c0 = T_w_c1.inverse() * T_w_c0;

      if (T_c1_c0.translation().squaredNorm()
          < SVOConfig::triangulation_dist_threshold) {
        continue;
      }

      Eigen::Vector4d t_c0_x =
        Geometry::triangulate(bearing0, bearing1, T_c1_c0);
      if (t_c0_x.array().isFinite().all() && t_c0_x[3] > 0
          && t_c0_x[3] < kMaxTriangulatedInvDist) {
        mp->bearing()           = t_c0_x.head<3>();
        mp->inv_dist()          = t_c0_x[3];
        mp->host_frame_cam_id() = frame_cam_id0;
        mp->status(MapPoint::Status::TRACKING);
        erase_mp_ids.insert(mp_id);
        sliding_window_->add_map_point(mp);
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
       stale_candidate_count,
       initial_candidate_count,
       candidates.size());

  return init_count;
}

void StereoVO::select_marginal_frames(
  std::set<uint64_t>& marginal_none_keyframe_ids,
  std::set<uint64_t>& marginal_keyframe_ids) {
  marginal_none_keyframe_ids.clear();
  marginal_keyframe_ids.clear();

  // The two most recent keyframes are always kept out of marginalization.
  constexpr size_t kProtectedRecentKeyframes = 2;
  // Guards the reciprocal-distance sum against division by zero when two
  // keyframe positions coincide.
  constexpr double kMinPairwiseDistanceEpsilon = 1e-5;

  const auto& frame_ids    = sliding_window_->frame_ids();
  const auto& keyframe_ids = sliding_window_->keyframe_ids();

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
  std::shared_ptr<Frame>  latest_frame = sliding_window_->frame(latest_id);
  if (latest_frame) {
    const auto& obs = latest_frame->observations().front();
    for (const auto& [mp_id, _] : obs) {
      auto mp = sliding_window_->map_point(mp_id);
      if (!mp || mp->status() < MapPoint::Status::TRACKING) {
        continue;
      }

      connected_map_points[mp->host_frame_cam_id().frame_id]++;
    }
  }

  auto kf_ids = keyframe_ids;
  while (kf_ids.size() > SVOConfig::max_keyframe_size) {
    if (kf_ids.size() <= kProtectedRecentKeyframes) {
      break;
    }

    bool     selected   = false;
    uint64_t id_to_marg = std::numeric_limits<uint64_t>::max();

    auto end_minus_2 = std::prev(kf_ids.end(), kProtectedRecentKeyframes);
    for (auto it = kf_ids.begin(); it != end_minus_2; ++it) {
      const uint64_t kf_id   = *it;
      const int      count   = connected_map_points[kf_id];
      int            created = created_map_point_nums_[kf_id];

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
        std::shared_ptr<Frame> frame_i = sliding_window_->frame(*it1);
        if (!frame_i) {
          continue;
        }
        double denom = 0.0;
        for (auto it2 = kf_ids.begin(); it2 != end_minus_2; ++it2) {
          std::shared_ptr<Frame> frame_j = sliding_window_->frame(*it2);
          if (!frame_j) {
            continue;
          }
          denom +=
            1.0
            / ((frame_i->twb().translation() - frame_j->twb().translation())
                 .norm()
               + kMinPairwiseDistanceEpsilon);
        }

        std::shared_ptr<Frame> last_kf = sliding_window_->frame(last_kf_id);
        if (!last_kf) {
          continue;
        }
        double score = std::sqrt((frame_i->twb().translation()
                                  - last_kf->twb().translation())
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

OdometryResult StereoVO::build_odometry_result(
  const std::shared_ptr<Frame>& frame) {
  TrackingResult* tracking_result = frame->tracking_result_ptr();

  OdometryResult result;
  result.frame_id     = frame->id();
  result.timestamp_ns = frame->timestamp_ns();

  const size_t cam_num = frame->cam_num();
  result.T_b_c.reserve(cam_num);
  for (size_t i = 0; i < cam_num; i++) {
    result.T_b_c.push_back(frame->tbc(i));
  }

  const auto& window_ids = sliding_window_->frame_ids();
  result.window_frame_ids.assign(window_ids.begin(), window_ids.end());
  result.T_w_b_window.reserve(result.window_frame_ids.size());
  for (const auto frame_id : result.window_frame_ids) {
    std::shared_ptr<Frame> window_frame = sliding_window_->frame(frame_id);
    if (window_frame) {
      result.T_w_b_window.push_back(window_frame->twb());
    }
    else {
      result.T_w_b_window.emplace_back();
    }
  }

  result.images.reserve(cam_num);
  result.tracking.ids.resize(cam_num);
  result.tracking.uvs.resize(cam_num);
  for (size_t i = 0; i < cam_num; ++i) {
    result.images.push_back(frame->image(i));
    result.tracking.ids[i] = tracking_result->ids(i);
    result.tracking.uvs[i] = tracking_result->uvs(i);
  }

  const auto& map_points = sliding_window_->map_points();
  result.map_points.reserve(map_points.size());
  result.map_point_uvs.resize(cam_num);
  std::vector<CameraModelBase*> cams(cam_num, nullptr);
  std::vector<Sophus::SE3d>     T_c_w(cam_num);
  std::vector<cv::Size>         img_sizes(cam_num);
  for (size_t i = 0; i < cam_num; ++i) {
    cams[i]            = frame->cam(i);
    T_c_w[i]           = frame->twc(i).inverse();
    const cv::Mat& img = frame->image(i);
    img_sizes[i]       = cv::Size(img.cols, img.rows);
    result.map_point_uvs[i].reserve(map_points.size());
  }
  for (const auto& [mp_id, mp] : map_points) {
    const double inv_dist = mp->inv_dist();
    if (inv_dist <= 0.0) {
      continue;
    }

    std::shared_ptr<Frame> host_frame =
      sliding_window_->frame(mp->host_frame_cam_id().frame_id);
    if (!host_frame) {
      continue;
    }

    const Eigen::Vector3d bearing = mp->bearing();
    const Eigen::Vector3d p_c     = bearing / inv_dist;
    const Eigen::Vector3d p_w     = host_frame->twc(0) * p_c;

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
      const cv::Point2d uv       = cams[i]->project(p_c);
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
