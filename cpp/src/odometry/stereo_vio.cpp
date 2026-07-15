#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <set>
#include <unordered_map>
#include <unordered_set>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "config/svio_config.hpp"
#include "database/frame.hpp"
#include "database/map_point.hpp"
#include "feature_tracking/optical_flow.hpp"
#include "odometry/sliding_window.hpp"
#include "omni_slam/stereo_vio.hpp"
#include "optimizer/geometry.hpp"
#include "optimizer/vio_estimator.hpp"
#include "utils/logger.hpp"
#include "utils/omni_assert.hpp"
#include "utils/timer.hpp"

namespace omni_slam {
namespace {

ImuData interpolate_imu_data(const ImuData& imu0,
                             const ImuData& imu1,
                             int64_t        timestamp_ns) {
  if (imu0.t_ns == imu1.t_ns) {
    ImuData out = imu0;
    out.t_ns    = timestamp_ns;
    return out;
  }

  const double alpha =
    std::clamp(static_cast<double>(timestamp_ns - imu0.t_ns)
                 / static_cast<double>(imu1.t_ns - imu0.t_ns),
               0.0,
               1.0);
  ImuData out;
  out.t_ns = timestamp_ns;
  out.acc  = (1.0 - alpha) * imu0.acc + alpha * imu1.acc;
  out.gyr  = (1.0 - alpha) * imu0.gyr + alpha * imu1.gyr;
  return out;
}

}  // namespace

StereoVIO::StereoVIO()
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
  , latest_result_{}
  , imu_queue_{kMaxImuQueueSize}
  , imu_data_buffer_{}
  , has_last_frame_imu_{false}
  , last_frame_imu_{}
  , inertial_states_{}
  , imu_parameters{} {
  sliding_window_ = std::make_unique<SlidingWindow>();
  estimator_      = std::make_unique<VIOEstimator>();
}

StereoVIO::~StereoVIO() {}

bool StereoVIO::setup(const std::string& config_path) {
  Logger::info("Initializing VIO Pipeline");
  if (config_path.empty()) {
    Logger::warn("Empty config path for VIO pipeline");
    return false;
  }

  SVIOConfig::ParseConfig(config_path);
  Logger::info("Loaded VIO config: {}", config_path.c_str());

  imu_parameters.acc_noise_sigma      = SVIOConfig::acc_noise_density;
  imu_parameters.gyr_noise_sigma      = SVIOConfig::gyr_noise_density;
  imu_parameters.acc_bias_rw_sigma    = SVIOConfig::acc_random_walk;
  imu_parameters.gyr_bias_rw_sigma    = SVIOConfig::gyr_random_walk;
  imu_parameters.min_integration_dt_s = SVIOConfig::imu_min_integration_dt_s;

  sliding_window_->max_size(SVIOConfig::max_keyframe_size + 1u);
  optical_flow_ =
    std::make_unique<OpticalFlow>(kCamNum, frame_queue_, result_queue_);

  return true;
}

void StereoVIO::run() {
  Logger::info("Running VIO Pipeline");
  running_.store(true, std::memory_order_release);

  optical_flow_thread_ = std::thread(&StereoVIO::optical_flow_loop, this);
  estimator_thread_    = std::thread(&StereoVIO::estimator_loop, this);
}

void StereoVIO::shutdown() {
  Logger::info("Shutting down VIO Pipeline");
  running_.store(false, std::memory_order_release);

  if (optical_flow_thread_.joinable()) {
    optical_flow_thread_.join();
  }
  if (estimator_thread_.joinable()) {
    estimator_thread_.join();
  }
}

void StereoVIO::on_camera_frame(
  int64_t                             timestamp_ns,
  const std::vector<cv::Mat>&         images,
  const std::vector<CameraParameter>& camera_parameters) {
  if (images.empty() || images[0].empty()) {
    Logger::warn("Received camera frame with empty left image");
    return;
  }

  auto frame = std::make_shared<Frame>(timestamp_ns, images, camera_parameters);
  frame_queue_.push(frame);
}

void StereoVIO::on_imu_data(const ImuData& imu_data) {
  imu_queue_.push(imu_data);
}

void StereoVIO::optical_flow_loop() {
  optical_flow_->run(running_);
}

void StereoVIO::estimator_loop() {
  std::shared_ptr<Frame> frame;
  while (running_.load(std::memory_order_acquire)) {
    if (!result_queue_.try_pop(frame)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    const int64_t frame_ts_ns = frame->timestamp_ns();
    pop_imu_data_until(frame_ts_ns, imu_data_buffer_);
    process(frame, imu_data_buffer_);
  }
}

void StereoVIO::process(std::shared_ptr<Frame>&     frame,
                        const std::vector<ImuData>& imu_data) {
  switch (status_) {
  case Status::Initializing:
    if (initialize(frame, imu_data)) {
      status_ = Status::Tracking;
    }
    break;
  case Status::Tracking:
    track(frame, imu_data);
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

  // Statistics::reportAll();
}

void StereoVIO::pop_imu_data_until(int64_t               timestamp_ns,
                                   std::vector<ImuData>& imu_data) {
  imu_data.clear();

  auto append_sample = [&](const ImuData& sample) {
    if (imu_data.empty() || sample.t_ns > imu_data.back().t_ns) {
      imu_data.push_back(sample);
    }
  };

  if (has_last_frame_imu_) {
    append_sample(last_frame_imu_);
  }

  const auto is_before = [&](const ImuData& sample) {
    return sample.t_ns <= timestamp_ns;
  };

  ImuData imu;
  while (imu_queue_.try_pop_if(imu, is_before)) {
    append_sample(imu);
  }

  if (imu_data.empty()) {
    return;
  }

  // The first sample past timestamp_ns stays queued for the next frame and
  // serves as the interpolation endpoint at the frame boundary.
  ImuData frame_imu = imu_data.back();
  ImuData next;
  if (frame_imu.t_ns < timestamp_ns && imu_queue_.try_peek(next)) {
    frame_imu = interpolate_imu_data(frame_imu, next, timestamp_ns);
    imu_data.push_back(frame_imu);
  }

  last_frame_imu_     = frame_imu;
  has_last_frame_imu_ = true;
}

bool StereoVIO::initialize(std::shared_ptr<Frame>&     frame,
                           const std::vector<ImuData>& imu_data) {
  ScopedTimer loop_timer("stereo_vio::Initialize");

  sliding_window_->add_frame(frame);

  update_frame_observations(frame);

  int created_map_point_num = initialize_map_points(frame);

  if (created_map_point_num < SVIOConfig::min_init_map_point_count) {
    Logger::warn(
      "StereoVIO initialization failed at frame {} (map points: {}, required: "
      "{}), resetting sliding window",
      frame->id(),
      created_map_point_num,
      SVIOConfig::min_init_map_point_count);

    sliding_window_->clear();
    inertial_states_.clear();
    imu_preintegrations_.clear();
    created_map_point_nums_.clear();

    new_keyframe_after_ = SVIOConfig::new_keyframe_after + 1;

    return false;
  }

  created_map_point_nums_[frame->id()] = created_map_point_num;
  frame->keyframe();
  sliding_window_->mark_keyframe(frame->id());

  OMNI_ASSERT(!imu_data.empty());

  // use first imu as a gravity direction
  const Eigen::Vector3d& acc0_b       = imu_data.back().acc;
  const double           acc0_norm    = acc0_b.norm();
  const double           gravity_norm = SVIOConfig::g_w.norm();
  const double           kEps         = 1e-9;

  if (acc0_norm > kEps) {
    const Sophus::SE3d    T_w_b          = frame->twb();
    const Eigen::Vector3d measured_dir_w = (T_w_b.so3() * acc0_b).normalized();
    const Eigen::Vector3d target_dir_w   = -SVIOConfig::g_w / gravity_norm;
    const Eigen::Quaterniond q_w_align =
      Eigen::Quaterniond::FromTwoVectors(measured_dir_w, target_dir_w);
    const Sophus::SO3d R_w_align(q_w_align.normalized());
    frame->twb(Sophus::SE3d(R_w_align * T_w_b.so3(), T_w_b.translation()));
  }
  else {
    Logger::warn(
      "Skip gravity alignment at frame {} due to invalid norm (acc: {}, g: {})",
      frame->id(),
      acc0_norm,
      gravity_norm);
  }

  inertial_states_[frame->id()] = InertialState{};

  new_keyframe_after_ = 0;

  Logger::info("stereoVIO initialized at frame {}, map_point_count: {}",
               frame->id(),
               sliding_window_->map_point_count());

  return true;
}

void StereoVIO::track(std::shared_ptr<Frame>&     frame,
                      const std::vector<ImuData>& imu_data) {
  ScopedTimer loop_timer("loop_timer_tracking");

  std::shared_ptr<Frame> latest_frame;
  InertialState          predicted_inertial_state;

  // IMU-based prediction from the latest frame state.
  const auto&    frame_ids = sliding_window_->frame_ids();
  const uint64_t latest_id = *frame_ids.rbegin();
  latest_frame             = sliding_window_->frame(latest_id);

  frame->twb()               = latest_frame->twb();
  const auto latest_state_it = inertial_states_.find(latest_id);
  OMNI_ASSERT_MESSAGE(latest_state_it != inertial_states_.end(),
                      "latest frame has no inertial state");
  predicted_inertial_state = latest_state_it->second;

  OMNI_ASSERT(imu_data.size() >= 2);

  ImuPreintegration preintegration(latest_id,
                                   frame->id(),
                                   predicted_inertial_state.bias_acc,
                                   predicted_inertial_state.bias_gyr,
                                   imu_parameters);

  if (preintegration.integrate_measurements(imu_data)) {
    const double          dt_sec  = preintegration.delta_time_sec();
    const Sophus::SE3d&   T_w_b_i = latest_frame->twb();
    const Sophus::SO3d    R_w_b_j = T_w_b_i.so3() * preintegration.delta_r();
    const Eigen::Vector3d g_w     = SVIOConfig::g_w;
    const Eigen::Vector3d t_w_b_j =
      T_w_b_i.translation() + predicted_inertial_state.v_w_b * dt_sec
      + 0.5 * g_w * dt_sec * dt_sec + T_w_b_i.so3() * preintegration.delta_p();
    const Eigen::Vector3d v_w_b_j = predicted_inertial_state.v_w_b
                                    + g_w * dt_sec
                                    + T_w_b_i.so3() * preintegration.delta_v();
    frame->twb(Sophus::SE3d(R_w_b_j, t_w_b_j));
    predicted_inertial_state.v_w_b = v_w_b_j;
    imu_preintegrations_.insert_or_assign(preintegration.from_frame_id(),
                                          std::move(preintegration));
    inertial_states_[frame->id()] = predicted_inertial_state;
  }

  sliding_window_->add_frame(frame);

  float connect_mp_ratio = update_frame_observations(frame);

  if (connect_mp_ratio < SVIOConfig::keyframe_min_mp_ratio) {
    make_keyframe_ = true;
  }

  if (make_keyframe_ && new_keyframe_after_ > SVIOConfig::new_keyframe_after) {
    int created_map_point_num = initialize_map_points(frame);

    created_map_point_nums_[frame->id()] = created_map_point_num;
    frame->keyframe();

    new_keyframe_after_ = 0;
    sliding_window_->mark_keyframe(frame->id());
  }
  else {
    ++new_keyframe_after_;
  }

  // // single frame pose estimation
  // {
  //   ScopedTimer timer("optimize_frame");
  //   VIOEstimator::OptimizeSingleFrame(frame, this->sliding_window_.get());
  // }
  LogE("frame id : {}", frame->id());

  // sliding window bundle
  {
    ScopedTimer timer("optimize_window");
    estimator_->optimize_window(this->sliding_window_.get(),
                                inertial_states_,
                                imu_preintegrations_);
  }

  // Re-integrate preintegrations with the freshly optimized bias.
  // The cost function only applies a first-order correction around the bias
  // captured at construction; replaying the integration with the updated
  // bias keeps the next window's deltas/Jacobians consistent and prevents
  // intermittent bias spikes from stale linearization.
  for (auto& [from_id, preintegration] : imu_preintegrations_) {
    const auto state_it = inertial_states_.find(from_id);
    if (state_it == inertial_states_.end()) {
      continue;
    }
    preintegration.repropagate(state_it->second.bias_acc,
                               state_it->second.bias_gyr);
  }

  // select marginal frames
  std::set<uint64_t> marginal_frame_ids;
  std::set<uint64_t> marginal_inertial_state_ids;
  select_marginal_frames(marginal_frame_ids, marginal_inertial_state_ids);

  {
    auto join_ids = [](const std::set<uint64_t>& ids) {
      std::string s;
      for (const auto id : ids) {
        if (!s.empty()) {
          s += ",";
        }
        s += std::to_string(id);
      }
      return s;
    };
    LogI("margin frame_ids   : [{}]", join_ids(marginal_frame_ids));
    LogI("margin inertial_ids: [{}]", join_ids(marginal_inertial_state_ids));
  }
  // marginalize
  ScopedTimer timer("marginalize ");
  estimator_->marginalize(this->sliding_window_.get(),
                          marginal_frame_ids,
                          marginal_inertial_state_ids,
                          inertial_states_,
                          imu_preintegrations_);

  // remove keyframe
  {
    ScopedTimer timer("remove keyframe");
    sliding_window_->remove_frames(marginal_frame_ids);

    for (const auto& id : marginal_frame_ids) {
      created_map_point_nums_.erase(id);
    }
    for (const auto& id : marginal_inertial_state_ids) {
      inertial_states_.erase(id);
      OMNI_ASSERT_MESSAGE(inertial_states_.count(id) == 0,
                          "marginal inertial state not removed");
      imu_preintegrations_.erase(id);
      OMNI_ASSERT_MESSAGE(imu_preintegrations_.count(id) == 0,
                          "marginal preintegration not removed");
    }
  }
}

float StereoVIO::update_frame_observations(std::shared_ptr<Frame>& frame) {
  TrackingResult* tracking_result = frame->tracking_result_ptr();
  const size_t    kCamNum         = frame->cam_num();
  size_t          connected       = 0;

  for (size_t i = 0; i < kCamNum; ++i) {
    auto&                        ids = tracking_result->ids(i);
    auto&                        uvs = tracking_result->uvs(i);
    std::vector<Eigen::Vector3d> bearings;
    std::vector<bool>            valid;

    frame->cam(i)->unproject(uvs, bearings, valid);

    const auto& point_num = tracking_result->size(i);
    for (size_t j = 0; j < point_num; j++) {
      if (!valid[j]) {
        continue;
      }
      const auto&     id = ids[j];
      std::shared_ptr mp = sliding_window_->map_point(ids[j]);
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

bool StereoVIO::fetch_result(OdometryResult& out) {
  std::lock_guard<std::mutex> lock(result_mutex_);
  if (!has_result_) {
    return false;
  }
  out = latest_result_;
  return true;
}

int StereoVIO::initialize_map_points(std::shared_ptr<Frame>& frame) {
  // triangulate
  auto& candidates = sliding_window_->map_point_candidates();

  int init_count = 0;
  int old_count  = 0;
  int try_count  = candidates.size();

  FrameCamId         frame_cam_id0{frame->id(), 0};
  std::set<uint64_t> erase_mp_ids;

  // add map points in SlidingWindow
  for (auto& [mp_id, mp] : candidates) {
    auto& frame_id_to_bearing = mp->observation();

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

      std::shared_ptr<Frame> frame1 =
        sliding_window_->frame(frame_cam_id1.frame_id);
      if (!frame1) {
        continue;
      }

      auto T_w_c0  = frame->twc(frame_cam_id0.cam_id);
      auto T_w_c1  = frame1->twc(frame_cam_id1.cam_id);
      auto T_c1_c0 = T_w_c1.inverse() * T_w_c0;

      if (T_c1_c0.translation().squaredNorm()
          < SVIOConfig::triangulation_dist_threshold) {
        continue;
      }

      Eigen::Vector4d t_c0_x =
        Geometry::triangulate(bearing0, bearing1, T_c1_c0);
      if (t_c0_x.array().isFinite().all() && t_c0_x[3] > 0 && t_c0_x[3] < 3.0) {
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

  // LogD("init : {}, oldCount :{}, cand : {} -> {}",
  //      init_count,
  //      old_count,
  //      try_count,
  //      candidates.size());

  return init_count;
}

void StereoVIO::select_marginal_frames(
  std::set<uint64_t>& marginal_frame_ids,
  std::set<uint64_t>& marginal_inertial_state_ids) {
  marginal_frame_ids.clear();
  marginal_inertial_state_ids.clear();

  const auto& frame_ids    = sliding_window_->frame_ids();
  const auto& keyframe_ids = sliding_window_->keyframe_ids();
  if (frame_ids.empty()) {
    return;
  }

  // Phase 1: select oldest inertial states that exceed max_inertial_states.
  if (inertial_states_.size() > SVIOConfig::max_inertial_states) {
    const size_t num_inertial_states_to_marg =
      inertial_states_.size() - SVIOConfig::max_inertial_states;
    auto inertial_state_it = inertial_states_.begin();
    for (size_t i = 0; i < num_inertial_states_to_marg;
         ++i, ++inertial_state_it) {
      marginal_inertial_state_ids.insert(inertial_state_it->first);
    }
  }

  auto has_inertial_state = [&](uint64_t frame_id) {
    return inertial_states_.count(frame_id) > 0;
  };

  // Phase 3: marginalize non-keyframes that have no inertial state.
  // Frames whose inertial state is being marginalized this step must wait
  // until the inertial state is actually removed before the frame can go.
  for (const auto id : frame_ids) {
    if (keyframe_ids.count(id) > 0) {
      continue;
    }
    if (!has_inertial_state(id)) {
      marginal_frame_ids.insert(id);
    }
  }

  // Phase 4: keyframe overflow - only keyframes without retained inertial
  // states are eligible for removal.
  if (keyframe_ids.size() > SVIOConfig::max_keyframe_size) {
    std::map<uint64_t, int> connected_map_points;
    const uint64_t          latest_frame_id = *frame_ids.rbegin();
    std::shared_ptr<Frame>  latest_frame =
      sliding_window_->frame(latest_frame_id);
    if (latest_frame && !latest_frame->observations().empty()) {
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
    while (kf_ids.size() > SVIOConfig::max_keyframe_size) {
      bool     selected   = false;
      uint64_t id_to_marg = std::numeric_limits<uint64_t>::max();
      if (kf_ids.size() <= 1) {
        break;
      }

      const size_t keep_tail_count =
        std::min(SVIOConfig::max_inertial_states + 1u, kf_ids.size() - 1u);

      const auto end_minus_inertial_states = std::prev(kf_ids.end(),
                                                       keep_tail_count);
      if (end_minus_inertial_states == kf_ids.begin()) {
        break;
      }

      for (auto it = kf_ids.begin(); it != end_minus_inertial_states; ++it) {
        const uint64_t kf_id = *it;
        if (has_inertial_state(kf_id)) {
          continue;
        }

        const int  count      = connected_map_points[kf_id];
        const auto created_it = created_map_point_nums_.find(kf_id);
        const int  created    = (created_it == created_map_point_nums_.end())
                                  ? 0
                                  : created_it->second;
        if (created <= 0) {
          id_to_marg = kf_id;
          selected   = true;
          break;
        }
        if (count == 0
            || (static_cast<float>(count) / static_cast<float>(created))
                 < static_cast<float>(
                   SVIOConfig::marg_feature_connection_ratio)) {
          id_to_marg = kf_id;
          selected   = true;
          break;
        }
      }

      if (!selected) {
        const uint64_t last_kf_id   = *kf_ids.rbegin();
        uint64_t       min_score_id = std::numeric_limits<uint64_t>::max();
        double         min_score    = std::numeric_limits<double>::max();

        for (auto it1 = kf_ids.begin(); it1 != end_minus_inertial_states;
             ++it1) {
          if (has_inertial_state(*it1)) {
            continue;
          }

          std::shared_ptr<Frame> frame_i = sliding_window_->frame(*it1);
          if (!frame_i) {
            continue;
          }

          double denom = 0.0;
          for (auto it2 = kf_ids.begin(); it2 != end_minus_inertial_states;
               ++it2) {
            if (has_inertial_state(*it2)) {
              continue;
            }

            std::shared_ptr<Frame> frame_j = sliding_window_->frame(*it2);
            if (!frame_j) {
              continue;
            }
            denom +=
              1.0
              / ((frame_i->twb().translation() - frame_j->twb().translation())
                   .norm()
                 + 1e-5);
          }

          std::shared_ptr<Frame> last_kf = sliding_window_->frame(last_kf_id);
          if (!last_kf) {
            continue;
          }

          const double score = std::sqrt((frame_i->twb().translation()
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
      marginal_frame_ids.insert(id_to_marg);
    }
  }
}

OdometryResult StereoVIO::build_odometry_result(
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

  const auto inertial_it = inertial_states_.find(frame->id());
  if (inertial_it != inertial_states_.end()) {
    result.acc_bias = inertial_it->second.bias_acc;
    result.gyr_bias = inertial_it->second.bias_gyr;
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
