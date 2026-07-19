#include <cmath>
#include <fstream>

#include <nlohmann/json.hpp>

#include "config/svo_config.hpp"
#include "omni_slam/types.hpp"
#include "utils/logger.hpp"

namespace omni_slam {
bool             SVOConfig::debug                             = false;
bool             SVOConfig::tbb                               = true;
bool             SVOConfig::equalize_histogram                = false;
double           SVOConfig::clahe_clip_limit                  = 3.0;
int              SVOConfig::clahe_tile_size                   = 8;
int              SVOConfig::optical_flow_patch_size           = 21;
float            SVOConfig::optical_flow_dist_threshold       = 5.0;
int              SVOConfig::fast_threshold                    = 20;
int              SVOConfig::feature_grid_rows                 = 4;
int              SVOConfig::feature_grid_cols                 = 4;
int              SVOConfig::max_pyramid_level                 = 3;
size_t           SVOConfig::max_keyframe_size                 = 8;
float            SVOConfig::keyframe_min_mp_ratio             = 0.8f;
size_t           SVOConfig::min_init_map_point_count          = 20;
float            SVOConfig::marg_feature_connection_ratio     = 0.2f;
int              SVOConfig::new_keyframe_after                = 1;
double           SVOConfig::triangulation_dist_threshold      = 0.0025;
double           SVOConfig::bearing_huber_const               = 0.01;
double           SVOConfig::bearing_cost_scale                = 1.0;
int              SVOConfig::single_frame_max_iterations       = 10;
int              SVOConfig::window_max_iterations             = 10;
int              SVOConfig::window_num_threads                = 2;
double           SVOConfig::inv_dist_initial_value            = 1e-3;
double           SVOConfig::inv_dist_min_value                = 1e-6;
double           SVOConfig::marginalizer_initial_prior_weight = 1e10;
std::vector<int> SVOConfig::camera_models;
std::vector<std::vector<double>> SVOConfig::camera_intrinsics;
std::vector<std::vector<double>> SVOConfig::camera_distortions;
std::vector<std::vector<int>>    SVOConfig::camera_resolutions;
std::vector<Sophus::SE3d>        SVOConfig::camera_T_b_c;

static void parse_camera_params(const nlohmann::json&             node,
                                std::vector<int>*                 models,
                                std::vector<std::vector<double>>* intrinsics,
                                std::vector<std::vector<double>>* distortions,
                                std::vector<std::vector<int>>*    resolutions,
                                std::vector<Sophus::SE3d>*        T_bc) {
  if (!node.is_object()) {
    return;
  }

  if (models) {
    models->push_back(
      node.value("camera_model",
                 static_cast<int>(CameraModel::PINHOLE_RAD_TAN)));
  }

  if (intrinsics) {
    std::vector<double> vals;
    if (node.contains("intrinsics") && node["intrinsics"].is_array()) {
      vals.reserve(node["intrinsics"].size());
      for (const auto& v : node["intrinsics"]) {
        vals.push_back(v.get<double>());
      }
    }
    intrinsics->push_back(std::move(vals));
  }

  if (distortions) {
    std::vector<double> vals;
    if (node.contains("distortions") && node["distortions"].is_array()) {
      vals.reserve(node["distortions"].size());
      for (const auto& v : node["distortions"]) {
        vals.push_back(v.get<double>());
      }
    }
    distortions->push_back(std::move(vals));
  }

  if (resolutions) {
    std::vector<int> vals;
    if (node.contains("resolution") && node["resolution"].is_array()) {
      vals.reserve(node["resolution"].size());
      for (const auto& v : node["resolution"]) {
        vals.push_back(v.get<int>());
      }
    }
    resolutions->push_back(std::move(vals));
  }
  constexpr int kSE3MatrixElements = 16;  // flattened 4x4
  if (T_bc && node.contains("T_b_c") && node["T_b_c"].is_array()
      && node["T_b_c"].size() == kSE3MatrixElements) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (int i = 0; i < kSE3MatrixElements; ++i) {
      T(i / 4, i % 4) = node["T_b_c"][i].get<double>();
    }
    T_bc->push_back(Sophus::SE3d(T));
  }
  else if (T_bc) {
    T_bc->push_back(Sophus::SE3d());
  }
}

void SVOConfig::parse_config(const std::string& file) {
  std::ifstream input(file);
  if (!input.is_open()) {
    return;
  }

  nlohmann::json config;
  try {
    input >> config;
  } catch (const std::exception&) {
    LogE("json parsing error");
    return;
  }

  debug = config.value("debug", debug);
  if (debug) {
    Logger::init();
    spdlog::set_level(spdlog::level::debug);
  }
  tbb                = config.value("tbb", tbb);
  equalize_histogram = config.value("equalize_histogram", equalize_histogram);
  clahe_clip_limit   = config.value("clahe_clip_limit", clahe_clip_limit);
  clahe_tile_size    = config.value("clahe_tile_size", clahe_tile_size);
  optical_flow_patch_size     = config.value("optical_flow_patch_size",
                                         optical_flow_patch_size);
  optical_flow_dist_threshold = config.value("optical_flow_dist_threshold",
                                             optical_flow_dist_threshold);
  fast_threshold              = config.value("fast_threshold", fast_threshold);
  feature_grid_rows = config.value("feature_grid_rows", feature_grid_rows);
  feature_grid_cols = config.value("feature_grid_cols", feature_grid_cols);
  max_pyramid_level = config.value("max_pyramid_level", max_pyramid_level);
  max_keyframe_size = config.value("max_keyframe_size", max_keyframe_size);
  if (max_keyframe_size < 3) {
    max_keyframe_size = 3;
  }
  triangulation_dist_threshold  = config.value("triangulation_dist_threshold",
                                              triangulation_dist_threshold);
  keyframe_min_mp_ratio         = config.value("keyframe_min_mp_ratio",
                                       keyframe_min_mp_ratio);
  min_init_map_point_count      = config.value("min_init_map_point_count",
                                          min_init_map_point_count);
  marg_feature_connection_ratio = config.value("marg_feature_connection_ratio",
                                               marg_feature_connection_ratio);
  new_keyframe_after  = config.value("new_keyframe_after", new_keyframe_after);
  bearing_huber_const = config.value("bearing_huber_const",
                                     bearing_huber_const);
  bearing_cost_scale = config.value("bearing_cost_scale", bearing_cost_scale);
  single_frame_max_iterations = config.value("single_frame_max_iterations",
                                             single_frame_max_iterations);
  window_max_iterations       = config.value("window_max_iterations",
                                       window_max_iterations);
  window_num_threads = config.value("window_num_threads", window_num_threads);
  inv_dist_initial_value = config.value("inv_dist_initial_value",
                                        inv_dist_initial_value);
  inv_dist_min_value = config.value("inv_dist_min_value", inv_dist_min_value);
  marginalizer_initial_prior_weight =
    config.value("marginalizer_initial_prior_weight",
                 marginalizer_initial_prior_weight);

  if (bearing_huber_const <= 0.0) {
    bearing_huber_const = 0.01;
  }
  if (!std::isfinite(bearing_cost_scale) || bearing_cost_scale <= 0.0) {
    bearing_cost_scale = 1.0;
  }
  if (single_frame_max_iterations < 1) {
    single_frame_max_iterations = 1;
  }
  if (window_max_iterations < 1) {
    window_max_iterations = 1;
  }
  if (window_num_threads < 1) {
    window_num_threads = 1;
  }
  if (min_init_map_point_count < 1) {
    min_init_map_point_count = 1;
  }
  if (inv_dist_min_value <= 0.0) {
    inv_dist_min_value = 1e-6;
  }
  if (inv_dist_initial_value <= inv_dist_min_value) {
    inv_dist_initial_value = inv_dist_min_value * 10.0;
  }
  if (!std::isfinite(marginalizer_initial_prior_weight)
      || marginalizer_initial_prior_weight <= 0.0) {
    marginalizer_initial_prior_weight = 1e10;
  }

  camera_models.clear();
  camera_intrinsics.clear();
  camera_distortions.clear();
  camera_resolutions.clear();
  camera_T_b_c.clear();
  if (config.contains("cam0")) {
    parse_camera_params(config["cam0"],
                        &camera_models,
                        &camera_intrinsics,
                        &camera_distortions,
                        &camera_resolutions,
                        &camera_T_b_c);
  }
  if (config.contains("cam1")) {
    parse_camera_params(config["cam1"],
                        &camera_models,
                        &camera_intrinsics,
                        &camera_distortions,
                        &camera_resolutions,
                        &camera_T_b_c);
  }

  if (debug) {
    Logger::info("SVOConfig.debug: {}", debug);
    Logger::info("SVOConfig.tbb: {}", tbb);
    Logger::info("SVOConfig.equalize_histogram: {}", equalize_histogram);
    Logger::info("SVOConfig.clahe_clip_limit: {}", clahe_clip_limit);
    Logger::info("SVOConfig.clahe_tile_size: {}", clahe_tile_size);
    Logger::info("SVOConfig.optical_flow_patch_size: {}",
                 optical_flow_patch_size);
    Logger::info("SVOConfig.optical_flow_dist_threshold: {}",
                 optical_flow_dist_threshold);
    Logger::info("SVOConfig.fast_threshold: {}", fast_threshold);
    Logger::info("SVOConfig.feature_grid_rows: {}", feature_grid_rows);
    Logger::info("SVOConfig.feature_grid_cols: {}", feature_grid_cols);
    Logger::info("SVOConfig.max_pyramid_level: {}", max_pyramid_level);
    Logger::info("SVOConfig.max_keyframe_size: {}", max_keyframe_size);
    Logger::info("SVOConfig.triangulation_dist_threshold: {}",
                 triangulation_dist_threshold);
    Logger::info("SVOConfig.keyframe_min_mp_ratio: {}", keyframe_min_mp_ratio);
    Logger::info("SVOConfig.min_init_map_point_count: {}",
                 min_init_map_point_count);
    Logger::info("SVOConfig.marg_feature_connection_ratio: {}",
                 marg_feature_connection_ratio);
    Logger::info("SVOConfig.new_keyframe_after: {}", new_keyframe_after);
    Logger::info("SVOConfig.bearing_huber_const: {}", bearing_huber_const);
    Logger::info("SVOConfig.bearing_cost_scale: {}", bearing_cost_scale);
    Logger::info("SVOConfig.single_frame_max_iterations: {}",
                 single_frame_max_iterations);
    Logger::info("SVOConfig.window_max_iterations: {}", window_max_iterations);
    Logger::info("SVOConfig.window_num_threads: {}", window_num_threads);
    Logger::info("SVOConfig.inv_dist_initial_value: {}",
                 inv_dist_initial_value);
    Logger::info("SVOConfig.inv_dist_min_value: {}", inv_dist_min_value);
    Logger::info("SVOConfig.marginalizer_initial_prior_weight: {}",
                 marginalizer_initial_prior_weight);
    Logger::info("SVOConfig.camera_models: {}", camera_models.size());
  }
}
}  // namespace omni_slam
