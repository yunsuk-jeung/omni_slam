#include <fstream>

#include "device/euroc_loader.hpp"
#include "utils/logger.hpp"

namespace omni_slam {

bool EurocLoader::setup(const std::string& dataset_path) {
  dataset_path_ = dataset_path;

  // Extract dataset name from path
  std::filesystem::path path(dataset_path);
  dataset_name_ = path.filename().string();

  Logger::info("Initializing EuRoC loader for dataset: {}", dataset_name_);

  // Validate dataset structure
  if (!validate_dataset_structure()) {
    LogE("Invalid EuRoC dataset structure at: {}", dataset_path_);
    return false;
  }

  // Parse camera data (cam0 required, cam1 optional for stereo)
  std::string cam0_csv = dataset_path_ + "/mav0/cam0/data.csv";
  if (!parse_camera_csv(cam0_csv, 0)) {
    LogE("Failed to parse cam0 data");
    return false;
  }

  std::string cam1_csv = dataset_path_ + "/mav0/cam1/data.csv";
  if (std::filesystem::exists(cam1_csv)) {
    if (!parse_camera_csv(cam1_csv, 1)) {
      Logger::warn("Failed to parse cam1 data, using mono mode");
      cam1_data_.clear();
      is_stereo_ = false;
    }
    else {
      is_stereo_ = true;
      Logger::info("Stereo camera data loaded");
    }
  }
  else {
    Logger::info("Mono camera data loaded (cam1 not found)");
    is_stereo_ = false;
  }

  // Parse IMU data (required)
  std::string imu_csv = dataset_path_ + "/mav0/imu0/data.csv";
  if (!parse_imu_csv(imu_csv)) {
    LogE("Failed to parse IMU data");
    return false;
  }

  // Parse ground truth data (optional)
  std::string gt_csv = dataset_path_
                       + "/mav0/state_groundtruth_estimate0/data.csv";
  if (std::filesystem::exists(gt_csv)) {
    if (!parse_ground_truth_csv(gt_csv)) {
      Logger::warn("Failed to parse ground truth data, continuing without GT");
      ground_truth_data_.clear();
    }
    else {
      Logger::info("Ground truth data loaded");
    }
  }
  else {
    Logger::info("Ground truth data not found (optional)");
  }

  // Compute timestamp range
  compute_timestamp_range();

  // Reset iterators
  reset();

  initialized_ = true;

  Logger::info("EuRoC dataset loaded successfully:");
  Logger::info("  Camera frames: {}", camera_frame_count());
  Logger::info("  IMU measurements: {}", imu_measurement_count());
  Logger::info("  Ground truth poses: {}", ground_truth_pose_count());
  Logger::info("  Stereo: {}", is_stereo_ ? "Yes" : "No");

  return true;
}

void EurocLoader::reset() {
  camera_index_       = 0;
  imu_index_          = 0;
  ground_truth_index_ = 0;
}

bool EurocLoader::has_camera_data() const {
  return camera_index_ < cam0_data_.size();
}

CameraFrame EurocLoader::get_next_camera_frame() {
  if (!has_camera_data()) {
    LogE("get_next_camera_frame() called but no more camera data available");
    return CameraFrame{};
  }

  CameraFrame frame;
  frame.t_ns            = cam0_data_[camera_index_].first;
  frame.cam0_image_path = cam0_data_[camera_index_].second;

  if (is_stereo_ && camera_index_ < cam1_data_.size()) {
    frame.cam1_image_path = cam1_data_[camera_index_].second;
  }

  camera_index_++;
  return frame;
}

size_t EurocLoader::camera_frame_count() const {
  return cam0_data_.size();
}

bool EurocLoader::has_imu_data() const {
  return imu_index_ < imu_data_.size();
}

ImuData EurocLoader::get_next_imu_measurement() {
  if (!has_imu_data()) {
    LogE("get_next_imu_measurement() called but no more IMU data available");
    return ImuData{};
  }

  return imu_data_[imu_index_++];
}

size_t EurocLoader::imu_measurement_count() const {
  return imu_data_.size();
}

bool EurocLoader::has_ground_truth_data() const {
  return ground_truth_index_ < ground_truth_data_.size();
}

GroundTruthPose EurocLoader::get_next_ground_truth_pose() {
  if (!has_ground_truth_data()) {
    LogE("get_next_ground_truth_pose() called but no more ground truth data "
         "available");
    return GroundTruthPose{};
  }

  return ground_truth_data_[ground_truth_index_++];
}

size_t EurocLoader::ground_truth_pose_count() const {
  return ground_truth_data_.size();
}

int64_t EurocLoader::start_timestamp_ns() const {
  return start_timestamp_ns_;
}

int64_t EurocLoader::end_timestamp_ns() const {
  return end_timestamp_ns_;
}

std::string EurocLoader::dataset_name() const {
  return dataset_name_;
}

bool EurocLoader::is_stereo() const {
  return is_stereo_;
}

bool EurocLoader::parse_camera_csv(const std::string& csv_path, int cam_id) {
  std::ifstream file(csv_path);
  if (!file.is_open()) {
    LogE("Failed to open camera CSV: {}", csv_path);
    return false;
  }

  std::string line;
  int         line_number = 0;

  // Read lines
  while (std::getline(file, line)) {
    line_number++;
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }

    // Skip header lines (start with '#')
    if (line.empty() || line[0] == '#') {
      continue;
    }

    // Parse line: timestamp,filename
    std::stringstream ss(line);
    std::string       timestamp_str, filename;

    if (!std::getline(ss, timestamp_str, ',') || !std::getline(ss, filename)) {
      Logger::warn("Malformed camera CSV line {} in {}", line_number, csv_path);
      continue;
    }

    try {
      int64_t t_ns = std::stoll(timestamp_str);

      // Construct full path
      std::string full_path = dataset_path_ + "/mav0/cam"
                              + std::to_string(cam_id) + "/data/" + filename;

      // Store in appropriate vector
      if (cam_id == 0) {
        cam0_data_.emplace_back(t_ns, full_path);
      }
      else if (cam_id == 1) {
        cam1_data_.emplace_back(t_ns, full_path);
      }
    } catch (const std::exception& e) {
      Logger::warn("Failed to parse camera CSV line {}: {}",
                   line_number,
                   e.what());
      continue;
    }
  }

  file.close();

  if (cam_id == 0 && cam0_data_.empty()) {
    LogE("No camera data loaded from {}", csv_path);
    return false;
  }

  if (cam_id == 1 && cam1_data_.empty()) {
    LogE("No camera data loaded from {}", csv_path);
    return false;
  }

  // Verify stereo timestamp alignment if both cameras loaded
  if (cam_id == 1 && !cam0_data_.empty() && !cam1_data_.empty()) {
    if (cam0_data_.size() != cam1_data_.size()) {
      Logger::warn(
        "Stereo cameras have different frame counts: cam0={}, cam1={}",
        cam0_data_.size(),
        cam1_data_.size());
    }
  }

  return true;
}

bool EurocLoader::parse_imu_csv(const std::string& csv_path) {
  std::ifstream file(csv_path);
  if (!file.is_open()) {
    LogE("Failed to open IMU CSV: {}", csv_path);
    return false;
  }

  std::string line;
  int         line_number = 0;

  // Read lines
  while (std::getline(file, line)) {
    line_number++;
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }

    // Skip header lines (start with '#')
    if (line.empty() || line[0] == '#') {
      continue;
    }

    // Parse line (EuRoC): timestamp,wx,wy,wz,ax,ay,az
    std::stringstream        ss(line);
    std::string              token;
    std::vector<std::string> tokens;

    while (std::getline(ss, token, ',')) {
      tokens.push_back(token);
    }

    if (tokens.size() < 7) {
      Logger::warn("Malformed IMU CSV line {} in {}", line_number, csv_path);
      continue;
    }

    try {
      ImuData imu;
      imu.t_ns = std::stoll(tokens[0]);
      imu.gyr  = Eigen::Vector3d(std::stod(tokens[1]),
                                std::stod(tokens[2]),
                                std::stod(tokens[3]));
      imu.acc  = Eigen::Vector3d(std::stod(tokens[4]),
                                std::stod(tokens[5]),
                                std::stod(tokens[6]));

      imu_data_.push_back(imu);
    } catch (const std::exception& e) {
      Logger::warn("Failed to parse IMU CSV line {}: {}",
                   line_number,
                   e.what());
      continue;
    }
  }

  file.close();

  if (imu_data_.empty()) {
    LogE("No IMU data loaded from {}", csv_path);
    return false;
  }

  return true;
}

bool EurocLoader::parse_ground_truth_csv(const std::string& csv_path) {
  std::ifstream file(csv_path);
  if (!file.is_open()) {
    LogE("Failed to open ground truth CSV: {}", csv_path);
    return false;
  }

  std::string line;
  int         line_number = 0;

  // Read lines
  while (std::getline(file, line)) {
    line_number++;
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }

    // Skip header lines (start with '#')
    if (line.empty() || line[0] == '#') {
      continue;
    }

    // Parse line: timestamp,px,py,pz,qw,qx,qy,qz,...
    std::stringstream        ss(line);
    std::string              token;
    std::vector<std::string> tokens;

    while (std::getline(ss, token, ',')) {
      tokens.push_back(token);
    }

    if (tokens.size() < 8) {
      Logger::warn("Malformed ground truth CSV line {} in {}",
                   line_number,
                   csv_path);
      continue;
    }

    try {
      GroundTruthPose gt;
      gt.t_ns        = std::stoll(tokens[0]);
      gt.position    = Eigen::Vector3d(std::stod(tokens[1]),
                                    std::stod(tokens[2]),
                                    std::stod(tokens[3]));
      gt.orientation = Eigen::Quaterniond(std::stod(tokens[4]),
                                          std::stod(tokens[5]),
                                          std::stod(tokens[6]),
                                          std::stod(tokens[7]));

      ground_truth_data_.push_back(gt);
    } catch (const std::exception& e) {
      Logger::warn("Failed to parse ground truth CSV line {}: {}",
                   line_number,
                   e.what());
      continue;
    }
  }

  file.close();

  if (ground_truth_data_.empty()) {
    Logger::warn("No ground truth data loaded from {}", csv_path);
    return false;
  }

  return true;
}

void EurocLoader::compute_timestamp_range() {
  int64_t min_ts = std::numeric_limits<int64_t>::max();
  int64_t max_ts = std::numeric_limits<int64_t>::min();

  // Check camera data
  if (!cam0_data_.empty()) {
    min_ts = std::min(min_ts, cam0_data_.front().first);
    max_ts = std::max(max_ts, cam0_data_.back().first);
  }

  // Check IMU data
  if (!imu_data_.empty()) {
    min_ts = std::min(min_ts, imu_data_.front().t_ns);
    max_ts = std::max(max_ts, imu_data_.back().t_ns);
  }

  // Check ground truth data
  if (!ground_truth_data_.empty()) {
    min_ts = std::min(min_ts, ground_truth_data_.front().t_ns);
    max_ts = std::max(max_ts, ground_truth_data_.back().t_ns);
  }

  start_timestamp_ns_ = min_ts;
  end_timestamp_ns_   = max_ts;
}

bool EurocLoader::validate_dataset_structure() {
  namespace fs = std::filesystem;

  // Check if dataset path exists
  if (!fs::exists(dataset_path_)) {
    LogE("Dataset path does not exist: {}", dataset_path_);
    return false;
  }

  // Check if mav0 directory exists
  std::string mav0_path = dataset_path_ + "/mav0";
  if (!fs::exists(mav0_path)) {
    LogE("mav0 directory not found: {}", mav0_path);
    return false;
  }

  // Check if cam0 data exists (required)
  std::string cam0_csv = mav0_path + "/cam0/data.csv";
  if (!fs::exists(cam0_csv)) {
    LogE("cam0 data.csv not found: {}", cam0_csv);
    return false;
  }

  // Check if imu0 data exists (required)
  std::string imu_csv = mav0_path + "/imu0/data.csv";
  if (!fs::exists(imu_csv)) {
    LogE("imu0 data.csv not found: {}", imu_csv);
    return false;
  }

  return true;
}

}  // namespace omni_slam
