#include <fstream>

#include "device/euroc_loader.hpp"
#include "core/utils/logger.hpp"

namespace omni_slam {

bool EurocLoader::Initialize(const std::string& dataset_path) {
  dataset_path_ = dataset_path;

  // Extract dataset name from path
  std::filesystem::path path(dataset_path);
  dataset_name_ = path.filename().string();

  Logger::Info("Initializing EuRoC loader for dataset: {}", dataset_name_);

  // Validate dataset structure
  if (!ValidateDatasetStructure()) {
    LogE("Invalid EuRoC dataset structure at: {}", dataset_path_);
    return false;
  }

  // Parse camera data (cam0 required, cam1 optional for stereo)
  std::string cam0_csv = dataset_path_ + "/mav0/cam0/data.csv";
  if (!ParseCameraCsv(cam0_csv, 0)) {
    LogE("Failed to parse cam0 data");
    return false;
  }

  std::string cam1_csv = dataset_path_ + "/mav0/cam1/data.csv";
  if (std::filesystem::exists(cam1_csv)) {
    if (!ParseCameraCsv(cam1_csv, 1)) {
      Logger::Warn("Failed to parse cam1 data, using mono mode");
      cam1_data_.clear();
      is_stereo_ = false;
    }
    else {
      is_stereo_ = true;
      Logger::Info("Stereo camera data loaded");
    }
  }
  else {
    Logger::Info("Mono camera data loaded (cam1 not found)");
    is_stereo_ = false;
  }

  // Parse IMU data (required)
  std::string imu_csv = dataset_path_ + "/mav0/imu0/data.csv";
  if (!ParseImuCsv(imu_csv)) {
    LogE("Failed to parse IMU data");
    return false;
  }

  // Parse ground truth data (optional)
  std::string gt_csv = dataset_path_ + "/mav0/state_groundtruth_estimate0/data.csv";
  if (std::filesystem::exists(gt_csv)) {
    if (!ParseGroundTruthCsv(gt_csv)) {
      Logger::Warn("Failed to parse ground truth data, continuing without GT");
      ground_truth_data_.clear();
    }
    else {
      Logger::Info("Ground truth data loaded");
    }
  }
  else {
    Logger::Info("Ground truth data not found (optional)");
  }

  // Compute timestamp range
  ComputeTimestampRange();

  // Reset iterators
  Reset();

  initialized_ = true;

  Logger::Info("EuRoC dataset loaded successfully:");
  Logger::Info("  Camera frames: {}", GetCameraFrameCount());
  Logger::Info("  IMU measurements: {}", GetImuMeasurementCount());
  Logger::Info("  Ground truth poses: {}", GetGroundTruthPoseCount());
  Logger::Info("  Stereo: {}", is_stereo_ ? "Yes" : "No");

  return true;
}

void EurocLoader::Reset() {
  camera_index_       = 0;
  imu_index_          = 0;
  ground_truth_index_ = 0;
}

bool EurocLoader::HasCameraData() const {
  return camera_index_ < cam0_data_.size();
}

CameraFrame EurocLoader::GetNextCameraFrame() {
  if (!HasCameraData()) {
    LogE("getNextCameraFrame() called but no more camera data available");
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

size_t EurocLoader::GetCameraFrameCount() const {
  return cam0_data_.size();
}

bool EurocLoader::HasImuData() const {
  return imu_index_ < imu_data_.size();
}

ImuData EurocLoader::GetNextImuMeasurement() {
  if (!HasImuData()) {
    LogE("getNextImuMeasurement() called but no more IMU data available");
    return ImuData{};
  }

  return imu_data_[imu_index_++];
}

size_t EurocLoader::GetImuMeasurementCount() const {
  return imu_data_.size();
}

bool EurocLoader::HasGroundTruthData() const {
  return ground_truth_index_ < ground_truth_data_.size();
}

GroundTruthPose EurocLoader::GetNextGroundTruthPose() {
  if (!HasGroundTruthData()) {
    LogE("getNextGroundTruthPose() called but no more ground truth data available");
    return GroundTruthPose{};
  }

  return ground_truth_data_[ground_truth_index_++];
}

size_t EurocLoader::GetGroundTruthPoseCount() const {
  return ground_truth_data_.size();
}

int64_t EurocLoader::GetStartTimestampNs() const {
  return start_timestamp_ns_;
}

int64_t EurocLoader::GetEndTimestampNs() const {
  return end_timestamp_ns_;
}

std::string EurocLoader::GetDatasetName() const {
  return dataset_name_;
}

bool EurocLoader::IsStereo() const {
  return is_stereo_;
}

bool EurocLoader::ParseCameraCsv(const std::string& csv_path, int cam_id) {
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
      Logger::Warn("Malformed camera CSV line {} in {}", line_number, csv_path);
      continue;
    }

    try {
      int64_t t_ns = std::stoll(timestamp_str);

      // Construct full path
      std::string full_path = dataset_path_ + "/mav0/cam" + std::to_string(cam_id)
                              + "/data/" + filename;

      // Store in appropriate vector
      if (cam_id == 0) {
        cam0_data_.emplace_back(t_ns, full_path);
      }
      else if (cam_id == 1) {
        cam1_data_.emplace_back(t_ns, full_path);
      }
    } catch (const std::exception& e) {
      Logger::Warn("Failed to parse camera CSV line {}: {}", line_number, e.what());
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
      Logger::Warn("Stereo cameras have different frame counts: cam0={}, cam1={}",
                   cam0_data_.size(),
                   cam1_data_.size());
    }
  }

  return true;
}

bool EurocLoader::ParseImuCsv(const std::string& csv_path) {
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

    // Parse line: timestamp,wx,wy,wz,ax,ay,az
    std::stringstream        ss(line);
    std::string              token;
    std::vector<std::string> tokens;

    while (std::getline(ss, token, ',')) {
      tokens.push_back(token);
    }

    if (tokens.size() < 7) {
      Logger::Warn("Malformed IMU CSV line {} in {}", line_number, csv_path);
      continue;
    }

    try {
      ImuData imu;
      imu.t_ns = std::stoll(tokens[0]);
      imu.acc  = Eigen::Vector3d(std::stod(tokens[1]),
                                std::stod(tokens[2]),
                                std::stod(tokens[3]));
      imu.gyr  = Eigen::Vector3d(std::stod(tokens[4]),
                                std::stod(tokens[5]),
                                std::stod(tokens[6]));

      imu_data_.push_back(imu);
    } catch (const std::exception& e) {
      Logger::Warn("Failed to parse IMU CSV line {}: {}", line_number, e.what());
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

bool EurocLoader::ParseGroundTruthCsv(const std::string& csv_path) {
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
      Logger::Warn("Malformed ground truth CSV line {} in {}", line_number, csv_path);
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
      Logger::Warn("Failed to parse ground truth CSV line {}: {}", line_number, e.what());
      continue;
    }
  }

  file.close();

  if (ground_truth_data_.empty()) {
    Logger::Warn("No ground truth data loaded from {}", csv_path);
    return false;
  }

  return true;
}

void EurocLoader::ComputeTimestampRange() {
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

bool EurocLoader::ValidateDatasetStructure() {
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
