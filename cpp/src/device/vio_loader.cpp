#include <filesystem>

#include "device/euroc_loader.hpp"
#include "device/vio_loader.hpp"
#include "utils/logger.hpp"

namespace omni_slam {

std::unique_ptr<VioLoader> VIOLoaderFactory::create_loader(
  const std::string& dataset_path,
  DatasetType        type) {
  // Auto-detect if requested
  if (type == DatasetType::AUTO) {
    LogD("Auto-detecting dataset type for: {}", dataset_path);
    type = detect_dataset_type(dataset_path);

    if (type == DatasetType::AUTO) {
      LogE("Could not auto-detect dataset type for: {}", dataset_path);
      return nullptr;
    }
  }

  // Create loader based on type
  std::unique_ptr<VioLoader> loader;

  switch (type) {
  case DatasetType::EUROC:
    Logger::info("Creating EuRoC dataset loader");
    loader = std::make_unique<EurocLoader>();
    break;

  default:
    LogE("Unsupported dataset type");
    return nullptr;
  }

  // Initialize the loader
  if (loader && !loader->setup(dataset_path)) {
    LogE("Failed to initialize dataset loader");
    return nullptr;
  }

  return loader;
}

DatasetType VIOLoaderFactory::detect_dataset_type(
  const std::string& dataset_path) {
  namespace fs = std::filesystem;

  if (!fs::exists(dataset_path)) {
    LogE("Dataset path does not exist: {}", dataset_path);
    return DatasetType::AUTO;
  }

  // Check for EuRoC dataset structure
  if (is_euroc_dataset(dataset_path)) {
    Logger::info("Detected EuRoC dataset format");
    return DatasetType::EUROC;
  }

  // TODO: add detection for other dataset formats (e.g. KITTI) here.

  Logger::warn("Unknown dataset type at: {}", dataset_path);
  return DatasetType::AUTO;
}

bool VIOLoaderFactory::is_euroc_dataset(const std::string& dataset_path) {
  namespace fs = std::filesystem;

  // Check for mav0 directory (characteristic of EuRoC datasets)
  std::string mav0_path = dataset_path + "/mav0";
  if (!fs::exists(mav0_path)) {
    return false;
  }

  // Check for required subdirectories
  bool has_cam0 = fs::exists(mav0_path + "/cam0");
  bool has_imu0 = fs::exists(mav0_path + "/imu0");

  return has_cam0 && has_imu0;
}

}  // namespace omni_slam
