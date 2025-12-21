#include <filesystem>

#include "core/utils/logger.hpp"
#include "dataset_loader/vio_loader.hpp"
#include "dataset_loader/euroc_loader.hpp"

namespace omni_slam {

std::unique_ptr<VioLoader> VIOLoaderFactory::CreateLoader(const std::string& dataset_path,
                                                          DatasetType        type) {
  // Auto-detect if requested
  if (type == DatasetType::AUTO) {
    Logger::Info("Auto-detecting dataset type for: " + dataset_path);
    type = DetectDatasetType(dataset_path);

    if (type == DatasetType::AUTO) {
      Logger::Error("Could not auto-detect dataset type for: " + dataset_path);
      return nullptr;
    }
  }

  // Create loader based on type
  std::unique_ptr<VioLoader> loader;

  switch (type) {
  case DatasetType::EUROC:
    Logger::Info("Creating EuRoC dataset loader");
    loader = std::make_unique<EurocLoader>();
    break;

  default:
    Logger::Error("Unsupported dataset type");
    return nullptr;
  }

  // Initialize the loader
  if (loader && !loader->Initialize(dataset_path)) {
    Logger::Error("Failed to initialize dataset loader");
    return nullptr;
  }

  return loader;
}

DatasetType VIOLoaderFactory::DetectDatasetType(const std::string& dataset_path) {
  namespace fs = std::filesystem;

  if (!fs::exists(dataset_path)) {
    Logger::Error("Dataset path does not exist: " + dataset_path);
    return DatasetType::AUTO;
  }

  // Check for EuRoC dataset structure
  if (IsEurocDataset(dataset_path)) {
    Logger::Info("Detected EuRoC dataset format");
    return DatasetType::EUROC;
  }

  // Future dataset type detection can be added here:
  // if (IsKittiDataset(dataset_path)) {
  //   Logger::Info("Detected KITTI dataset format");
  //   return DatasetType::KITTI;
  // }

  Logger::Warn("Unknown dataset type at: " + dataset_path);
  return DatasetType::AUTO;
}

bool VIOLoaderFactory::IsEurocDataset(const std::string& dataset_path) {
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
