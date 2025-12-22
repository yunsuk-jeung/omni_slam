#include <filesystem>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include "core/utils/logger.hpp"
#include "device/euroc_loader.hpp"
#include "odometry/stereo_vo.hpp"

int main(int argc, char** argv) {
  LogI("Starting VO application");

  const auto project_root = std::filesystem::path(__FILE__)
                              .parent_path()
                              .parent_path()
                              .parent_path();

  std::filesystem::path dataset_path = project_root
                                       / "datasets/EUROC/vicon_room1/V1_01_easy";

  omni_slam::EurocLoader loader;
  if (!loader.Initialize(dataset_path.string())) {
    LogE("Failed to initialize EuRoC loader");
    return -1;
  }

  if (!loader.HasCameraData()) {
    LogE("No camera data available in dataset");
    return -1;
  }

  omni_slam::StereoVO stereo_vo;

  if (!stereo_vo.Initialize()) {
    LogE("Failed to initialize VO pipeline");
    return -1;
  }

  while (true) {
    stereo_vo.Run();
    // Process state...
    break;  // Remove this in actual implementation
  }

  stereo_vo.Shutdown();
  LogI("VO application finished");

  return 0;
}
