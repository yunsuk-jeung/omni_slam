#include <array>
#include <chrono>
#include <filesystem>
#include <thread>

#include <opencv2/core.hpp>

#include "utils/logger.hpp"
#include "device/dataset_simulator.hpp"
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

  omni_slam::StereoVO   stereo_vo;
  std::filesystem::path config_path = project_root / "configs/svo.json";

  if (!stereo_vo.Initialize(config_path.string())) {
    LogE("Failed to initialize VO pipeline");
    return -1;
  }

  omni_slam::DatasetSimulator simulator(loader);
  simulator.SetCameraCallback([&stereo_vo](const std::array<cv::Mat, 2>& images) {
    stereo_vo.OnCameraFrame(images);
  });

  simulator.Start();
  stereo_vo.Run();

  while (loader.HasCameraData()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  simulator.Stop();

  stereo_vo.Shutdown();
  LogI("VO application finished");

  return 0;
}
