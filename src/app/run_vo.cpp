#include <filesystem>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include "core/utils/logger.hpp"
#include "dataset_loader/euroc_loader.hpp"
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

  const auto frame = loader.GetNextCameraFrame();

  LogI("cam0 path: {}", frame.cam0_image_path);

  const auto cam0 = cv::imread(frame.cam0_image_path, cv::IMREAD_ANYCOLOR);

  if (cam0.empty()) {
    LogE("Failed to load cam0 image: {}", frame.cam0_image_path);
    return -1;
  }

  cv::imshow("cam0", cam0);

  if (loader.IsStereo() && !frame.cam1_image_path.empty()) {
    const auto cam1 = cv::imread(frame.cam1_image_path, cv::IMREAD_ANYCOLOR);
    if (!cam1.empty()) {
      cv::imshow("cam1", cam1);
    }
    else {
      LogW("Failed to load cam1 image: {}", frame.cam1_image_path);
    }
  }

  cv::waitKey(0);

  omni_slam::StereoVO pipeline;

  if (!pipeline.Initialize()) {
    LogE("Failed to initialize VO pipeline");
    return -1;
  }

  while (true) {
    pipeline.Run();
    // Process state...
    break;  // Remove this in actual implementation
  }

  pipeline.Shutdown();
  LogI("VO application finished");

  return 0;
}
