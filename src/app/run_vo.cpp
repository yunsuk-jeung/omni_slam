#include "core/utils/logger.hpp"
#include "odometry/vo_pipeline.hpp"

int main(int argc, char** argv) {
  omni_slam::Logger::Info("Starting VO application");

  omni_slam::VOPipeline pipeline;

  if (!pipeline.Initialize()) {
    omni_slam::Logger::Error("Failed to initialize VO pipeline");
    return -1;
  }

  while (true) {
    pipeline.Run();
    //Process state...
    break;  //Remove this in actual implementation
  }

  pipeline.Shutdown();
  omni_slam::Logger::Info("VO application finished");

  return 0;
}
