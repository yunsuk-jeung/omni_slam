#include "core/utils/logger.hpp"
#include "odometry/vo_pipeline.hpp"

int main(int argc, char** argv) {
  omni_slam::Logger::info("Starting VO application");

  omni_slam::VOPipeline pipeline;

  if (!pipeline.initialize()) {
    omni_slam::Logger::error("Failed to initialize VO pipeline");
    return -1;
  }

  while (true) {
    pipeline.run();
    //Process state...
    break;  //Remove this in actual implementation
  }

  pipeline.shutdown();
  omni_slam::Logger::info("VO application finished");

  return 0;
}
