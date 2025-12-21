#include "odometry/vo_pipeline.hpp"

#include "core/utils/logger.hpp"

namespace omni_slam {

bool VOPipeline::Initialize() {
  Logger::Info("Initializing VO Pipeline");
  return true;
}

void VOPipeline::Run() {
  Logger::Info("Running VO Pipeline");
}

void VOPipeline::Shutdown() {
  Logger::Info("Shutting down VO Pipeline");
}

}  //namespace omni_slam
