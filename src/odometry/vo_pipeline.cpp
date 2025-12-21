#include "odometry/vo_pipeline.hpp"

#include "core/utils/logger.hpp"

namespace omni_slam {

bool VOPipeline::initialize() {
  Logger::info("Initializing VO Pipeline");
  return true;
}

void VOPipeline::run() {
  Logger::info("Running VO Pipeline");
}

void VOPipeline::shutdown() {
  Logger::info("Shutting down VO Pipeline");
}

}  //namespace omni_slam