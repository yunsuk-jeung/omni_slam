#include "odometry/stereo_vo.hpp"

#include "core/utils/logger.hpp"

namespace omni_slam {

bool StereoVO::Initialize() {
  Logger::Info("Initializing VO Pipeline");
  return true;
}

void StereoVO::Run() {
  Logger::Info("Running VO Pipeline");
}

void StereoVO::Shutdown() {
  Logger::Info("Shutting down VO Pipeline");
}

}  // namespace omni_slam
