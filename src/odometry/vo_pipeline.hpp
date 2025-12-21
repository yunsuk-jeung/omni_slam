#pragma once

#include "odometry/pipeline.hpp"

namespace omni_slam {

class VOPipeline : public OdometryPipeline {
public:
  VOPipeline() = default;

  bool initialize() override;
  void run() override;
  void shutdown() override;
};

}  //namespace omni_slam
