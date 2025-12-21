#pragma once

#include "odometry/pipeline.hpp"

namespace omni_slam {

class VOPipeline : public OdometryPipeline {
public:
  VOPipeline() = default;

  bool Initialize() override;
  void Run() override;
  void Shutdown() override;
};

}  //namespace omni_slam
